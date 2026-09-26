// ms41908_lens.c — standalone PTZ lens bench tool for the Xiongmai
// HI3516D_N81820 (HiSilicon Hi3516A V100) + MS41908M (AN41908A clone) lens.
//
// Lens module: "LENS_LH13_FHD_X16" (16x varifocal, IMX291/IMX323/OV4689).
//
// Manual jog + PI home-seek. No MPP/ISP dependency; builds with a bare Hi3516A
// uClibc toolchain. Bus/registers/GPIO map + init/pinmux/interrupt setup and the
// per-move handshake are all ported from the stock firmware's libxmaf.so
// (xmspi_*/ms419x9_init/ms419_plsintr_init/ms9x9_*/motor_*_init):
//
//   SPI    : /dev/spidev1.0, mode 0x0C (SPI_LSB_FIRST|SPI_CS_HIGH), 8-bit, 5 MHz.
//            one {addr,lo,hi} triple per SPI_IOC_MESSAGE; read = {addr|0x40,0,0}.
//   GPIO   : no gpiolib/sysfs on this board — PL061 driven via /dev/mem mmap.
//            EN=GPIO8_7 (HIGH per transfer), VD_FZ=GPIO10_5 (move latch),
//            PI focus=GPIO0_3, PI zoom=GPIO0_4 (inputs).
//   Motion-done interrupt block @ 0x20220400.. (status +0x14, clear +0x1C;
//            zoom=bit1, focus=bit0).
//   Move   : write ctrl (zoom 0x24 / focus 0x29 = (4*step)|0x0400|(dir<<8))
//            -> clear_isr -> VD_FZ pulse -> poll motion-done.  iris DAC = reg 0x00.
//
// Self-contained: it programs the SPI1 pinmux/clock and the lens GPIOs itself
// (plsintr_init below), so it needs no vendor lib. But the *motor* only steps
// while the ISP pipeline is generating VD timing, so a streamer must be running:
//   - OpenIPC : majestic (it drives the pipeline but never touches the MS41908M,
//               so this tool owns the lens IC while video runs). Nothing to stop.
//   - stock XM: Sofia is both streamer and lens driver, so stop its lens half
//               first (it owns /dev/spidev1.0 + the lens GPIOs):
//                 killall Sofia dvrHelper
//               and restore afterwards with:
//                 setsid dvrHelper /lib/modules /usr/bin/Sofia 127.0.0.1 9578 &
//
// Build:  make ms41908-lens-openipc   (OpenIPC musl toolchain; default target)
// Run on-device:
//   ms41908-lens             interactive jog (soft travel limits, see below)
//   ms41908-lens probe       read-only register dump (safe with the streamer up)
//   ms41908-lens home        motion proof: bounded PI home-seek on both axes
//   ms41908-lens watchpi N   read-only PI monitor for N s (safe with streamer up)

#include <errno.h>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <termios.h>
#include <unistd.h>

#define DEVICE_NAME "/dev/spidev1.0"
/* musl's <sys/ioctl.h> does not expose _IOC_SIZEBITS, which
 * <linux/spi/spidev.h>'s SPI_MSGSIZE()/SPI_IOC_MESSAGE() reference; the kernel
 * value is 14. glibc/uClibc define it, so this only fills the musl gap. */
#ifndef _IOC_SIZEBITS
#define _IOC_SIZEBITS 14
#endif

#define PORT_SPEED  5000000
#define SPI_MODE    (SPI_LSB_FIRST | SPI_CS_HIGH) /* 0x0C, matches libxmaf */

/* PL061 GPIO groups (8 pins each), and the pin map recovered from libxmaf. */
#define GPIO_BASE(g) (0x20140000u + (unsigned)(g) * 0x10000u)
#define GPIO_DIR(g)  (GPIO_BASE(g) + 0x400u)
#define EN_GRP  8
#define EN_PIN  7   /* xmspi_enable — HIGH during each SPI transfer */
#define VD_GRP  10
#define VD_PIN  5   /* VD_FZ move latch/trigger */
#define PIF_GRP 0
#define PIF_PIN 3   /* focus photo-interrupter (input) */
#define PIZ_GRP 0
#define PIZ_PIN 4   /* zoom photo-interrupter (input) */

/* Motion-done interrupt block (libxmaf gpio_intr_no_wait / gpio_intr_clera_isr). */
#define INTR_STAT 0x20220414u
#define INTR_CLR  0x2022041Cu
#define ISR_ZOOM  1
#define ISR_FOCUS 0

/* ctrl reg = (4*step) | 0x0400(start) | (dir<<8); PI-enable bit 0x0800 left off. */
#define CTRL_BASE 0x0400
#define REG_IRIS  0x00
#define REG_ZOOM  0x24
#define REG_FOCUS 0x29

/* home-seek: PI "home level" and travel bound per axis (LENS_LH13_FHD_X16 cfg). */
#define ZOOM_HOME_LEVEL 1
#define FOCUS_HOME_LEVEL 0
#define ZOOM_MAX 2610
#define FOCUS_MAX 1280

#define IRIS_MIN 0
#define IRIS_MAX 0x03E8
#define IRIS_DEFAULT 0xC8

static int spi_fd = -1;
static int mem_fd = -1;
static int iris_value = IRIS_DEFAULT;

/* ---- /dev/mem MMIO (page-cached) ---------------------------------------- */

#define MAX_PAGES 16
static struct { uint32_t base; volatile uint8_t *p; } pages[MAX_PAGES];

static volatile uint32_t *reg(uint32_t phys) {
  uint32_t base = phys & ~0xFFFu;
  int i;
  for (i = 0; i < MAX_PAGES && pages[i].p; i++)
    if (pages[i].base == base)
      return (volatile uint32_t *)(pages[i].p + (phys & 0xFFF));
  if (i == MAX_PAGES) return NULL;
  void *m = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, base);
  if (m == MAP_FAILED) {
    fprintf(stderr, "mmap %#x: %s\n", base, strerror(errno));
    return NULL;
  }
  pages[i].base = base;
  pages[i].p = m;
  return (volatile uint32_t *)(pages[i].p + (phys & 0xFFF));
}

static uint32_t mmio_r(uint32_t a) { volatile uint32_t *r = reg(a); return r ? *r : 0; }
static void mmio_w(uint32_t a, uint32_t v) { volatile uint32_t *r = reg(a); if (r) *r = v; }

/* PL061 masked access: address selects the pin, data carries its bit. */
static void gpio_set(int g, int pin, bool lvl) {
  mmio_w(GPIO_BASE(g) + (1u << (pin + 2)), lvl ? (1u << pin) : 0);
}
static int gpio_get(int g, int pin) {
  return (mmio_r(GPIO_BASE(g) + (1u << (pin + 2))) >> pin) & 1;
}
static void vd_fz_pulse(void) {
  gpio_set(VD_GRP, VD_PIN, true);
  usleep(1000);
  gpio_set(VD_GRP, VD_PIN, false);
}
static void clear_isr(int bit) { mmio_w(INTR_CLR, mmio_r(INTR_CLR) | (1u << bit)); }
static int poll_done(int bit, int timeout_us) {
  for (int w = 0; w < timeout_us; w += 2000) {
    if ((mmio_r(INTR_STAT) >> bit) & 1) return 1;
    usleep(2000);
  }
  return 0;
}

/* SoC pin-mux + GPIO direction + interrupt setup, from libxmaf ms419_plsintr_init.
 * IOCONFIG (pad-function) registers live at 0x200F00xx; the SPI1 SCLK/SDO/SDI
 * pads take function 1 (SPI0 is the same pattern at 0x200F0050/54/58). Stock
 * Sofia runs this at lens init; OpenIPC does not, so the tool must, or every
 * SPI read comes back 0. */
static void plsintr_init(void) {
  mmio_w(0x200F0060, 1); mmio_w(0x200F0064, 1); mmio_w(0x200F0068, 1); /* SPI1 SCLK/SDO/SDI = func 1 */
  mmio_w(0x200F006C, 0);
  mmio_w(0x201C0000, mmio_r(0x201C0000) | 0x80u);                     /* SPI1 clock enable */
  mmio_w(GPIO_DIR(EN_GRP), mmio_r(GPIO_DIR(EN_GRP)) | (1u << EN_PIN)); /* EN output */
  gpio_set(EN_GRP, EN_PIN, false);                                    /* xmspi_disable */
  mmio_w(0x200F0150, 1); mmio_w(0x200F014C, 1);
  for (uint32_t a = 0x20220400; a <= 0x20220410; a += 4)             /* intr type/mask */
    mmio_w(a, mmio_r(a) & ~3u);
  mmio_w(INTR_CLR, mmio_r(INTR_CLR) | 3u);                           /* enable 2 lines */
  mmio_w(0x200F00A4, 0);
  mmio_w(GPIO_DIR(VD_GRP), mmio_r(GPIO_DIR(VD_GRP)) | (1u << VD_PIN)); /* VD_FZ output (0x201E0400 |= 0x20) */
  gpio_set(VD_GRP, VD_PIN, false);
  mmio_w(0x200F00E0, 0); mmio_w(0x200F00E4, 0);
  mmio_w(GPIO_DIR(PIF_GRP),
         mmio_r(GPIO_DIR(PIF_GRP)) & ~((1u << PIF_PIN) | (1u << PIZ_PIN))); /* PIs input (0x20140400 &= ~0x18) */
}

/* ---- SPI register access (mirrors xmspi_write / xmspi_read) -------------- */

static void spi_xfer(uint8_t *tx, uint8_t *rx) {
  struct spi_ioc_transfer tr = {
      .tx_buf = (unsigned long)tx, .rx_buf = (unsigned long)rx,
      .len = 3, .speed_hz = PORT_SPEED, .bits_per_word = 8,
  };
  gpio_set(EN_GRP, EN_PIN, true);
  if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0)
    fprintf(stderr, "spi ioctl: %s\n", strerror(errno));
  gpio_set(EN_GRP, EN_PIN, false);
}
static void spi_write(uint8_t addr, uint16_t val) {
  uint8_t tx[3] = {addr, val & 0xff, val >> 8}, rx[3] = {0};
  spi_xfer(tx, rx);
}
static uint16_t spi_read(uint8_t addr) {
  uint8_t tx[3] = {addr | 0x40, 0, 0}, rx[3] = {0};
  spi_xfer(tx, rx);
  return rx[1] | (rx[2] << 8);
}

/* ---- MS41908M init (mirrors libxmaf ms419x9_init) ----------------------- */

static void ms419_init(void) {
  spi_write(0x20, 0x5C02); spi_write(0x22, 0x0001); spi_write(0x27, 0x0001);
  spi_write(0x23, 0xD0D0); spi_write(0x28, 0xD0D0);
  spi_write(0x25, 0x0160); spi_write(0x2A, 0x0160); /* zoom/focus PPS */
  spi_write(0x0B, 0x8480); spi_write(0x21, 0x0087);
  uint16_t r21 = spi_read(0x21), r20 = spi_read(0x20);
  printf("MS41908 init: 0x21=%#06x %s, 0x20=%#06x %s\n",
         r21, r21 == 0x0087 ? "OK" : "FAIL", r20, r20 == 0x5C02 ? "OK" : "FAIL");
}

/* ---- motor moves (full handshake: write ctrl -> clear_isr -> VD_FZ -> poll) */

static void move_axis(uint8_t rgn, int isr_bit, bool dir, int step) {
  if (step < 1) step = 1;
  if (step > 63) step = 63;
  uint16_t ctrl = (uint16_t)((4 * step) | CTRL_BASE | (dir ? 0x0100 : 0));
  spi_write(rgn, ctrl);
  clear_isr(isr_bit);
  vd_fz_pulse();
  if (!poll_done(isr_bit, 60000)) usleep(12000); /* fallback if no done signal */
}
static void zoom(bool tele, int step)  { move_axis(REG_ZOOM,  ISR_ZOOM,  tele, step); }
static void focus(bool far, int step)  { move_axis(REG_FOCUS, ISR_FOCUS, far,  step); }

static void set_iris(int value) {
  if (value < IRIS_MIN) value = IRIS_MIN;
  if (value > IRIS_MAX) value = IRIS_MAX;
  spi_write(REG_IRIS, (uint16_t)value);
  iris_value = value;
  printf("iris set %#x, read %#x\n", value, spi_read(REG_IRIS));
}

/* ---- PI home-seek (Phase 1 of libxmaf motor_*_init) --------------------- *
 * Drives 4-step bursts toward the home flag until the PI GPIO reaches its
 * home level, bounded by the axis travel. A PI transition = physical motion. */
static int home_seek(const char *name, uint8_t rgn, int isr_bit, int pi_grp,
                     int pi_pin, int home_level, int max_units, int cap) {
  (void)home_level; (void)max_units;
  int start = gpio_get(pi_grp, pi_pin);
  printf("%s home-seek: PI start=%d, bidirectional fine sweep (8 microsteps/burst)...\n",
         name, start);
  /* Direction polarity is board-specific (libxmaf XORs the requested dir with a
   * per-axis cfg bit), so just sweep BOTH ways: any PI transition = motion. */
  for (int dir = 0; dir <= 1; dir++) {
    int base = gpio_get(pi_grp, pi_pin);
    for (int u = 0; u < cap; u += 2) {
      move_axis(rgn, isr_bit, dir, 2);
      int pi = gpio_get(pi_grp, pi_pin);
      if (pi != base) {
        printf("%s: PI %d->%d after %d microsteps, dir=%d => MOTION CONFIRMED\n",
               name, base, pi, (u + 2) * 4, dir);
        return 1;
      }
      if ((u / 2) % 25 == 0) printf("  %s dir=%d: %d microsteps, PI=%d\n",
                                    name, dir, u * 4, pi);
    }
    printf("  %s dir=%d: no PI change over %d microsteps\n", name, dir, cap * 4);
  }
  printf("%s: NO motion detected either direction\n", name);
  return 0;
}

/* ---- setup / entry ------------------------------------------------------ */

static int io_open(void) {
  spi_fd = open(DEVICE_NAME, O_RDWR);
  if (spi_fd < 0) { fprintf(stderr, "open %s: %s\n", DEVICE_NAME, strerror(errno)); return -1; }
  uint8_t mode = SPI_MODE, bits = 8;
  uint32_t speed = PORT_SPEED;
  if (ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) < 0 ||
      ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
      ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
    fprintf(stderr, "SPI config: %s\n", strerror(errno)); return -1;
  }
  mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
  if (mem_fd < 0) { fprintf(stderr, "open /dev/mem: %s\n", strerror(errno)); return -1; }
  return 0;
}

/* ---- soft travel limits ------------------------------------------------- *
 * Ranges are the libxmaf motor_config_register() values for this lens
 * (LENS_LH13_FHD_X16): zoom 2610, focus 1280 microsteps end-to-end. There is
 * no working PI home on this board, so "0" is set either by gently jogging to
 * a mechanical end and pressing 'o', or by 'h' (bounded seek into the stop).
 * Once zeroed, jog refuses to drive past 0 or MAX, so it can't ram a stop.
 * NB: dir polarity is board-specific, so the WIDE/TELE labels may be swapped
 * vs the physical lens — the point is the *range* can't be exceeded either way. */
static int z_pos = 0, f_pos = 0, lens_homed = 0;

static void jog_move(bool is_zoom, bool up, int step) {
  int *pos = is_zoom ? &z_pos : &f_pos;
  int max  = is_zoom ? ZOOM_MAX : FOCUS_MAX;
  const char *ax = is_zoom ? "zoom" : "focus";
  const char *hi = is_zoom ? "TELE" : "FAR", *lo = is_zoom ? "WIDE" : "NEAR";
  int nxt = up ? *pos + step : *pos - step;
  if (lens_homed) {
    int room = up ? (max - *pos) : *pos;
    if (room <= 0) { printf("%s at %s limit (%d/%d)\n", ax, up ? hi : lo, *pos, max); return; }
    if (step > room) step = room;
  } else if (nxt > max + 200 || nxt < -(max + 200)) {
    printf("%s UNHOMED backstop (%+d) — jog to a stop and press 'o', or 'h' to home\n", ax, *pos);
    return;
  }
  if (is_zoom) zoom(up, step); else focus(up, step);
  *pos += up ? step : -step;
  printf("%s %s  pos %+d/%d%s\n", ax, up ? hi : lo, *pos, max, lens_homed ? "" : "  (UNHOMED)");
}

static void lens_home(void) { /* bounded seek to the WIDE+NEAR stops = 0 (no PI here) */
  puts("homing: seek to WIDE + NEAR stops to set zero (brief clicking at the stop is normal)...");
  for (int i = 0; i < ZOOM_MAX + 150; i += 15) zoom(false, 15);
  z_pos = 0;
  for (int i = 0; i < FOCUS_MAX + 150; i += 15) focus(false, 15);
  f_pos = 0;
  lens_homed = 1;
  puts("homed: zoom=0 focus=0. Soft limits active — jog can no longer drive past a stop.");
}

int main(int argc, char **argv) {
  setvbuf(stdout, NULL, _IONBF, 0); /* unbuffered: progress visible over telnet */
  const char *mode = argc > 1 ? argv[1] : "jog";

  if (!strcmp(mode, "watchpi")) { /* read-only PI monitor: safe while Sofia is up */
    mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd < 0) { fprintf(stderr, "open /dev/mem: %s\n", strerror(errno)); return 1; }
    int secs = argc > 2 ? atoi(argv[2]) : 15;
    int pf = -1, pz = -1;
    printf("watchpi %ds: reading focus=GPIO0_%d zoom=GPIO0_%d (raw + masked)\n",
           secs, PIF_PIN, PIZ_PIN);
    for (int t = 0; t < secs * 100; t++) {
      int f = gpio_get(PIF_GRP, PIF_PIN), z = gpio_get(PIZ_GRP, PIZ_PIN);
      if (f != pf || z != pz) {
        printf("[%5dms] focus=%d zoom=%d  (GPIO0 raw data=%#010x)\n",
               t * 10, f, z, mmio_r(GPIO_BASE(0) + 0x3FC));
        pf = f; pz = z;
      }
      usleep(10000);
    }
    printf("watchpi done (final focus=%d zoom=%d)\n", pf, pz);
    return 0;
  }

  if (!strcmp(mode, "probe")) { /* read-only: safe while Sofia is up */
    if (io_open() != 0) return 1;
    mmio_w(GPIO_DIR(EN_GRP), mmio_r(GPIO_DIR(EN_GRP)) | (1u << EN_PIN));
    gpio_set(EN_GRP, EN_PIN, false);
    printf("probe: 0x20=%#06x 0x21=%#06x 0x24=%#06x 0x29=%#06x 0x00=%#06x\n",
           spi_read(0x20), spi_read(0x21), spi_read(0x24), spi_read(0x29), spi_read(0x00));
    return 0;
  }

  if (io_open() != 0) return 1;
  if (getenv("NOPLS")) printf("(skipping plsintr_init — using existing pin setup)\n");
  else plsintr_init();
  ms419_init();

  if (!strcmp(mode, "diag")) { /* instrument one axis to locate the failure */
    int n = argc > 2 ? atoi(argv[2]) : 12;
    printf("diag: INTR_STAT(0x%08x) GPIO0raw(0x%08x) zoomPI zoomCtrlRB per zoom move\n",
           INTR_STAT, GPIO_BASE(0) + 0x3FC);
    for (int i = 0; i < n; i++) {
      uint32_t st0 = mmio_r(INTR_STAT), g0 = mmio_r(GPIO_BASE(0) + 0x3FC);
      /* one zoom move, dir=1, 4 microsteps */
      spi_write(REG_ZOOM, (uint16_t)((4 * 1) | CTRL_BASE | 0x0100));
      clear_isr(ISR_ZOOM);
      gpio_set(VD_GRP, VD_PIN, true); usleep(1000); gpio_set(VD_GRP, VD_PIN, false);
      usleep(20000);
      uint32_t st1 = mmio_r(INTR_STAT), g1 = mmio_r(GPIO_BASE(0) + 0x3FC);
      printf("  %2d: STAT %08x->%08x  GPIO0 %08x->%08x  PIz=%d  ctrl0x24=%#06x  VDdir=%#x\n",
             i, st0, st1, g0, g1, (g1 >> PIZ_PIN) & 1, spi_read(0x24),
             mmio_r(GPIO_DIR(VD_GRP)));
    }
    return 0;
  }

  if (!strcmp(mode, "home")) { /* motion proof — needs Sofia stopped */
    int cap = argc > 2 ? atoi(argv[2]) : 800;
    int z = home_seek("zoom", REG_ZOOM, ISR_ZOOM, PIZ_GRP, PIZ_PIN, ZOOM_HOME_LEVEL, ZOOM_MAX, cap);
    int f = home_seek("focus", REG_FOCUS, ISR_FOCUS, PIF_GRP, PIF_PIN, FOCUS_HOME_LEVEL, FOCUS_MAX, cap);
    printf("RESULT: zoom motion=%d focus motion=%d\n", z, f);
    return 0;
  }

  struct termios old, raw;
  int tty = tcgetattr(STDIN_FILENO, &old) == 0;
  if (tty) { raw = old; raw.c_lflag &= ~(ICANON | ECHO | ISIG); tcsetattr(STDIN_FILENO, TCSANOW, &raw); }
  printf("MS41908M lens jog (soft limits zoom=%d focus=%d):\n"
         "  +/- zoom   x/z focus   q/w iris   p status   o zero-here   h home   Ctrl-C quit\n"
         "  UNHOMED: jog to a stop (first click) then press 'o' to enable exact limits.\n",
         ZOOM_MAX, FOCUS_MAX);
  int step = 20;
  for (char ch; read(STDIN_FILENO, &ch, 1) > 0;) {
    switch (ch) {
      case '+': jog_move(true,  true,  step); break;   /* zoom tele */
      case '-': jog_move(true,  false, step); break;   /* zoom wide */
      case 'x': jog_move(false, true,  step); break;   /* focus far */
      case 'z': jog_move(false, false, step); break;   /* focus near */
      case 'q': set_iris(iris_value + 0x80); break;
      case 'w': set_iris(iris_value - 0x80); break;
      case 'o':
        z_pos = f_pos = 0; lens_homed = 1;
        printf("zeroed here; soft limits active: zoom[0,%d] focus[0,%d]\n", ZOOM_MAX, FOCUS_MAX);
        break;
      case 'h': lens_home(); break;
      case 'p':
        printf("pos zoom=%d/%d focus=%d/%d  PI(z=%d f=%d)  homed=%d\n",
               z_pos, ZOOM_MAX, f_pos, FOCUS_MAX,
               gpio_get(PIZ_GRP, PIZ_PIN), gpio_get(PIF_GRP, PIF_PIN), lens_homed);
        break;
      case 3: goto out;
      default: break;
    }
  }
out:
  if (tty) tcsetattr(STDIN_FILENO, TCSANOW, &old);
  return 0;
}
