// sd2n4g-motor.c — standalone pan/tilt stepper tool for the Zenointel SD-2N-4G
// (Goke GK7205V510, armv7, Linux 4.9.37). No vendor driver, no app: drives the
// two 4-wire steppers directly over memory-mapped GPIO, so it works under
// OpenIPC (where motor.ko/gpioStep.ko/hunter are absent) and on stock (with the
// vendor app stopped). Modeled on the Xiongmai ms41908_lens.c bench tool.
//
// Everything here is ported from the stock firmware:
//   GPIO   : Goke/HiSilicon PL061 blocks at 0x120B0000 + bank*0x1000 (banks 0-9).
//            masked data write: *(base + (4<<pin)) = level<<pin ; DIR at +0x400.
//            pinmux (IOCFG) at 0x100C0000 / 0x112C0000 ; nibble 0 = GPIO.
//   Motors : from the device's live devcfg "motor"+"motorCfg":
//              PAN  ("roll",  1640 steps, dir 0): GPIO [3,4,72,73],  timer 2
//              TILT ("pitch",  580 steps, dir 1): GPIO [69,59,58,57], timer 3
//            coil order from cfg "line"=[0,2,1,3] -> pCtl[line[i]] = gpio[i], i.e.
//              PAN  phase cols = [3,72,4,73] ; TILT phase cols = [69,58,59,57].
//   Step   : gpioStep.ko half-step 8-phase table (verbatim), one 4-bit write per
//            tick; period_us = 1e6/speed, halved for half-step. speed ~150..600.
//            Open-loop step counting (no home/PI sensor wired on this unit).
//
// Build:  make            (arm-openipc-linux-musleabi-gcc, static)
// Run  :  on OpenIPC nothing else owns the pins — just run it. On STOCK, direct
//         register writes still move the motors while hunter runs (as long as no
//         PTZ command is in flight); for exclusive control stop the vendor owner
//         and disarm the watchdog first:
//            printf V > /dev/watchdog   # disarm (if not NOWAYOUT)
//            killall hunter ; rmmod zoomfocus gpioStep motor   # optional but clean
//         then, on-device:
//            ./sd2n4g-motor                    interactive jog (l/r pan, u/d tilt, arrows)
//            ./sd2n4g-motor -d l -x 400 -s 400 pan left 400 steps
//            ./sd2n4g-motor -d r -a 45         pan right 45 degrees
//            ./sd2n4g-motor -d u -y 200        tilt up 200 steps
//            ./sd2n4g-motor -d p               read pin state, no motion (safe w/ app up)
//            ./sd2n4g-motor -d s               de-energize both motors' coils

#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <termios.h>
#include <unistd.h>

/* ---- SoC GPIO (PL061) --------------------------------------------------- */
#define GPIO_BASE(bank) (0x120B0000u + (unsigned)(bank) * 0x1000u)
#define GPIO_DIR(bank)  (GPIO_BASE(bank) + 0x400u)

/* One coil pin: PL061 bank/pin + its pinmux (IOCFG) register (nibble 0 = GPIO). */
typedef struct { uint8_t bank, pin; uint32_t mux; } Pin;

/* One motor: 4 coil pins already in phase-column order (cfg "line" applied),
 * a direction-invert flag (cfg iostepDir), soft step limit, and live phase. */
typedef struct {
  const char *name;
  Pin coil[4];      /* [c0,c1,c2,c3] = the 4 phase-table columns */
  int inv;          /* iostepDir: 1 => swap cw/ccw */
  int steps_max;    /* full mechanical travel (cfg "steps") */
  int span_deg;     /* angular travel over steps_max (for -a degrees) */
  int phase;        /* current index into the phase table */
  long pos;         /* open-loop step counter */
} Motor;

/* PAN = cfg motor[1]/"roll": gpio[3,4,72,73], line[0,2,1,3] -> [3,72,4,73]. */
/* TILT= cfg motor[0]/"pitch": gpio[69,59,58,57], line[0,2,1,3] -> [69,58,59,57]. */
/* Travel/span from the device cfg + calibration: pan 1640 st = 280deg,           */
/* tilt 580 st = 90deg (measured: 400 pan half-steps ~= 68deg).                    */
static Motor motors[2] = {
  { .name = "pan",  .inv = 0, .steps_max = 1640, .span_deg = 280, .coil = {
      {0,3,0x100C000Cu}, {9,0,0x100C009Cu}, {0,4,0x100C0010u}, {9,1,0x100C00A0u} } },
  { .name = "tilt", .inv = 1, .steps_max = 580,  .span_deg = 90,  .coil = {
      {8,5,0x112C004Cu}, {7,2,0x112C0060u}, {7,3,0x112C0064u}, {7,1,0x112C005Cu} } },
};

/* degrees -> half-steps for an axis (relative move magnitude). Clamps to the
 * axis travel BEFORE the cast so a huge/inf input can't overflow long, and maps
 * NaN to 0. motor_step re-clamps too, but the cast must be made safe first. */
static long deg_to_steps(const Motor *m, double deg) {
  if (deg < 0) deg = -deg;
  double st = deg * m->steps_max / m->span_deg + 0.5;
  if (!(st >= 0)) return 0;                 /* NaN */
  if (st > m->steps_max) return m->steps_max;
  return (long)st;
}

/* gpioStep.ko half-step / 8-step table (columns = coil[0..3]), extracted verbatim. */
static const uint8_t HALF[8][4] = {
  {1,0,0,1}, {1,0,0,0}, {1,0,1,0}, {0,0,1,0},
  {0,1,1,0}, {0,1,0,0}, {0,1,0,1}, {0,0,0,1},
};

#define SPEED_MIN 100
#define SPEED_MAX 900
#define SPEED_DEF 400
#define STEP_DEF  100

/* ---- /dev/mem MMIO (page-cached), same pattern as ms41908_lens.c -------- */
static int mem_fd = -1;
#define MAX_PAGES 16
static struct { uint32_t base; volatile uint8_t *p; } pages[MAX_PAGES];

/* A GPIO/pinmux access that cannot map is unrecoverable for a bare-metal tool —
 * fail hard rather than silently pretending the coil moved (a partial write set
 * would leave the motor mid-phase and still report success otherwise). */
static void die(const char *what, uint32_t a) {
  fprintf(stderr, "sd2n4g-motor: %s %#x: %s\n", what, a, strerror(errno));
  exit(1);
}
static volatile uint32_t *reg(uint32_t phys) {
  uint32_t base = phys & ~0xFFFu;
  int i;
  for (i = 0; i < MAX_PAGES && pages[i].p; i++)
    if (pages[i].base == base) return (volatile uint32_t *)(pages[i].p + (phys & 0xFFF));
  if (i == MAX_PAGES) die("page table full for", phys);
  void *m = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, base);
  if (m == MAP_FAILED) die("mmap", base);
  pages[i].base = base; pages[i].p = m;
  return (volatile uint32_t *)(pages[i].p + (phys & 0xFFF));
}
static uint32_t rd(uint32_t a) { return *reg(a); }
static void wr(uint32_t a, uint32_t v) { *reg(a) = v; }

/* PL061 masked write: the address selects the pin, data carries its bit. */
static void gpio_set(const Pin *p, int lvl) {
  wr(GPIO_BASE(p->bank) + (1u << (p->pin + 2)), lvl ? (1u << p->pin) : 0);
}
static int gpio_get(const Pin *p) {
  return (rd(GPIO_BASE(p->bank) + (1u << (p->pin + 2))) >> p->pin) & 1;
}
static void gpio_make_output(const Pin *p) {
  if (p->mux) wr(p->mux, rd(p->mux) & ~0xFu);              /* select GPIO function */
  wr(GPIO_DIR(p->bank), rd(GPIO_DIR(p->bank)) | (1u << p->pin));
}

/* ---- stepping ----------------------------------------------------------- */
static void apply_phase(Motor *m) {
  const uint8_t *row = HALF[m->phase & 7];
  for (int i = 0; i < 4; i++) gpio_set(&m->coil[i], row[i]);
}
/* Recover the phase index from whatever the coils are currently holding, so a
 * one-shot invocation continues from the phase a previous run left energized
 * instead of jumping electrically to phase 0. If the live pattern matches no
 * table row (coils released, or an unknown state), establish phase 0. */
static void motor_recover_phase(Motor *m) {
  uint8_t cur[4];
  for (int i = 0; i < 4; i++) cur[i] = (uint8_t)gpio_get(&m->coil[i]);
  for (int p = 0; p < 8; p++) {
    if (!memcmp(HALF[p], cur, 4)) { m->phase = p; return; }
  }
  m->phase = 0; apply_phase(m);        /* unknown -> known starting phase */
}
static void motor_init_pins(Motor *m) {
  for (int i = 0; i < 4; i++) gpio_make_output(&m->coil[i]);
  motor_recover_phase(m);
}
static void motor_release(Motor *m) {          /* de-energize all coils */
  for (int i = 0; i < 4; i++) gpio_set(&m->coil[i], 0);
}
/* dir: +1 / -1 (before iostepDir). Clamps to the axis travel; returns steps taken. */
static long motor_step(Motor *m, int dir, long steps, int speed) {
  if (speed < SPEED_MIN) speed = SPEED_MIN;
  if (speed > SPEED_MAX) speed = SPEED_MAX;
  if (steps > m->steps_max) {           /* a single move cannot exceed full travel */
    fprintf(stderr, "%s: clamping %ld to travel %d steps\n", m->name, steps, m->steps_max);
    steps = m->steps_max;
  }
  int d = (m->inv ? -dir : dir) >= 0 ? 1 : -1;
  useconds_t period = (useconds_t)(1000000 / speed / 2);   /* half-step */
  long n = 0;
  for (; n < steps; n++) {
    m->phase = (m->phase + d) & 7;
    apply_phase(m);
    m->pos += d;
    usleep(period);
  }
  return n;
}

/* ---- modes -------------------------------------------------------------- */
static void probe(void) {
  puts("pin state (bank/pin : mux level dir):");
  for (unsigned i = 0; i < sizeof(motors)/sizeof(motors[0]); i++) {
    Motor *m = &motors[i];
    printf("  %-4s", m->name);
    for (int c = 0; c < 4; c++) {
      Pin *p = &m->coil[c];
      int dir = (rd(GPIO_DIR(p->bank)) >> p->pin) & 1;
      printf("  [b%d p%d mux=%#x lvl=%d %s]", p->bank, p->pin, p->mux,
             gpio_get(p), dir ? "out" : "in");
    }
    putchar('\n');
  }
}

static void release_all(void) {
  motor_init_pins(&motors[0]); motor_init_pins(&motors[1]);
  motor_release(&motors[0]);   motor_release(&motors[1]);
}

/* Terminal state restored on normal exit AND on catchable termination, so an
 * interrupted jog never leaves the invoking shell in raw / no-echo mode. */
static struct termios g_saved_tio;
static int g_tty_raw = 0;
static void tty_restore(void) {
  if (g_tty_raw) { tcsetattr(STDIN_FILENO, TCSANOW, &g_saved_tio); g_tty_raw = 0; }
}
static void tty_signal(int sig) { tty_restore(); _exit(128 + sig); }

/* Interactive jog. Keys follow the repo PTZ convention: u/d = tilt, l/r = pan. */
static void jog(int speed) {
  motor_init_pins(&motors[0]);
  motor_init_pins(&motors[1]);
  int tty = tcgetattr(STDIN_FILENO, &g_saved_tio) == 0;
  if (tty) {
    struct termios raw = g_saved_tio;
    raw.c_lflag &= ~(ICANON | ECHO | ISIG);
    atexit(tty_restore);
    signal(SIGINT, tty_signal); signal(SIGTERM, tty_signal); signal(SIGHUP, tty_signal);
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    g_tty_raw = 1;
  }
  printf("jog: l/r = pan, u/d = tilt (arrows too), +/- speed, o off, p probe, q quit.  speed=%d\n",
         speed);
  int burst = 20;
  for (unsigned char ch; read(STDIN_FILENO, &ch, 1) > 0;) {
    if (ch == 0x1b) {                 /* arrow keys: ESC [ A/B/C/D */
      unsigned char s[2]; if (read(STDIN_FILENO, s, 2) != 2) continue;
      if (s[0] != '[') continue;
      ch = (s[1]=='D')?'l':(s[1]=='C')?'r':(s[1]=='A')?'u':(s[1]=='B')?'d':0;
    }
    switch (ch) {
      case 'l': motor_step(&motors[0], -1, burst, speed); break;
      case 'r': motor_step(&motors[0], +1, burst, speed); break;
      case 'u': motor_step(&motors[1], +1, burst, speed); break;
      case 'd': motor_step(&motors[1], -1, burst, speed); break;
      case '+': case '=': speed = speed<SPEED_MAX-50?speed+50:SPEED_MAX; printf("speed=%d\n",speed); break;
      case '-': case '_': speed = speed>SPEED_MIN+50?speed-50:SPEED_MIN; printf("speed=%d\n",speed); break;
      case 'o': motor_release(&motors[0]); motor_release(&motors[1]); puts("coils released"); break;
      case 'p': printf("pos pan=%ld tilt=%ld\n", motors[0].pos, motors[1].pos); probe(); break;
      case 'q': case 3: goto out;
      default: break;
    }
  }
out:
  tty_restore();
}

static void usage(const char *a0) {
  fprintf(stderr,
    "usage: %s                       interactive jog (l/r pan, u/d tilt, arrows;\n"
    "                                +/- speed; o off; p probe; q quit)\n"
    "       %s -d <dir> -s <speed> [-x <pan steps> | -y <tilt steps> | -a <deg>]\n"
    "  -d l|r  pan  -/+   (magnitude from -x steps or -a degrees)\n"
    "  -d u|d  tilt +/-   (magnitude from -y steps or -a degrees)\n"
    "  -d s    stop / de-energize both coils\n"
    "  -d p    probe pin state (read-only)\n"
    "  -a <deg> move by degrees instead of steps (pan 280deg, tilt 90deg full travel)\n"
    "  -s speed %d..%d (default %d);  steps default %d; a single move clamps to travel\n",
    a0, a0, SPEED_MIN, SPEED_MAX, SPEED_DEF, STEP_DEF);
}

int main(int argc, char **argv) {
  setvbuf(stdout, NULL, _IONBF, 0);
  mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
  if (mem_fd < 0) { fprintf(stderr, "open /dev/mem: %s\n", strerror(errno)); return 1; }

  char dir = 0; int speed = SPEED_DEF; long xsteps = 0, ysteps = 0; double adeg = 0;
  int have_x = 0, have_y = 0, have_a = 0;
  char *end;
  for (int c; (c = getopt(argc, argv, "d:s:x:y:a:")) != -1; ) {
    switch (c) {
      case 'd': dir = optarg[0]; break;
      case 's': speed = (int)strtol(optarg, &end, 10); if (*end) goto bad; break;
      case 'x': xsteps = strtol(optarg, &end, 10); have_x = 1; if (*end) goto bad; break;
      case 'y': ysteps = strtol(optarg, &end, 10); have_y = 1; if (*end) goto bad; break;
      case 'a': adeg = strtod(optarg, &end); have_a = 1; if (*end || end == optarg) goto bad; break;
      default: goto bad;
    }
  }
  /* reject negative / non-finite magnitudes rather than moving a default amount */
  if (xsteps < 0 || ysteps < 0 || adeg < 0 || adeg != adeg /* NaN */ || adeg > 1e6) {
bad:
    usage(argv[0]); return 2;
  }
  if (!dir) { jog(SPEED_DEF); return 0; }   /* no -d => interactive jog */

  switch (dir) {
    case 'p': probe(); return 0;
    case 's': release_all(); puts("both motors de-energized"); return 0;
    case 'l': case 'r': {
      Motor *m = &motors[0];
      long n = have_a ? deg_to_steps(m, adeg) : have_x ? xsteps : STEP_DEF;
      int d = (dir == 'r') ? +1 : -1;
      motor_init_pins(m); n = motor_step(m, d, n, speed);
      printf("pan %ld steps %s @%d -> pos=%ld\n", n, d>0?"right":"left", speed, m->pos);
      return 0; }
    case 'u': case 'd': {
      Motor *m = &motors[1];
      long n = have_a ? deg_to_steps(m, adeg) : have_y ? ysteps : STEP_DEF;
      int d = (dir == 'u') ? +1 : -1;
      motor_init_pins(m); n = motor_step(m, d, n, speed);
      printf("tilt %ld steps %s @%d -> pos=%ld\n", n, d>0?"up":"down", speed, m->pos);
      return 0; }
    default: usage(argv[0]); return 2;
  }
}
