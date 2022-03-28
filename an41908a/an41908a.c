#include <errno.h>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/param.h>
#include <termios.h>
#include <unistd.h>

#include "hi_type.h"
#include "mpi_isp.h"

#define MAX_LENGTH 64
#define PORT_SPEED 4000000
#define DEVICE_NAME "/dev/spidev2.0"

#define GPIO_RSTB 27
#define GPIO_VD_IS 19
#define GPIO_VD_FZ 24

void send_spi(int fd, unsigned char *data, size_t len) {
  __u8 miso[MAX_LENGTH];
  struct spi_ioc_transfer tr = {
      .tx_buf = (unsigned long)data,
      .rx_buf = (unsigned long)miso,
      .delay_usecs = 1,
      .len = len,
      .speed_hz = PORT_SPEED,
  };
  int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
  if (ret == -1) {
    fprintf(stderr, "main: ioctl SPI_IOC_MESSAGE: %d: %s\n", fd,
            strerror(errno));
  }
}

static bool set_gpio(const int gpio_num, const bool bit) {
  char fname[PATH_MAX];

  char buf = bit ? '1' : '0';
  snprintf(fname, sizeof(fname), "/sys/class/gpio/gpio%d/value", gpio_num);
  int fd = open(fname, O_WRONLY);
  if (fd < 0) {
    fprintf(stderr, "set_gpio(%d, %d) error\n", gpio_num, bit);
    return false;
  }
  write(fd, &buf, 1);
  close(fd);
  return true;
}

static void signal_gpio(int gpio_num) {
  set_gpio(gpio_num, true);
  usleep(10000);
  set_gpio(gpio_num, false);
}

void turn_on(int fd) {
  unsigned char cmd[] = {0x23, 0xff, 0xff, 0x28, 0xff, 0xff,
                         0x29, 0,    0x5,  0x24, 0,    0x4};
  send_spi(fd, cmd, sizeof(cmd));
}

void turn_off(int fd) {
  unsigned char cmd[] = {0x23, 0, 0, 0x28, 0, 0, 0x29, 0, 0x4, 0x24, 0, 0x4};
  send_spi(fd, cmd, sizeof(cmd));
}

void send_focus_cmd(int fd, bool forward) {
  unsigned char cmd[] = {0x29, 0x5,  0x4 + forward, 0x2a, 0x35, 0xc,
                         0x24, 0,    0x4,           0x25, 0x38, 0x1,
                         0x23, 0xff, 0xff,          0x28, 0xff, 0xff};
  send_spi(fd, cmd, sizeof(cmd));
}

u_int16_t get_iris(int fd) {
  unsigned char cmd[] = {0x40, 0, 0};
  __u8 miso[MAX_LENGTH];
  struct spi_ioc_transfer tr = {
      .tx_buf = (unsigned long)cmd,
      .rx_buf = (unsigned long)miso,
      .delay_usecs = 1,
      .len = sizeof(cmd),
      .speed_hz = PORT_SPEED,
  };
  int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
  if (ret == -1) {
    fprintf(stderr, "main: ioctl SPI_IOC_MESSAGE: %d: %s\n", fd,
            strerror(errno));
  }
  send_spi(fd, cmd, sizeof(cmd));
  return miso[1] | miso[2] << 8;
}

static int iris_value;
void set_iris(int fd, unsigned value) {
  unsigned char cmd[] = {0x0, value & 0xff, value >> 8};
  send_spi(fd, cmd, sizeof(cmd));
  signal_gpio(GPIO_VD_IS);
  iris_value = value;

  printf("set iris %#x, get %#x\n", value, get_iris(fd));
}

#define STEP 0x80

#define IRIS_MIN 0
#define IRIS_MAX 0x03E8
#define IRIS_DEFAULT 0xC8

void send_iris_cmd(int fd, bool forward) {
  int step = forward ? STEP : -STEP;
  int nvalue = iris_value + step;
  if (nvalue < IRIS_MIN)
    nvalue = IRIS_MIN;
  if (nvalue > IRIS_MAX)
    nvalue = IRIS_MAX;

  set_iris(fd, nvalue);
}

static void init_iris(int fd) {
  unsigned char cmd[] = {
      0x1,  0x8A, 0x7C, 0x2,  0xF0, 0x66, 0x3,  0x10, 0x0E, 0x4,  0xFF,
      0x80, 0x5,  0x24, 0x00, 0xb,  0x80, 0x04, 0xe,  0x00, 0x03,
  };
  send_spi(fd, cmd, sizeof(cmd));
  set_iris(fd, IRIS_DEFAULT);
}

static void init_original(int fd) {
  unsigned char cmd1[] = {0x20, 0x1,  0x7f, 0x22, 0x1,  0,    0x27, 0x1,  0,
                          0x23, 0xff, 0xff, 0x28, 0xff, 0xff, 0x21, 0x87, 0,
                          0x1,  0x8a, 0x7c, 0x2,  0xf0, 0x66, 0x3,  0x10, 0xe,
                          0x4,  0xff, 0x80, 0x5,  0x24, 0,    0xb,  0x80, 0x4,
                          0xe,  0,    0x3,  0,    0,    0};
  send_spi(fd, cmd1, sizeof(cmd1));
  unsigned char cmd2[] = {0x24, 0, 0xc, 0x29, 0, 0xc};
  send_spi(fd, cmd2, sizeof(cmd2));
}

static void init_lens(int fd) {
  unsigned char cmd[] = {
      0x20, 0x0A, 0x3C, 0x21, 0x00, 0x00, 0x22, 0x03, 0x00,
      0x23, 0x00, 0x00, 0x27, 0x03, 0x00, 0x28, 0x00, 0x00,
  };
  send_spi(fd, cmd, sizeof(cmd));
}

void set_focus(int fd, bool forward) {
  turn_on(fd);
  // for (int i = 0; i < 20; i++) {
  send_focus_cmd(fd, forward);
  puts("Step");
  signal_gpio(GPIO_VD_FZ);
  //}
  turn_off(fd);
}

void send_zoom_cmd(int fd, bool forward) {
  unsigned char cmd[] = {0x29, 0x0,  0x0,           0x2a, 0x35, 0xc,
                         0x24, 0x50, 0x4 + forward, 0x25, 0x38, 0x1,
                         0x23, 0xff, 0xff,          0x28, 0xff, 0xff};
  send_spi(fd, cmd, sizeof(cmd));
}

void set_zoom(int fd, bool forward) {
  turn_on(fd);
  for (int i = 0; i < 20; i++) {
    send_zoom_cmd(fd, forward);
    puts("Step");
    signal_gpio(24);
  }
  turn_off(fd);
}

#define BLEND_SHIFT 6
#define ALPHA 64 // 1
#define BELTA 54 // 0.85

static const int AFWeight[15][17] = {
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    {1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1},
    {1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1},
    {1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1},
    {1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1},
    {1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1},
    {1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1},
    {1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1},
    {1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1},
    {1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1},
    {1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1},
    {1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1},
    {1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1},
    {1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1},
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
};

static const ISP_FOCUS_STATISTICS_CFG_S gstFocusCfg = {
    {1,
     17,
     15,
     3840,
     2160,
     1,
     0,
     {0, 0, 0, 3840, 2160},
     0,
     {0x2, 0x4, 0},
     {1, 0x9bff},
     0xf0},
    {1,
     {1, 1, 1},
     15,
     {188, 414, -330, 486, -461, 400, -328},
     {7, 0, 3, 1},
     {1, 0, 255, 0, 220, 8, 14},
     {127, 12, 2047}},
    {0,
     {1, 1, 0},
     2,
     {200, 200, -110, 461, -415, 0, 0},
     {6, 0, 1, 0},
     {0, 0, 0, 0, 0, 0, 0},
     {15, 12, 2047}},
    {{20, 16, 0, -16, -20}, {1, 0, 255, 0, 220, 8, 14}, {38, 12, 1800}},
    {{-12, -24, 0, 24, 12}, {1, 0, 255, 0, 220, 8, 14}, {15, 12, 2047}},
    {4, {0, 0}, {1, 1}, 0}};

int exit_AF = 0;
void *AF_proc(void *arg) {
  HI_S32 s32Ret = HI_SUCCESS;
  ISP_AF_STATISTICS_S stIspAfStatics;
  int ViPipe = 0;

  while (exit_AF == 0) {
    s32Ret = HI_MPI_ISP_GetVDTimeOut(ViPipe, ISP_VD_FE_START, 5000);
    if (HI_SUCCESS != s32Ret) {
      printf("HI_MPI_ISP_GetVDTimeOut error!(s32Ret = 0x%x)\n", s32Ret);
      return ((void *)-1);
    }

    s32Ret = HI_MPI_ISP_GetFocusStatistics(ViPipe, &stIspAfStatics);
    if (HI_SUCCESS != s32Ret) {
      printf("HI_MPI_ISP_GetStatistics error!(s32Ret = 0x%x)\n", s32Ret);
      return ((void *)-1);
    }

    HI_U32 i, j;
    HI_U32 u32SumFv1 = 0;
    HI_U32 u32SumFv2 = 0;
    HI_U32 u32WgtSum = 0;
    HI_U32 u32Fv1_n, u32Fv2_n, u32Fv1, u32Fv2;
    for (i = 1; i < gstFocusCfg.stConfig.u16Vwnd; i++) {
      for (j = 1; j < gstFocusCfg.stConfig.u16Hwnd; j++) {
        HI_U32 u32H1 = stIspAfStatics.stBEAFStat.stZoneMetrics[i][j].u16h1;
        HI_U32 u32H2 = stIspAfStatics.stBEAFStat.stZoneMetrics[i][j].u16h2;
        HI_U32 u32V1 = stIspAfStatics.stBEAFStat.stZoneMetrics[i][j].u16v1;
        HI_U32 u32V2 = stIspAfStatics.stBEAFStat.stZoneMetrics[i][j].u16v2;
        u32Fv1_n = (u32H1 * ALPHA + u32V1 * ((1 << BLEND_SHIFT) - ALPHA)) >>
                   BLEND_SHIFT;
        u32Fv2_n = (u32H2 * BELTA + u32V2 * ((1 << BLEND_SHIFT) - BELTA)) >>
                   BLEND_SHIFT;

        u32SumFv1 += AFWeight[i][j] * u32Fv1_n;
        u32SumFv2 += AFWeight[i][j] * u32Fv2_n;
        u32WgtSum += AFWeight[i][j];
      }
    }

    u32Fv1 = u32SumFv1 / u32WgtSum;
    u32Fv2 = u32SumFv2 / u32WgtSum;
    printf("u32Fv1=%4d u32Fv2=%4d\n", u32Fv1, u32Fv2);
  }

  return ((void *)0);
}

pthread_t g_af_thread;
static void toggle_af(void) {
  if (g_af_thread) {
    exit_AF = 1;
    pthread_join(g_af_thread, NULL);
    g_af_thread = 0;
    return;
  }

  exit_AF = 0;
  pthread_attr_t attr1;
  pthread_attr_init(&attr1);
  pthread_attr_setstacksize(&attr1, 0x10000);
  pthread_create(&g_af_thread, &attr1, AF_proc, NULL);
}

static void setup_gpio(int gpio_num, bool output) {
  char fname[PATH_MAX];
  snprintf(fname, sizeof(fname), "/sys/class/gpio/gpio%d", gpio_num);
  if (access(fname, 0)) {
    int fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd < 0) {
      fprintf(stderr, "Cannot export GPIO %d\n", gpio_num);
      return;
    }

    char num[16];
    snprintf(num, sizeof(num), "%d", gpio_num);
    write(fd, num, sizeof(num) - 1);
    close(fd);
  }

  snprintf(fname, sizeof(fname), "/sys/class/gpio/gpio%d/direction", gpio_num);
  int fd = open(fname, O_RDWR);
  if (fd < 0) {
    fprintf(stderr, "Cannot switch GPIO direction %d\n", gpio_num);
    return;
  }

  const char *direction = output ? "out" : "in";
  size_t dirlen = strlen(direction);

  char buf[32] = {0};
  if (read(fd, buf, sizeof(buf)) <= 0) {
    fprintf(stderr, "Cannot read GPIO direction %d\n", gpio_num);
    return;
  }
  if (memcmp(buf, direction, dirlen) == 0) {
    return;
  }

  write(fd, direction, dirlen);
  close(fd);
}

static void export_gpios() {
  setup_gpio(GPIO_RSTB, true);
  set_gpio(GPIO_RSTB, 1);
  usleep(10000);
  set_gpio(GPIO_RSTB, 0);
  usleep(10000);
  set_gpio(GPIO_RSTB, 1);

  setup_gpio(GPIO_VD_FZ, true);
  setup_gpio(GPIO_VD_IS, true);
}

int main(int argc, char *argv[]) {
  int ret;

  struct termios ctrl;
  tcgetattr(STDIN_FILENO, &ctrl);
  ctrl.c_lflag &= ~ICANON; // turning off canonical mode makes input unbuffered
  ctrl.c_lflag &= ~ECHO;   // disable echo
  ctrl.c_lflag &= ~ISIG;   // disable system Ctrl-C
  tcsetattr(STDIN_FILENO, TCSANOW, &ctrl);

  int fd = open("/dev/spidev2.0", O_RDWR);
  if (fd == -1) {
    fprintf(stderr, "main: opening device file: %s: %s\n", DEVICE_NAME,
            strerror(errno));
    return -1;
  }

  int options = SPI_CPHA | SPI_CPOL | SPI_LSB_FIRST | SPI_LSB_FIRST;
  ret = ioctl(fd, SPI_IOC_WR_MODE, &options);
  if (ret < 0) {
    printf("ioctl SPI_IOC_WR_MODE err, value = %d ret = %d\n", options, ret);
    return ret;
  }

  int value = 8;
  ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &value);
  if (ret < 0) {
    printf("ioctl SPI_IOC_WR_BITS_PER_WORD err, value = %d ret = %d\n", value,
           ret);
    return ret;
  }

  value = PORT_SPEED;
  ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &value);
  if (ret < 0) {
    printf("ioctl SPI_IOC_WR_MAX_SPEED_HZ err, value = %d ret = %d\n", value,
           ret);
    return ret;
  }

  export_gpios();
  init_iris(fd);
  init_lens(fd);
  // init_original(fd);

  printf("AN41908A controller\n");
  printf("Commands:\n+ - (Zoom) z x (Focus) a (toggle AF) q w (Iris)\n");

  while (1) {
    char ch;
    int i = read(0, &ch, 1);

    if (!i) {
      printf("stdin closed\n");
      return 0;
    }

    switch (ch) {
    case '-':
      puts("Zoom out");
      set_zoom(fd, true);
      break;

    case '+':
      puts("Zoom in");
      set_zoom(fd, false);
      break;

    case 'z':
      puts("Focus near");
      set_focus(fd, false);
      break;

    case 'x':
      puts("Focus far");
      set_focus(fd, true);
      break;

    case 'a':
      toggle_af();
      break;

    case 'q':
      send_iris_cmd(fd, true);
      break;

    case 'w':
      send_iris_cmd(fd, false);
      break;

    case 3:
      goto outer;

    default:
      printf("Unknown command %c\n", ch);
    }
  }

outer:
  turn_off(fd);
  close(fd);

  return ret;
}
