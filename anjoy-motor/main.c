/* anjoy-motor — drive zoom/focus/iris and the pan/tilt head of Anjoy AF
 * camera modules (verified on MTF45-4G_AF, FW V3.4.5.6) over the motor-MCU
 * UART (/dev/ttyS2 on the stock SigmaStar board).
 *
 * TX: 7-byte Pelco-D-style frames   FF 01 C1 C2 D1 D2 CK
 *     CK = (01 + C1 + C2 + D1 + D2) & 0xff. Motion frames are one-shot;
 *     the MCU keeps moving until a stop frame arrives. The stock vendor
 *     stack follows every motion verb with two stop frames 10 ms apart.
 *
 * RX: the MCU streams unsolicited 17-byte status reports (~4-5 Hz), and
 *     ONLY while a motor is actually moving — silent at rest:
 *         51 01 04 78 01 0A 0A 10 3B ZZ ZZ 21 3B FF FF 3B CK
 *     CK = 8-bit sum of the 16 preceding bytes (verified on live frames).
 *     ZZ ZZ = zoom position, 16-bit big-endian counter (live-verified
 *     counting down 01 08 -> 01 04 while zooming wide). FF FF = focus
 *     position, same encoding (14 0F observed at rest). Other fields not
 *     yet mapped. The tool listens through the whole verb + hold window,
 *     so reports are captured while the motor actually runs.
 *
 * Stock-firmware contention: comm_server and media_server hold the same
 * port open; whichever daemon is blocked in read() usually eats the RX
 * reports (comm_server feeds the vendor RTSP zoom OSD with them), and
 * procmgr respawns both within seconds of a kill. For exclusive control
 * stop both and listen inside the respawn gap (verified working:
 * killall followed immediately by a listen yields frames while the
 * motor moves). The kernel byte counters in /proc/tty/driver/ms_uart
 * rise during motion regardless of who is reading — use them to check
 * whether motion/reporting is happening at all. See Readme.
 */

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

struct verb {
  char key;
  const char *name;
  unsigned char c1, c2, d1, d2;
};

static const struct verb VERBS[] = {
    {'u', "zoom-tele", 0x00, 0x20, 0x14, 0x14},
    {'d', "zoom-wide", 0x00, 0x40, 0x14, 0x14},
    {'l', "focus-near", 0x01, 0x00, 0x30, 0x30},
    {'r', "focus-far", 0x00, 0x80, 0x30, 0x30},
    {'I', "iris-open", 0x02, 0x00, 0x30, 0x30},
    {'O', "iris-close", 0x04, 0x00, 0x30, 0x30},
    {'U', "head-up", 0x00, 0x08, 0x00, 0x14},
    {'D', "head-down", 0x00, 0x10, 0x00, 0x14},
    {'L', "head-left", 0x00, 0x04, 0x14, 0x00},
    {'R', "head-right", 0x00, 0x02, 0x14, 0x00},
    {'s', "stop", 0x00, 0x00, 0x00, 0x00},
};
#define NVERBS (sizeof(VERBS) / sizeof(VERBS[0]))

#define REPORT_LEN 17
#define REPORT_SYNC 0x51

static void usage(const char *prog) {
  printf("Usage: %s -d <key> [-t secs] [-s speed] [-r ms] [-j] [-D dev] [-B]\n", prog);
  printf("  -d <key>  verb key:\n");
  for (unsigned i = 0; i < NVERBS; i++)
    printf("     %c  %s\n", VERBS[i].key, VERBS[i].name);
  printf("  -t secs   hold the verb then send stop twice (vendor cadence); 0 = verb only\n");
  printf("  -s speed  override both speed bytes (0x00-0xff); default per verb\n");
  printf("  -r ms     total listen window, starting right before the verb is sent\n");
  printf("            (the MCU reports only while its motors move, so reports arrive\n");
  printf("            during the hold, not after the stop)\n");
  printf("  -j        decode reports as one JSON object per line on stdout (consumed\n");
  printf("            by web UIs; TX diagnostics go to stderr)\n");
  printf("  -D dev    UART device (default /dev/ttyS2)\n");
  printf("  -B        inherit port settings (do not touch termios; co-exist with the stock stack)\n");
}

static int build_frame(unsigned char *f, const struct verb *v, int speed) {
  f[0] = 0xFF;
  f[1] = 0x01;
  f[2] = v->c1;
  f[3] = v->c2;
  f[4] = speed >= 0 ? (unsigned char)speed : v->d1;
  f[5] = speed >= 0 ? (unsigned char)speed : v->d2;
  f[6] = (unsigned char)(f[1] + f[2] + f[3] + f[4] + f[5]);
  return 7;
}

static void print_frame(const char *tag, const unsigned char *f, int json) {
  FILE *out = json ? stderr : stdout;
  fprintf(out, "%s:", tag);
  for (int i = 0; i < 7; i++)
    fprintf(out, " %02X", f[i]);
  fprintf(out, "\n");
}

/* accumulate RX bytes and emit a decoded report per 17-byte frame */
static void report_feed(unsigned char *buf, int *len, unsigned char b, int json) {
  if (*len == 0) {
    if (b != REPORT_SYNC)
      return;
    buf[(*len)++] = b;
    return;
  }
  buf[(*len)++] = b;
  if (*len < REPORT_LEN)
    return;

  unsigned char ck = 0;
  for (int i = 0; i < REPORT_LEN - 1; i++)
    ck += buf[i];
  if (ck == buf[REPORT_LEN - 1]) {
    /* zoom: bytes 9-10, focus: bytes 13-14 — both 16-bit big-endian */
    int zoom = buf[9] * 256 + buf[10];
    int focus = buf[13] * 256 + buf[14];
    if (json) {
      printf("{\"zoom_pos\":%d,\"focus_pos\":%d,\"flags\":\"%02X%02X\",\"raw\":\"",
             zoom, focus, buf[2], buf[5]);
      for (int i = 0; i < REPORT_LEN; i++)
        printf("%02X", buf[i]);
      printf("\"}\n");
    } else {
      printf("REPORT zoom=%d focus=%d raw:", zoom, focus);
      for (int i = 0; i < REPORT_LEN; i++)
        printf(" %02X", buf[i]);
      printf("\n");
    }
    *len = 0;
    return;
  }
  /* bad checksum: resync — drop the oldest bytes up to the next sync */
  int next = -1;
  for (int i = 1; i < REPORT_LEN; i++) {
    if (buf[i] == REPORT_SYNC) {
      next = i;
      break;
    }
  }
  if (next < 0) {
    *len = 0;
    return;
  }
  memmove(buf, buf + next, REPORT_LEN - next);
  *len = REPORT_LEN - next;
}

static long now_ms(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

int main(int argc, char **argv) {
  const char *dev = "/dev/ttyS2";
  const struct verb *verb = NULL;
  double hold = 1.0;
  int speed = -1;
  int rx_ms = 300;
  int inherit = 0;
  int json = 0;

  int opt;
  while ((opt = getopt(argc, argv, "d:t:s:r:D:Bjh")) != -1) {
    switch (opt) {
    case 'd': {
      for (unsigned i = 0; i < NVERBS; i++)
        if (VERBS[i].key == (unsigned char)optarg[0] && optarg[1] == '\0')
          verb = &VERBS[i];
      if (!verb) {
        fprintf(stderr, "unknown verb key '%s'\n", optarg);
        return 2;
      }
      break;
    }
    case 't':
      hold = atof(optarg);
      break;
    case 's':
      speed = strtol(optarg, NULL, 0);
      if (speed < 0 || speed > 255) {
        fprintf(stderr, "speed must fit a byte\n");
        return 2;
      }
      break;
    case 'r':
      rx_ms = atoi(optarg);
      break;
    case 'D':
      dev = optarg;
      break;
    case 'B':
      inherit = 1;
      break;
    case 'j':
      json = 1;
      break;
    default:
      usage(argv[0]);
      return opt == 'h' ? 0 : 2;
    }
  }
  if (!verb) {
    usage(argv[0]);
    return 2;
  }

  int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    fprintf(stderr, "open %s: %s\n", dev, strerror(errno));
    return 1;
  }

  if (!inherit) {
    struct termios t;
    if (tcgetattr(fd, &t) == 0) {
      cfmakeraw(&t);
      cfsetispeed(&t, B115200);
      cfsetospeed(&t, B115200);
      tcsetattr(fd, TCSANOW, &t);
    }
  }
  tcflush(fd, TCOFLUSH);

  unsigned char f[7];
  build_frame(f, verb, speed);
  print_frame("TX", f, json);
  ssize_t w = write(fd, f, 7);
  if (w != 7) {
    fprintf(stderr, "write: %zd (%s)\n", w, strerror(errno));
    close(fd);
    return 1;
  }

  /* Auto-stop fires after `hold` seconds (vendor cadence: stop twice,
   * 10 ms apart). The MCU only reports while its motors move, so the RX
   * window starts with the verb and runs through the hold — never let it
   * end before a pending auto-stop or the motor would be left running. */
  int auto_stop = (hold > 0 && verb->key != 's');
  long listen_ms = rx_ms;
  if (auto_stop)
    listen_ms = (long)(hold * 1000) + rx_ms;
  long t0 = now_ms();

  struct pollfd p = {fd, POLLIN, 0};
  unsigned char buf[256];
  unsigned char rbuf[REPORT_LEN];
  int rlen = 0;
  for (;;) {
    long elapsed = now_ms() - t0;
    if (auto_stop && elapsed >= (long)(hold * 1000)) {
      unsigned char st[7];
      const struct verb *stop = &VERBS[NVERBS - 1];
      build_frame(st, stop, -1);
      print_frame("TX", st, json);
      write(fd, st, 7);
      usleep(10000);
      write(fd, st, 7);
      auto_stop = 0;
    }
    long remain = listen_ms - (now_ms() - t0);
    if (remain <= 0)
      break;
    int r = poll(&p, 1, remain > 20 ? 20 : (int)remain);
    if (r < 0)
      break;
    if (r > 0) {
      ssize_t got = read(fd, buf, sizeof(buf));
      if (got < 0) {
        if (errno == EAGAIN)
          continue;
        break;
      }
      for (ssize_t i = 0; i < got; i++)
        report_feed(rbuf, &rlen, buf[i], json);
    }
  }

  close(fd);
  return 0;
}
