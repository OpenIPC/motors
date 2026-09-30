/*
 * xm-uart: drive the zoom/focus board of Xiongmai AF camera modules over its
 * UART, the way the stock camera firmware does. The protocol is specified in
 * PROTOCOL.md next to this file; every value below is taken from captures of
 * the stock firmware and of the board's behaviour, not from generic Pelco-D.
 */
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <poll.h>
#include <signal.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#define SYNC 0xc5    /* the only start byte the board accepts */
#define TRAILER 0x5c /* not checked by the board, sent by the stock firmware */
#define ADDRESS 1    /* ignored by the board, 1 in the stock configuration */

/* cmd1 / cmd2 bits (PROTOCOL.md, "Command bits") */
#define CMD1_FOCUS_FAR 0x01 /* stock "FocusFar"; Pelco-D calls this bit "near" */
#define CMD2_RIGHT 0x02
#define CMD2_LEFT 0x04
#define CMD2_UP 0x08
#define CMD2_DOWN 0x10
#define CMD2_ZOOM_TELE 0x20
#define CMD2_ZOOM_WIDE 0x40
#define CMD2_FOCUS_NEAR 0x80 /* stock "FocusNear"; Pelco-D calls this bit "far" */

#define MAX_SPEED 0x3f

static int uart = -1;
static struct termios saved_stdin;
static bool stdin_saved;
static volatile sig_atomic_t quit;

static void on_signal(int sig) {
  (void)sig;
  quit = 1;
}

/* The board parses a fixed 8-byte frame from any C5 or A5 byte and has no
 * inter-byte timeout: a short write would make it swallow the next command.
 * So every frame goes out whole. */
static int write_all(int fd, const uint8_t *buf, size_t len) {
  while (len) {
    ssize_t n = write(fd, buf, len);
    if (n < 0) {
      if (errno == EINTR)
        continue;
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        struct pollfd p = {.fd = fd, .events = POLLOUT};
        poll(&p, 1, 100);
        continue;
      }
      return -1;
    }
    buf += n;
    len -= (size_t)n;
  }
  return 0;
}

static void send_frame(uint8_t cmd1, uint8_t cmd2, uint8_t data1,
                       uint8_t data2) {
  uint8_t f[8] = {SYNC, ADDRESS, cmd1, cmd2, data1, data2, 0, TRAILER};
  f[6] = (uint8_t)(f[1] + f[2] + f[3] + f[4] + f[5]); /* sum % 256 */
  if (write_all(uart, f, sizeof(f)) < 0)
    fprintf(stderr, "write: %s\n", strerror(errno));
}

/* pan/tilt/zoom in -100..100 (sign = direction); speed scaled to 0..0x3f */
static void send_move(int pan, int tilt, int zoom) {
  uint8_t cmd2 = 0;
  if (pan < 0)
    cmd2 |= CMD2_LEFT;
  else if (pan > 0)
    cmd2 |= CMD2_RIGHT;
  if (tilt < 0)
    cmd2 |= CMD2_DOWN;
  else if (tilt > 0)
    cmd2 |= CMD2_UP;
  if (zoom < 0)
    cmd2 |= CMD2_ZOOM_WIDE;
  else if (zoom > 0)
    cmd2 |= CMD2_ZOOM_TELE;
  send_frame(0, cmd2, (uint8_t)(abs(pan) * MAX_SPEED / 100),
             (uint8_t)(abs(tilt) * MAX_SPEED / 100));
}

static void send_stop(void) { send_frame(0, 0, 0, 0); }

static void restore_stdin(void) {
  if (stdin_saved)
    tcsetattr(STDIN_FILENO, TCSANOW, &saved_stdin);
}

static void dump_hex(const char *what, const uint8_t *data, size_t size) {
  printf("%s:", what);
  for (size_t i = 0; i < size; i++)
    printf(" %02X", data[i]);
  printf("\n");
}

/* Board replies: EF 01 <type> <len> <payload[len]> (PROTOCOL.md, "Replies").
 * They arrive split across reads, so bytes are collected here until a whole
 * reply is present. */
static uint8_t rx[512];
static size_t rx_len;

static void handle_reply(const uint8_t *r) {
  uint8_t type = r[2], len = r[3];
  const uint8_t *p = r + 4;

  if (type == 0x00 && len >= 4 && !memcmp(p, "\x04\x03\x2f\x2e", 4)) {
    /* zoom report "X1.3 ", or six spaces ~6.7 s after the last one */
    char text[32] = {0};
    size_t n = (size_t)len - 4;
    if (n > sizeof(text) - 1)
      n = sizeof(text) - 1;
    memcpy(text, p + 4, n);
    for (size_t i = n; i > 0 && text[i - 1] == ' '; i--)
      text[i - 1] = '\0';
    if (text[0])
      printf("Zoom %s\n", text);
    else
      printf("Zoom report blank\n");
    return;
  }
  if (type == 0x02 && len == 1 && p[0] <= 1) {
    printf("%s mode\n", p[0] ? "NIGHT" : "DAY");
    return;
  }
  dump_hex("Reply", r, 4 + (size_t)len);
}

static void parse_incoming(const uint8_t *data, size_t size) {
  if (size > sizeof(rx) - rx_len)
    size = sizeof(rx) - rx_len;
  memcpy(rx + rx_len, data, size);
  rx_len += size;

  size_t i = 0;
  while (i < rx_len) {
    if (rx[i] != 0xef) {
      size_t j = i;
      while (j < rx_len && rx[j] != 0xef)
        j++;
      dump_hex("Unknown", rx + i, j - i);
      i = j;
      continue;
    }
    if (rx_len - i < 4)
      break; /* header incomplete */
    if (rx[i + 1] != 0x01) {
      dump_hex("Unknown", rx + i, 1);
      i++;
      continue;
    }
    size_t total = 4 + (size_t)rx[i + 3];
    if (rx_len - i < total)
      break; /* payload incomplete */
    handle_reply(rx + i);
    i += total;
  }
  memmove(rx, rx + i, rx_len - i);
  rx_len -= i;
}

static void usage(const char *argv0) {
  printf("Usage: %s [-d tty]\n", argv0);
  printf("\t-d tty device, default /dev/ttyAMA0\n\n");
  printf("Keys: + - zoom in/out, z x focus near/far, h l pan left/right,\n"
         "      j k tilt down/up, Space or Enter stop, q quit\n");
}

static int open_uart(const char *device) {
  int fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0)
    return -1;

  struct termios options;
  if (tcgetattr(fd, &options) < 0) {
    close(fd);
    return -1;
  }
  cfmakeraw(&options); /* 8 data bits, no parity, no flow control */
  options.c_cflag &= ~CSTOPB;
  options.c_cflag |= CLOCAL | CREAD;
  cfsetspeed(&options, B115200);
  if (tcsetattr(fd, TCSANOW, &options) < 0) {
    close(fd);
    return -1;
  }
  return fd;
}

int main(int argc, char *argv[]) {
  const char *device = "/dev/ttyAMA0";
  int c;

  while ((c = getopt(argc, argv, "d:h")) != -1) {
    switch (c) {
    case 'd':
      device = optarg;
      break;
    case 'h':
      usage(argv[0]);
      return 0;
    default:
      usage(argv[0]);
      return 1;
    }
  }

  uart = open_uart(device);
  if (uart < 0) {
    fprintf(stderr, "%s: %s\n", device, strerror(errno));
    return 1;
  }

  if (tcgetattr(STDIN_FILENO, &saved_stdin) == 0) {
    struct termios raw = saved_stdin;
    raw.c_lflag &= ~(ICANON | ECHO); /* unbuffered keys, no echo */
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    stdin_saved = true;
    atexit(restore_stdin);
  }

  struct sigaction sa = {.sa_handler = on_signal};
  sigaction(SIGINT, &sa, NULL);
  sigaction(SIGTERM, &sa, NULL);
  sigaction(SIGHUP, &sa, NULL);

  printf("Xiongmai UART Motors, get in a car and fasten your safety belt\n");
  usage(argv[0]);

  while (!quit) {
    struct pollfd pfds[2] = {
        {.fd = STDIN_FILENO, .events = POLLIN},
        {.fd = uart, .events = POLLIN},
    };

    /* A signal landing between the quit check and poll() would otherwise
     * wait for the next key; the timeout bounds that (no ppoll in uClibc). */
    if (poll(pfds, 2, 200) < 0) {
      if (errno == EINTR)
        continue;
      perror("poll");
      break;
    }

    if (pfds[0].revents & (POLLIN | POLLHUP)) {
      char ch;
      ssize_t n = read(STDIN_FILENO, &ch, 1);
      if (n <= 0) {
        if (n < 0 && errno == EINTR)
          continue;
        printf("stdin closed\n");
        break;
      }
      switch (ch) {
      case '+':
        puts("Zoom in");
        send_move(0, 0, 1);
        break;
      case '-':
        puts("Zoom out");
        send_move(0, 0, -1);
        break;
      case 'z':
        puts("Focus near");
        send_frame(0, CMD2_FOCUS_NEAR, 0, 0);
        break;
      case 'x':
        puts("Focus far");
        send_frame(CMD1_FOCUS_FAR, 0, 0, 0);
        break;
      case 'h':
        puts("Pan left");
        send_move(-100, 0, 0);
        break;
      case 'i': /* Colemak */
      case 'l':
        puts("Pan right");
        send_move(100, 0, 0);
        break;
      case 'n':
      case 'j':
        puts("Tilt down");
        send_move(0, -100, 0);
        break;
      case 'e':
      case 'k':
        puts("Tilt up");
        send_move(0, 100, 0);
        break;
      case '\n':
      case ' ':
        puts("Stop");
        send_stop();
        break;
      case 'q':
        quit = 1;
        break;
      default:
        printf("Unknown command %c\n", ch);
      }
    }

    if (pfds[1].revents & POLLIN) {
      uint8_t rbuf[256];
      ssize_t n = read(uart, rbuf, sizeof(rbuf));
      if (n == 0) {
        printf("UART closed, make sure system getty is disabled on UART\n");
        break;
      }
      if (n > 0)
        parse_incoming(rbuf, (size_t)n);
      else if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        printf("UART read: %s\n", strerror(errno));
        break;
      }
    }
    /* A hangup or error would otherwise make poll() return at once forever
     * (readable bytes, if any, were handled above); leave through the stop
     * below instead. */
    if (pfds[1].revents & (POLLHUP | POLLERR | POLLNVAL)) {
      printf("UART hung up or failed\n");
      break;
    }
    if (pfds[0].revents & (POLLERR | POLLNVAL))
      break;
    fflush(stdout);
  }

  /* Never leave a motor running: whatever ended the loop, stop first. */
  puts("Stop");
  send_stop();
  tcdrain(uart);
  close(uart);
  return 0;
}
