/*
 * xm-uart: drive the zoom/focus board of Xiongmai AF camera modules over its
 * UART, the way the stock camera firmware does. The protocol is specified in
 * PROTOCOL.md next to this file; every value below is taken from captures of
 * the stock firmware and of the board's behaviour, not from generic Pelco-D.
 */
#include <arpa/inet.h>
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
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>

#define SYNC 0xc5    /* the only start byte the board accepts */
#define TRAILER 0x5c /* not checked by the board, sent by the stock firmware */
#define ADDRESS 1    /* ignored by the board, 1 in the stock configuration */

/* cmd1 / cmd2 bits (PROTOCOL.md, "Command bits") */
#define CMD1_FOCUS_FAR 0x01 /* farther (measured); Pelco-D calls it "near" */
#define CMD2_RIGHT 0x02
#define CMD2_LEFT 0x04
#define CMD2_UP 0x08
#define CMD2_DOWN 0x10
#define CMD2_ZOOM_TELE 0x20
#define CMD2_ZOOM_WIDE 0x40
#define CMD2_FOCUS_NEAR 0x80 /* nearer (measured); Pelco-D calls it "far" */

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
 * inter-byte timeout: half a frame left on its wire swallows the next
 * command. So nothing is written directly: frames go through a queue that
 * only ever holds whole frames, plus the unsent rest of the one frame the
 * UART has started, and is drained without blocking. */
static long long now_ms(void);

#define FRAME 8
static uint8_t txq[4096];
static size_t txlen; /* txlen % FRAME = unsent rest of a frame already started */

static void uart_flush(void) {
  while (txlen) {
    ssize_t n = write(uart, txq, txlen);
    if (n <= 0) {
      if (n < 0 && errno == EINTR)
        continue;
      return; /* EAGAIN: POLLOUT in the main loop brings us back */
    }
    memmove(txq, txq + n, txlen - (size_t)n);
    txlen -= (size_t)n;
  }
}

/* Queue one whole frame. A normal frame is dropped (false) when there is no
 * room; a stop frame instead discards the queued frames that have not
 * started, keeping only the rest of the frame in progress, so it always
 * goes out right after it. */
static bool uart_send(const uint8_t *f, bool is_stop) {
  if (is_stop)
    txlen %= FRAME;
  else if (txlen + FRAME > sizeof(txq))
    return false;
  memcpy(txq + txlen, f, FRAME);
  txlen += FRAME;
  uart_flush();
  return true;
}

/* At exit: give the queue up to 2 s to drain, then let the kernel finish. */
static void uart_drain(void) {
  long long deadline = now_ms() + 2000;
  while (txlen && now_ms() < deadline) {
    struct pollfd p = {.fd = uart, .events = POLLOUT};
    poll(&p, 1, 50);
    uart_flush();
  }
  tcdrain(uart);
}

static void send_frame(uint8_t cmd1, uint8_t cmd2, uint8_t data1,
                       uint8_t data2) {
  uint8_t f[8] = {SYNC, ADDRESS, cmd1, cmd2, data1, data2, 0, TRAILER};
  f[6] = (uint8_t)(f[1] + f[2] + f[3] + f[4] + f[5]); /* sum % 256 */
  bool is_stop = !cmd1 && !cmd2 && !data1 && !data2;
  if (!uart_send(f, is_stop))
    fprintf(stderr, "UART queue full, frame dropped\n");
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

/* Network relay (-l): frames from a TCP client go to the lens board, the
 * board's replies go back to the client. Used to replay a vendor camera's
 * traffic onto another camera's lens (see PROTOCOL.md). */
#define PARTIAL_TIMEOUT_MS 300

static int client_fd = -1;
static struct in_addr allowed_client; /* -a: the only address let in */
static bool allow_any = true;
static uint8_t net_frame[8];
static size_t net_len;
static long long net_started_ms;
static unsigned long net_frames, net_dropped;

static long long now_ms(void) {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (long long)tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

/* Reassemble with the board's own rule: an A5 or C5 byte starts an 8-byte
 * frame. Only whole frames are written, since the board's parser has no
 * inter-byte timeout and a partial one would swallow the next command. */
static void net_expire(void);

static void net_write_frame(void) {
  static const uint8_t stop[FRAME] = {SYNC, ADDRESS, 0, 0, 0, 0, ADDRESS, TRAILER};
  bool is_stop = !memcmp(net_frame, stop, 2) && !memcmp(net_frame + 2, stop + 2, 4);
  if (uart_send(net_frame, is_stop))
    net_frames++;
  else
    net_dropped += FRAME; /* client outpaces the UART: lose whole frames only */
}

/* Resynchronise inside a rejected candidate: drop its first byte and carry
 * on from the next sync byte in it, if any. */
static void net_resync(void) {
  size_t i = 1;
  while (i < net_len && net_frame[i] != 0xa5 && net_frame[i] != SYNC)
    i++;
  net_dropped += i;
  memmove(net_frame, net_frame + i, net_len - i);
  net_len -= i;
  if (net_len)
    net_started_ms = now_ms();
}

#define TRUNCATED_GAP_MS 20

static void net_feed(const uint8_t *data, size_t len) {
  net_expire(); /* a late tail must not complete a frame that already timed out */
  /* Senders write whole frames in one go. A partial frame still waiting
   * when a new batch starts with a sync byte was truncated: drop it rather
   * than splice the new command into it. */
  if (net_len && len && (data[0] == 0xa5 || data[0] == SYNC) &&
      now_ms() - net_started_ms > TRUNCATED_GAP_MS) {
    printf("Discarded a truncated %zu-byte frame from the network\n", net_len);
    net_dropped += net_len;
    net_len = 0;
  }
  for (size_t i = 0; i < len; i++) {
    uint8_t b = data[i];
    if (net_len == 0) {
      if (b != 0xa5 && b != SYNC) {
        net_dropped++;
        continue;
      }
      net_started_ms = now_ms();
    }
    net_frame[net_len++] = b;
    while (net_len == FRAME) {
      /* A C5 command carries its own end byte: one without it is a splice
       * of two frames, never a command. (A5 frames have no such check.) */
      if (net_frame[0] == SYNC && net_frame[FRAME - 1] != TRAILER) {
        net_resync();
        continue;
      }
      net_write_frame();
      net_len = 0;
    }
  }
}

static void net_expire(void) {
  if (net_len && now_ms() - net_started_ms > PARTIAL_TIMEOUT_MS) {
    printf("Discarded a partial frame of %zu bytes from the network\n", net_len);
    net_dropped += net_len;
    net_len = 0;
  }
}

static void net_close_client(const char *why) {
  close(client_fd);
  client_fd = -1;
  net_len = 0;
  printf("Client %s after %lu frames (%lu bytes dropped); stop\n", why, net_frames,
         net_dropped);
  send_stop(); /* never leave a motor running for a sender that vanished */
}

static void net_close_client(const char *why);

/* Board replies to the client, whole: the socket is nonblocking, so a peer
 * that stops reading is dropped (and the lens stopped) instead of freezing
 * the loop or silently losing part of a reply. */
static void net_send_reply(const uint8_t *data, size_t len) {
  long long deadline = now_ms() + 200;
  while (len && client_fd >= 0) {
    ssize_t n = send(client_fd, data, len, MSG_NOSIGNAL);
    if (n > 0) {
      data += n;
      len -= (size_t)n;
    } else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) &&
               now_ms() < deadline) {
      struct pollfd p = {.fd = client_fd, .events = POLLOUT};
      poll(&p, 1, 20);
    } else {
      net_close_client("not reading replies");
    }
  }
}

static int net_listen(int port) {
  int fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0)
    return -1;
  int one = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  struct sockaddr_in a = {.sin_family = AF_INET,
                          .sin_port = htons((uint16_t)port),
                          .sin_addr.s_addr = htonl(INADDR_ANY)};
  if (bind(fd, (struct sockaddr *)&a, sizeof(a)) < 0 || listen(fd, 1) < 0) {
    close(fd);
    return -1;
  }
  return fd;
}

static void net_accept(int listen_fd) {
  struct sockaddr_in a;
  socklen_t alen = sizeof(a);
  int fd = accept(listen_fd, (struct sockaddr *)&a, &alen);
  if (fd < 0)
    return;
  if (!allow_any && a.sin_addr.s_addr != allowed_client.s_addr) {
    printf("Refused %s: not the allowed client\n", inet_ntoa(a.sin_addr));
    close(fd);
    return;
  }
  if (client_fd >= 0) {
    printf("Refused %s: a client is already connected\n", inet_ntoa(a.sin_addr));
    close(fd);
    return;
  }
  int one = 1;
  setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
  /* A client that vanishes without FIN/RST is found in ~11 s and the lens
   * stopped, instead of the last move running on forever. */
  setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));
#ifdef TCP_KEEPIDLE
  int idle = 5, intvl = 2, cnt = 3;
  setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
  setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
  setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt));
#endif
  fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
  client_fd = fd;
  net_frames = net_dropped = 0;
  printf("Client %s connected\n", inet_ntoa(a.sin_addr));
}

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
  printf("Usage: %s [-d tty] [-l port [-a client-ip]]\n", argv0);
  printf("\t-d tty device, default /dev/ttyAMA0\n");
  printf("\t-l listen on this TCP port: frames from the client go to the lens\n"
         "\t   board whole, its replies go back (stdin EOF does not quit)\n");
  printf("\t-a with -l: accept only this client address (anyone on the network\n"
         "\t   could otherwise drive the lens)\n\n");
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
  int port = 0, c;

  while ((c = getopt(argc, argv, "d:l:a:h")) != -1) {
    switch (c) {
    case 'd':
      device = optarg;
      break;
    case 'a':
      if (!inet_aton(optarg, &allowed_client)) {
        fprintf(stderr, "bad client address: %s\n", optarg);
        return 1;
      }
      allow_any = false;
      break;
    case 'l':
      port = atoi(optarg);
      if (port <= 0 || port > 65535) {
        fprintf(stderr, "bad port: %s\n", optarg);
        return 1;
      }
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

  int listen_fd = -1;
  if (port) {
    listen_fd = net_listen(port);
    if (listen_fd < 0) {
      fprintf(stderr, "listen on port %d: %s\n", port, strerror(errno));
      return 1;
    }
    if (allow_any)
      fprintf(stderr, "warning: no -a, any host that reaches port %d can move the lens\n", port);
  }
  bool stdin_open = true;

  struct sigaction sa = {.sa_handler = on_signal};
  sigaction(SIGINT, &sa, NULL);
  sigaction(SIGTERM, &sa, NULL);
  sigaction(SIGHUP, &sa, NULL);
  signal(SIGPIPE, SIG_IGN); /* a vanished client must not kill us mid-move */

  printf("Xiongmai UART Motors, get in a car and fasten your safety belt\n");
  usage(argv[0]);

  while (!quit) {
    struct pollfd pfds[4] = {
        {.fd = stdin_open ? STDIN_FILENO : -1, .events = POLLIN},
        {.fd = uart, .events = (short)(POLLIN | (txlen ? POLLOUT : 0))},
        {.fd = listen_fd, .events = POLLIN},
        {.fd = client_fd, .events = POLLIN},
    };

    /* A signal landing between the quit check and poll() would otherwise
     * wait for the next key; the timeout bounds that (no ppoll in uClibc)
     * and also ages out a partial network frame. */
    if (poll(pfds, 4, 100) < 0) {
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
        if (listen_fd >= 0) { /* a relay runs detached from any terminal */
          stdin_open = false;
          continue;
        }
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

    if (pfds[1].revents & POLLOUT)
      uart_flush();
    if (pfds[1].revents & POLLIN) {
      uint8_t rbuf[256];
      ssize_t n = read(uart, rbuf, sizeof(rbuf));
      if (n == 0) {
        printf("UART closed, make sure system getty is disabled on UART\n");
        break;
      }
      if (n > 0) {
        if (client_fd >= 0)
          net_send_reply(rbuf, (size_t)n);
        parse_incoming(rbuf, (size_t)n);
      } else if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
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

    if (pfds[2].revents & POLLIN)
      net_accept(listen_fd);
    if (client_fd >= 0 && pfds[3].revents & (POLLIN | POLLHUP | POLLERR)) {
      uint8_t nbuf[512];
      ssize_t n = recv(client_fd, nbuf, sizeof(nbuf), 0);
      if (n > 0)
        net_feed(nbuf, (size_t)n);
      else if (n == 0 || (errno != EAGAIN && errno != EINTR))
        net_close_client(n == 0 ? "disconnected" : "failed");
    }
    net_expire();
    fflush(stdout);
  }

  /* Never leave a motor running: whatever ended the loop, stop first. */
  puts("Stop");
  send_stop();
  if (client_fd >= 0)
    close(client_fd);
  if (listen_fd >= 0)
    close(listen_fd);
  uart_drain();
  close(uart);
  return 0;
}
