/* anjoy-motor — drive zoom/focus/iris and the pan/tilt head of Anjoy AF
 * camera modules (verified on MTF45-4G_AF, FW V3.4.5.6) over the motor-MCU
 * UART (/dev/ttyS2 on the stock SigmaStar board, **57600 8N1 raw** — the
 * MCU is deaf at 115200; the vendor rate shows up in comm_server as the
 * c_cflag constant B57600|CS8|CREAD|CLOCAL).
 *
 * PROTOCOL (reverse-engineered from the vendor comm_server with an
 * LD_PRELOAD write() logger; see Readme.md for the full capture story):
 *
 * TX 1 — session/state frames, 16 bytes at a ~280 ms cadence, ALWAYS
 *     flowing while the port is owned (idle and during motion alike):
 *         51 01 04 78 01 0d <FZhi FZlo> <Fhi Flo> 01 00 20 00 00 CK
 *     CK = 8-bit sum of bytes 0..14 (INCLUDING the 0x51 sync). b6..b9 look
 *     like live lens telemetry (they ramp while a motor moves), b10 drifts
 *     slowly; a frozen frame is accepted fine. Without this stream the MCU
 *     ignores command frames completely — bare writes move nothing (tx
 *     counter climbs, rx stays flat, lens inert). This is the reason every
 *     earlier raw-injection attempt failed even with correct frames.
 *     The MCU also needs the stream for a few seconds before it starts
 *     answering at all: priming with ~14 frames (~4 s, vendor cold-start
 *     pacing) gets ACKs and reports flowing; two frames buy silence.
 *
 * TX 2 — optional boot handshake, echoed from the vendor: ASCII "END"+CK,
 *     "VER"+CK, "EXVER"+CK where CK = 8-bit sum of the payload. The MCU
 *     never replies to these, they appear to be a one-way reset/version
 *     probe. Paced 300-500 ms apart like the vendor original.
 *
 * TX 3 — command frames, 7 bytes:
 *         FF 01 C1 C2 D1 D2 CK,  CK = (01 + C1 + C2 + D1 + D2) & 0xff
 *     C2 direction bits (OR for diagonals): 02 right, 04 left, 08 up,
 *     10 down, 20 zoom-tele, 40 zoom-wide, 80 focus-far.
 *     C1 lens bits: 01 focus-near, 02 iris-open, 04 iris-close.
 *     D1 = pan/zoom speed, D2 = tilt speed; vendor maps UI speed 1..10 to
 *     5*v+5 (1->0x0a, 3->0x14, 10->0x37). Focus/iris commands always went
 *     out with 30 30. Motion runs until a stop frame (all-zero fields).
 *
 * RX — single-byte ACKs sprinkled through the stream AND 17-byte status
 *     reports that arrive continuously while a motor is actually moving
 *     (pushing against an endstop counts):
 *         51 01 04 78 01 0A 0A 10 3B ZZ ZZ 21 3B FF FF 3B CK
 *     CK = 8-bit sum of the 16 preceding bytes. ZZ ZZ = zoom position as
 *     DECIMAL DIGITS: byte 9 is the tens digit (zero encoded as 0x3B),
 *     byte 10 the units digit: wide endstop reads 3B 01 (=1), tele endstop
 *     03 00 (=30) — position scale 1..30, wide -> tele. Report cadence is
 *     ~150-300 ms; each position shows up ~2x while passing. Frames with
 *     garbage (all 0x3B) appear during link shutdown — the 1..30 range
 *     guard drops them.
 *
 * Absolute zoom (verb T) is a closed loop on those reports: the target
 * optical multiple is mapped to a position on the 4.4x..45x geometric
 * curve, then tele/wide frames drive toward it. Verified both directions
 * (21->8, 9->20, exact landings): fast approach at 0x14 speed, stop on
 * the first crossing report, 700 ms settle, then ONE slow (0x0A)
 * closed-loop correction approach for any residual delta. Earlier designs
 * blind-pulsed corrections and oscillated — pulses shorter than the
 * report cadence are invisible to the loop (7->12 with no feedback), and
 * the RX race (below) made reports rare enough to hide even that.
 *
 * Stock-firmware contention — TWO vendors daemons sit on ttyS2:
 *   - comm_server runs the proper session and is the writer; freeze it
 *     with SIGSTOP (and SIGSTOP its respawner procman first, or it comes
 *     back),
 *   - media_server ALSO holds the port open and READS it — once
 *     comm_server is dead or frozen it eats the incoming bytes and your
 *     session sees only crumbs of the RX stream (ajtest: 2 of +170
 *     bytes; my goto: 3 of ~140 frames). SIGSTOPping media_server wins
 *     the stream back — but a watchdog reboots the camera roughly 60 s
 *     after media_server freezes, so do test runs in that window and
 *     kill -CONT when done. A reboot restores everything vendor-side; a
 *     plain SIGSTOP of comm_server alone is tolerated indefinitely.
 * The motor side does not care about the readers: commands go out on
 * TX and the MCU obeys them even while media_server swallows the
 * reports — manual verbs (-d u etc.) work with contention, closed-loop
 * goto needs the exclusive window. See Readme.md.
 */

#include <errno.h>
#include <fcntl.h>
#include <math.h>
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
    {'T', "zoom-goto-multiple", 0x00, 0x00, 0x00, 0x00},
    {'s', "stop", 0x00, 0x00, 0x00, 0x00},
};
#define NVERBS (sizeof(VERBS) / sizeof(VERBS[0]))

#define REPORT_LEN 17
#define REPORT_SYNC 0x51

/* 16-byte session/state frame, frozen from a live vendor capture
 * (b6..b9 = lens telemetry at rest). CK filled in at start, byte 15. */
static unsigned char STATE[16] = {0x51, 0x01, 0x04, 0x78, 0x01, 0x0d,
                                  0x59, 0x56, 0x07, 0x20, 0x2f, 0x00,
                                  0x20, 0x00, 0x00, 0x00};
#define STATE_PERIOD_MS 280

static const unsigned char POLL9[9] = {0x51, 0x01, 0x04, 0x79, 0x91,
                                      0x01, 0x00, 0x61, 0x00};

static void usage(const char *prog) {
  printf("Usage: %s -d <key> [-t secs] [-s speed] [-m mult] [-p pos] "
         "[-r ms] [-j] [-D dev] [-B]\n", prog);
  printf("  -d <key>  verb key:\n");
  for (unsigned i = 0; i < NVERBS; i++)
    printf("     %c  %s\n", VERBS[i].key, VERBS[i].name);
  printf("  -t secs   hold the verb then stop (default 1; 0 = verb only —\n");
  printf("            the motor KEEPS RUNNING until a stop frame)\n");
  printf("  -s byte   override both speed bytes (0x00-0xff; vendor UI 1..10 => 5*v+5)\n");
  printf("  -m mult   with -d T: drive to an optical multiple, e.g. 12.5\n");
  printf("            (closed loop on MCU position reports, curve 4.4x..45x)\n");
  printf("  -p pos    with -d T: aim at position 1..30 directly instead of -m\n");
  printf("  -r ms     extra listen window after the move (reports only arrive\n");
  printf("            while the motor moves; long -r keeps the session alive)\n");
  printf("  -j        decode reports as one JSON object per line on stdout (TX\n");
  printf("            diagnostics go to stderr)\n");
  printf("  verb T exits 0 when the final reported position equals the\n");
  printf("            target, 1 on a miss or dead link\n");
  printf("  -D dev    UART device (default /dev/ttyS2)\n");
  printf("  -B        inherit port settings (do not touch termios)\n");
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

static void print_frame(const char *tag, const unsigned char *f, int n, int json) {
  FILE *out = json ? stderr : stdout;
  fprintf(out, "%s:", tag);
  for (int i = 0; i < n; i++)
    fprintf(out, " %02X", f[i]);
  fprintf(out, "\n");
}

/* ---------------- live position tracking ---------------- */

static int g_zoom_pos = -1;   /* last decoded report position, -1 unknown */
static int g_focus_raw = -1;
static unsigned long g_report_frames; /* every CK-valid 17B frame */

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
    g_report_frames++;
    /* zoom bytes 9-10 are DECIMAL DIGITS: tens (0 encoded as 0x3B) x 10
     * + units -> live position 1..30 wide->tele. Focus bytes 13-14 stay
     * raw 16-bit: observed 14 0F / 16 0F, not position-tracking. */
    int zoom = (buf[9] == 0x3B ? 0 : buf[9]) * 10 + buf[10];
    if (zoom >= 1 && zoom <= 30)
      g_zoom_pos = zoom;
    g_focus_raw = buf[13] * 256 + buf[14];
    if (json) {
      printf("{\"zoom_pos\":%d,\"focus_pos\":%d,\"flags\":\"%02X%02X\",\"raw\":\"",
             zoom, g_focus_raw, buf[2], buf[5]);
      for (int i = 0; i < REPORT_LEN; i++)
        printf("%02X", buf[i]);
      printf("\"}\n");
    } else {
      printf("REPORT zoom=%d focus=%d raw:", zoom, g_focus_raw);
      for (int i = 0; i < REPORT_LEN; i++)
        printf(" %02X", buf[i]);
      printf("\n");
    }
    fflush(stdout);
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

/* ---------------- timing ---------------- */

static long now_ms(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

/* ---------------- position <-> optical multiple ----------------
 * Position scale 1..30 spans the full optical range. The module's
 * spec curve is geometric (fixed zoom ratio per step):
 * multiple(pos) = WMIN * r^(pos-1), r = (45/4.4)^(1/29). Fit is
 * exact at both endstops; midpoints uncalibrated (todo: vendor cur_multiple). */
#define MULT_WIDE 4.4
#define MULT_TELE 45.0
#define POS_MAX 30

static double pos_to_multiple(int pos) {
  double r = pow(MULT_TELE / MULT_WIDE, 1.0 / (POS_MAX - 1));
  return MULT_WIDE * pow(r, pos - 1);
}

static int multiple_to_pos(double m) {
  if (m < MULT_WIDE)
    return 1;
  if (m >= MULT_TELE)
    return POS_MAX;
  for (int p = 1; p <= POS_MAX; p++)
    if (pos_to_multiple(p) >= m)
      return p;
  return POS_MAX;
}

/* ---------------- session loop ----------------
 * Streams STATE frames at the vendor cadence, pumps RX, and calls the
 * caller's pump callback once per iteration. Runs until deadline. */

/* a pump may return 1 to end the session early (goto has reached its
 * target; the caller still sends the safety stop frame after the run) */
typedef int (*pump_fn)(void *ctx, long elapsed_ms);

struct session {
  int fd;
  int json;
  unsigned char rbuf[REPORT_LEN];
  int rlen;
  long last_state_ms;
  long last_poll_ms;
};

static void session_drain_rx(struct session *s) {
  unsigned char buf[256];
  for (;;) {
    ssize_t got = read(s->fd, buf, sizeof(buf)); /* fd is O_NONBLOCK */
    if (got <= 0)
      return;
    for (ssize_t i = 0; i < got; i++)
      report_feed(s->rbuf, &s->rlen, buf[i], s->json);
    if (got < (ssize_t)sizeof(buf))
      return;
  }
}

static void session_init(struct session *s, int fd, int json) {
  memset(s, 0, sizeof(*s));
  s->fd = fd;
  s->json = json;
  s->last_state_ms = -100000;
  s->last_poll_ms = now_ms();

  /* vendor boot handshake — one-way; harmless, kept for fidelity.
   * Paced like ajtest (300-500 ms gaps), the proven-good replication. */
  static const unsigned char END[] = {0x45, 0x4e, 0x44, 0xd7};
  static const unsigned char VER[] = {0x56, 0x45, 0x52, 0xed};
  static const unsigned char EXVER[] = {0x45, 0x58, 0x56, 0x45, 0x52, 0x8a};
  write(fd, END, sizeof END);
  usleep(300000);
  write(fd, END, sizeof END);
  usleep(300000);
  write(fd, VER, sizeof VER);
  usleep(500000);
  write(fd, EXVER, sizeof EXVER);
  usleep(500000);
  write(fd, EXVER, sizeof EXVER);
  usleep(400000);

  STATE[15] = 0; /* in case of a re-run */
  for (int i = 0; i < 15; i++)
    STATE[15] += STATE[i];

  /* Prime the session like the vendor cold start (ajtest-verified): the
   * MCU stays deaf to commands — no ACKs, no motion reports — until it
   * has seen a live state stream for a few seconds. Two frames buy
   * silence; 14 frames (~280 ms apart, ≈4 s) wake it up. */
  for (int i = 0; i < 14; i++) {
    write(fd, STATE, sizeof STATE);
    usleep(280000);
  }
  s->last_state_ms = now_ms();
}

/* one 20 ms slice: stream STATE when due, poll rarely, drain RX */
static void session_slice(struct session *s) {
  long now = now_ms();
  if (now - s->last_state_ms >= STATE_PERIOD_MS) {
    write(s->fd, STATE, sizeof STATE);
    s->last_state_ms = now;
  }
  if (now - s->last_poll_ms >= 10000) {
    write(s->fd, POLL9, sizeof POLL9);
    s->last_poll_ms = now;
  }
  session_drain_rx(s);
  usleep(20000);
  session_drain_rx(s);
}

/* run the session for `total_ms`, calling pump(ctx) every slice */
static void session_run(struct session *s, long total_ms, pump_fn pump, void *ctx) {
  long t0 = now_ms();
  while (now_ms() - t0 < total_ms) {
    session_slice(s);
    if (pump && pump(ctx, now_ms() - t0))
      break; /* pump signalled completion */
  }
}

/* ---------------- verbs ---------------- */

struct move_ctx {
  int fd;
  int json;
  int done_ms;       /* when the auto-stop fired, 0 = not yet */
  long hold_ms;
  unsigned char stop_frame[7];
  int need_stop;
  const struct verb *verb;
};

static int move_pump(void *vctx, long elapsed) {
  struct move_ctx *m = vctx;
  if (m->need_stop && elapsed >= m->hold_ms) {
    print_frame("TX", m->stop_frame, 7, m->json);
    write(m->fd, m->stop_frame, 7);
    m->need_stop = 0;
    m->done_ms = elapsed;
  }
  return 0; /* keep the session for the -r listen window */
}

struct goto_ctx {
  int fd;
  int target;
  unsigned char tele[7];
  unsigned char wide[7];
  unsigned char stop[7];
  unsigned char pulse_tele[7];
  unsigned char pulse_wide[7];
  int json;
  /* state machine: 0 approach, 1 halted/settling, 2 done, 3 correcting */
  int phase;
  long phase_t0;
  int sent_dir;    /* +1 tele command in flight, -1 wide, 0 none */
  int start_side;  /* sign(first_pos - target), 0 unknown */
  long last_report_ms;
  int corr_done;   /* the single correction round has been used */
  unsigned long frames_seen;
};

static void goto_send(struct goto_ctx *g, const unsigned char *f, int n) {
  print_frame("TX", f, n, g->json);
  write(g->fd, f, n);
}

static int goto_pump(void *vctx, long elapsed) {
  struct goto_ctx *g = vctx;
  if (g->phase == 2)
    return 1; /* done — session_run exits after this slice */

  if (g_report_frames != g->frames_seen) { /* a NEW frame just arrived */
    g->frames_seen = g_report_frames;
    g->last_report_ms = elapsed;
  }

  if (g->phase == 0) {
    if (g->start_side == 0 && g_zoom_pos >= 0) {
      g->start_side = g_zoom_pos == g->target ? 0
                    : g_zoom_pos > g->target ? 1 : -1;
      if (g->start_side == 0) {
        goto_send(g, g->stop, 7);
        g->sent_dir = 0;
        g->phase = 1;
        g->phase_t0 = elapsed;
        return 0;
      }
      int want = g->start_side > 0 ? -1 : +1; /* towards target */
      if (want != g->sent_dir) { /* initial guess was wrong */
        goto_send(g, g->stop, 7);
        goto_send(g, want > 0 ? g->tele : g->wide, 7);
        g->sent_dir = want;
        return 0;
      }
    }
    if (g_zoom_pos < 0)
      return 0; /* wait for the first report (only arrives in motion) */
    if (elapsed - g->last_report_ms > 4000) { /* dead link */
      goto_send(g, g->stop, 7);
      g->phase = 2;
      return 0;
    }
    /* crossed the target while moving? */
    if ((g->start_side > 0 && g_zoom_pos <= g->target) ||
        (g->start_side < 0 && g_zoom_pos >= g->target)) {
      goto_send(g, g->stop, 7);
      g->sent_dir = 0;
      g->phase = 1;
      g->phase_t0 = elapsed;
    }
    return 0;
  }

  if (g->phase == 1) {
    if (elapsed - g->phase_t0 < 700)
      return 0; /* let the lens halt; reports may keep arriving */
    if (g->corr_done) { /* correction round spent: accept +-1, done */
      g->phase = 2;
      return 0;
    }
    int delta = g_zoom_pos - g->target;
    if (delta == 0 || g_zoom_pos < 0) {
      g->phase = 2;
      return 0;
    }
    /* ONE slow closed-loop approach toward the target. Short pulses are
     * invisible to the loop (reports are sparse at crawl speed — pulses
     * moved the lens 7->12 with no feedback), so drive continuously at
     * crawl and stop on the next crossing report. */
    goto_send(g, delta < 0 ? g->pulse_tele : g->pulse_wide, 7);
    g->sent_dir = delta < 0 ? +1 : -1;
    g->start_side = delta;
    g->corr_done = 1;
    g->phase = 3;
    g->phase_t0 = elapsed;
    return 0;
  }

  if (g->phase == 3) { /* slow approach: stop on crossing or budget end */
    int crossed = (g->start_side > 0 && g_zoom_pos <= g->target) ||
                  (g->start_side < 0 && g_zoom_pos >= g->target);
    if (crossed || elapsed - g->phase_t0 > 4000) {
      goto_send(g, g->stop, 7);
      g->sent_dir = 0;
      g->phase = 4;
      g->phase_t0 = elapsed;
    }
    return 0;
  }

  if (g->phase == 4) { /* final settle, then done regardless */
    if (elapsed - g->phase_t0 >= 800)
      g->phase = 2;
  }
  return 0;
}

int main(int argc, char **argv) {
  const char *dev = "/dev/ttyS2";
  const struct verb *verb = NULL;
  double hold = 1.0;
  int speed = -1;
  int rx_ms = 1200;
  int inherit = 0;
  int json = 0;
  double mult = -1.0;
  int pos_target = -1;
  int rc = 0;

  int opt;
  while ((opt = getopt(argc, argv, "d:t:s:m:p:r:D:Bjh")) != -1) {
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
    case 'm':
      mult = atof(optarg);
      if (mult <= 0) {
        fprintf(stderr, "multiple must be positive\n");
        return 2;
      }
      break;
    case 'p':
      pos_target = atoi(optarg);
      if (pos_target < 1 || pos_target > POS_MAX) {
        fprintf(stderr, "position must be 1..%d\n", POS_MAX);
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
  if (verb->key == 'T' && mult <= 0 && pos_target < 0) {
    fprintf(stderr, "verb T needs -m <multiple> (e.g. 12.5) or -p <pos 1..30>\n");
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
      /* 57600 8N1 — verified empirically (115200 makes the MCU deaf:
       * rx counter flat, zero ACKs; at 57600 it ACKs and streams
       * reports). The vendor rate also lives in comm_server as the
       * c_cflag constant B57600|CS8|CREAD|CLOCAL. */
      cfsetispeed(&t, B57600);
      cfsetospeed(&t, B57600);
      tcsetattr(fd, TCSANOW, &t);
    }
  }
  tcflush(fd, TCIOFLUSH);

  struct session s;
  session_init(&s, fd, json);

  if (verb->key == 'T') {
    int target = pos_target > 0 ? pos_target : multiple_to_pos(mult);
    fprintf(stderr, "goto position %d (multiple %.1f)\n", target,
            pos_to_multiple(target));

    struct goto_ctx g;
    memset(&g, 0, sizeof(g));
    g.fd = fd;
    g.target = target;
    g.json = json;
    g.phase = 0;
    g.last_report_ms = 0;
    build_frame(g.tele, &VERBS[0], speed);  /* u = zoom-tele */
    build_frame(g.wide, &VERBS[1], speed);  /* d = zoom-wide */
    { const struct verb *stopv = &VERBS[NVERBS - 1];
      build_frame(g.stop, stopv, -1); }
    build_frame(g.pulse_tele, &VERBS[0], 0x0a);  /* slow corrections */
    build_frame(g.pulse_wide, &VERBS[1], 0x0a);

    /* first move: blind (position only arrives once moving) — start tele
     * unless the target is in the lower half; corrected on first report */
    goto_send(&g, target >= 15 ? g.tele : g.wide, 7);
    g.sent_dir = target >= 15 ? +1 : -1;
    session_run(&s, 30000, goto_pump, &g);
    /* ALWAYS stop: budget exhaustion or a dead link must not leave the
     * motor running into an endstop (a tele runaway reached the tele
     * stop once because reports were being consumed by another process) */
    write(fd, g.stop, 7);
    fprintf(stderr, "final position %d\n", g_zoom_pos);
    rc = g_zoom_pos != g.target; /* exit 0 on target, 1 on miss/dead link */
  } else {
    unsigned char f[7];
    build_frame(f, verb, speed);
    print_frame("TX", f, 7, json);
    write(fd, f, 7);

    struct move_ctx m;
    memset(&m, 0, sizeof(m));
    m.fd = fd;
    m.json = json;
    m.need_stop = (hold > 0 && verb->key != 's');
    m.hold_ms = (long)(hold * 1000);
    { const struct verb *stopv = &VERBS[NVERBS - 1];
      build_frame(m.stop_frame, stopv, -1); }

    long total = 600 + m.hold_ms + rx_ms; /* warm-up + hold + tail */
    if (verb->key == 's')
      total = 600 + rx_ms;
    session_run(&s, total, move_pump, &m);
  }

  close(fd);
  return rc;
}
