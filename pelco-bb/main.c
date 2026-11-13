/*
 * pelco-bb: send Pelco-D (or any bytes) at 9600 8N1 on the PTZ port of the
 * Xiongmai HI3516D_N81820 (HiSilicon Hi3516A/D V100) by bit-banging the UART1
 * pads, for debugging an RS-485 PTZ head when the UART path does not reach it.
 *
 * The N81820's CN2 pins 3/4 are UART1 RXD/TXD at 3.3 V TTL, with no
 * transceiver (proven with a TTL USB-UART on plain /dev/ttyAMA1). On a bare
 * RS-485 pair a lone TXD swing never pulls A below B, so frames written to
 * /dev/ttyAMA1 never show up on the bus, whatever GPIO9_2 ("DE" in the stock
 * libdvr) does. The default mode turns both pads into GPIOs and drives them as
 * a complementary pair -- TXD pad (GPIO9_5, bus A) = data, RXD pad (GPIO9_3,
 * bus B) = NOT data -- which is a real differential signal.
 *
 * -i drives only the TXD pad, with the inverted waveform, and raises GPIO9_2
 * around the burst. It is for an A/B pair wired the other way round. The pad
 * is set to the bus-mark level before GPIO9_2 rises, so the line does not drop
 * into a break first.
 *
 * Timing is a busy-wait on CLOCK_MONOTONIC under SCHED_FIFO. An interrupt can
 * still delay an edge, so each edge's lateness is measured after the write: a
 * frame with an edge later than 20 % of a bit is sent again (up to 5 tries),
 * after a fresh idle gap measured from the real time of the last edge, and the
 * head drops the damaged copy on its checksum. The pads, the GPIO direction
 * register and GPIO9_2's level are restored on exit, including after SIGINT,
 * SIGTERM or SIGHUP, so UART1 (majestic's ptz port) works again afterwards.
 *
 * usage: pelco-bb [-i] [-b baud] [-r repeat] <hex byte>...
 */
#include <errno.h>
#include <fcntl.h>
#include <sched.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#define IOCFG	0x200f0000
#define GPIO9	0x201d0000
#define GPIO_DIR (0x400 / 4)
#define PAD_RXD	(0x07c / 4)	/* UART1_RXD / GPIO9_3 */
#define PAD_TXD	(0x084 / 4)	/* UART1_TXD / GPIO9_5 */
#define BIT_DE	(1 << 2)	/* GPIO9_2 */
#define BIT_RXD	(1 << 3)	/* GPIO9_3 */
#define BIT_TXD	(1 << 5)	/* GPIO9_5 */
#define TRIES	5
#define BAUD_MIN 50
#define BAUD_MAX 115200

static volatile uint32_t *iocfg, *gpio;
static volatile sig_atomic_t stop;

static void on_signal(int sig)
{
	(void)sig;
	stop = 1;
}

/* PL061 masked data register: only the bits in the address mask change */
static void pins(uint32_t mask, uint32_t val)
{
	gpio[mask] = val;
}

static uint64_t now_ns(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000000ull + ts.tv_nsec;
}

static void until(uint64_t t)
{
	while (now_ns() < t)
		;
}

/* Parse a whole decimal or hex argument into [lo, hi]; -1 if it is not one. */
static long parse(const char *s, int base, long lo, long hi)
{
	char *end;

	errno = 0;
	long v = strtol(s, &end, base);
	if (errno || end == s || *end || v < lo || v > hi)
		return -1;
	return v;
}

static void usage(const char *prog)
{
	fprintf(stderr,
		"usage: %s [-i] [-b baud] [-r repeat] <hex byte>...\n"
		"  default  drive the UART1 TXD/RXD pads as a complementary pair\n"
		"  -i       drive the TXD pad only, inverted, with GPIO9_2 high\n"
		"  -b baud  bit rate, %d..%d (default 9600)\n"
		"  -r N     send the bytes N times (default 1)\n"
		"example: %s -r 100 ff 01 00 04 20 00 25   # Pelco-D addr 1 pan left\n",
		prog, BAUD_MIN, BAUD_MAX, prog);
}

int main(int argc, char **argv)
{
	int baud = 9600, repeat = 1, inverted = 0, opt;

	while ((opt = getopt(argc, argv, "ib:r:h")) != -1) {
		switch (opt) {
		case 'i':
			inverted = 1;
			break;
		case 'b':
			baud = parse(optarg, 10, BAUD_MIN, BAUD_MAX);
			break;
		case 'r':
			repeat = parse(optarg, 10, 1, 1000000);
			break;
		default:
			usage(argv[0]);
			return 2;
		}
	}
	int n = argc - optind;
	if (n <= 0 || baud <= 0 || repeat <= 0) {
		usage(argv[0]);
		return 2;
	}
	uint8_t *msg = malloc(n);
	if (!msg)
		return 1;
	for (int i = 0; i < n; i++) {
		long v = parse(argv[optind + i], 16, 0, 0xff);
		if (v < 0) {
			fprintf(stderr, "%s: not a hex byte: %s\n", argv[0], argv[optind + i]);
			return 2;
		}
		msg[i] = v;
	}

	int fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (fd < 0) {
		perror("/dev/mem");
		return 1;
	}
	iocfg = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, IOCFG);
	gpio = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, GPIO9);
	if (iocfg == MAP_FAILED || gpio == MAP_FAILED) {
		perror("mmap");
		return 1;
	}

	struct sched_param sp = { .sched_priority = sched_get_priority_max(SCHED_FIFO) };
	if (sched_setscheduler(0, SCHED_FIFO, &sp) < 0)
		perror("sched_setscheduler (timing will suffer)");
	mlockall(MCL_CURRENT | MCL_FUTURE);

	/* Everything we touch, so the exit path can put it back exactly. */
	uint32_t pad_txd = iocfg[PAD_TXD], pad_rxd = iocfg[PAD_RXD];
	uint32_t dir = gpio[GPIO_DIR];
	uint32_t level = gpio[BIT_DE | BIT_RXD | BIT_TXD];

	struct sigaction sa = { .sa_handler = on_signal };
	sigemptyset(&sa.sa_mask);
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);
	sigaction(SIGHUP, &sa, NULL);

	uint64_t bit = 1000000000ull / baud;
	uint32_t mask = inverted ? BIT_TXD : BIT_TXD | BIT_RXD;

	/* Set the bus-mark levels before the pads leave the UART. */
	if (inverted) {
		pins(BIT_TXD, 0);
		gpio[GPIO_DIR] = dir | BIT_TXD | BIT_DE;
		iocfg[PAD_TXD] = 0;
		pins(BIT_DE, BIT_DE);
	} else {
		pins(BIT_TXD | BIT_RXD, BIT_TXD);
		gpio[GPIO_DIR] = dir | BIT_TXD | BIT_RXD;
		iocfg[PAD_TXD] = 0;
		iocfg[PAD_RXD] = 0;
	}

	uint64_t t = now_ns() + 2 * bit;	/* two bit times of clean mark */
	int resent = 0, failed = 0;
	for (int r = 0; r < repeat && !stop; r++) {
		int tries = 0, late;
		do {
			uint64_t last = 0;	/* when the stop bit was really written */
			late = 0;
			for (int i = 0; i < n && !stop; i++) {
				/* start bit, 8 data bits LSB first, stop bit */
				uint32_t frame = 0x200 | (uint32_t)msg[i] << 1;
				for (int k = 0; k < 10; k++) {
					int one = (frame >> k) & 1;
					until(t);
					if (inverted)
						pins(mask, one ? 0 : BIT_TXD);
					else
						pins(mask, one ? BIT_TXD : BIT_RXD);
					last = now_ns();
					if (last - t > bit / 5)
						late = 1;
					t += bit;
				}
			}
			/*
			 * Inter-frame idle: two bit times of mark after the stop bit.
			 * If an interrupt pushed the real edges behind the schedule,
			 * count the gap from the real stop bit, or a retry would start
			 * with no mark at all and the head could not resync.
			 */
			t += 2 * bit;
			if (t < last + 3 * bit)
				t = last + 3 * bit;
			if (late)
				resent++;
		} while (late && !stop && ++tries < TRIES);
		if (late)
			failed++;
	}
	until(t);

	/* Bus mark, then back to the UART with the exact saved state. */
	if (inverted)
		pins(BIT_TXD, 0);
	else
		pins(BIT_TXD | BIT_RXD, BIT_TXD);
	pins(BIT_DE, level & BIT_DE);
	iocfg[PAD_TXD] = pad_txd;
	iocfg[PAD_RXD] = pad_rxd;
	gpio[GPIO_DIR] = dir;
	pins(BIT_DE | BIT_RXD | BIT_TXD, level);

	if (resent || failed)
		fprintf(stderr, "pelco-bb: %d frame(s) resent, %d not delivered\n",
			resent, failed);
	if (stop) {
		fprintf(stderr, "pelco-bb: interrupted, pads restored\n");
		return 130;
	}
	return failed ? 1 : 0;
}
