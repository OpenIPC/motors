/*
 * pelco-bb: send Pelco-D (or any bytes) at 9600 8N1 on the PTZ port of the
 * Xiongmai HI3516D_N81820 (HiSilicon Hi3516A/D V100) by bit-banging the UART1
 * pads, for debugging an RS-485 PTZ head when the UART path does not reach it.
 *
 * On the lab N81820 the bus pair sees UART1 TXD/RXD single-ended from CN2 pins
 * 3/4. A lone TXD swing never pulls A below B, so frames written to
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
 * and the head drops the damaged copy on its checksum. The pads, the GPIO
 * direction register and GPIO9_2 are restored on exit, so UART1 (majestic's
 * ptz port) works again afterwards.
 *
 * usage: pelco-bb [-i] [-b baud] [-r repeat] <hex byte>...
 */
#include <fcntl.h>
#include <sched.h>
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

static volatile uint32_t *iocfg, *gpio;

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

static void usage(const char *prog)
{
	fprintf(stderr,
		"usage: %s [-i] [-b baud] [-r repeat] <hex byte>...\n"
		"  default  drive the UART1 TXD/RXD pads as a complementary pair\n"
		"  -i       drive the TXD pad only, inverted, with GPIO9_2 high\n"
		"  -b baud  bit rate (default 9600)\n"
		"  -r N     send the bytes N times (default 1)\n"
		"example: %s -r 100 ff 01 00 04 20 00 25   # Pelco-D addr 1 pan left\n",
		prog, prog);
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
			baud = atoi(optarg);
			break;
		case 'r':
			repeat = atoi(optarg);
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
	for (int i = 0; i < n; i++)
		msg[i] = strtoul(argv[optind + i], NULL, 16);

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

	uint32_t pad_txd = iocfg[PAD_TXD], pad_rxd = iocfg[PAD_RXD];
	uint32_t dir = gpio[GPIO_DIR];
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
	for (int r = 0; r < repeat; r++) {
		int tries = 0, late;
		do {
			late = 0;
			for (int i = 0; i < n; i++) {
				/* start bit, 8 data bits LSB first, stop bit */
				uint32_t frame = 0x200 | (uint32_t)msg[i] << 1;
				for (int k = 0; k < 10; k++) {
					int one = (frame >> k) & 1;
					until(t);
					if (inverted)
						pins(mask, one ? 0 : BIT_TXD);
					else
						pins(mask, one ? BIT_TXD : BIT_RXD);
					if (now_ns() - t > bit / 5)
						late = 1;
					t += bit;
				}
			}
			t += 2 * bit;	/* inter-frame idle */
			if (late)
				resent++;
		} while (late && ++tries < TRIES);
		if (late)
			failed++;
	}
	until(t);

	if (inverted)
		pins(BIT_DE, 0);
	else
		pins(BIT_TXD, BIT_TXD);
	iocfg[PAD_TXD] = pad_txd;
	iocfg[PAD_RXD] = pad_rxd;
	gpio[GPIO_DIR] = inverted ? dir | BIT_DE : dir;

	if (resent || failed)
		fprintf(stderr, "pelco-bb: %d frame(s) resent, %d not delivered\n",
			resent, failed);
	return failed ? 1 : 0;
}
