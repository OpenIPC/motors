#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/poll.h>
#include <termios.h>
#include <unistd.h>

const unsigned char init[] = {0xa5, 0x7b, 0x9e, 0xf0, 0xef, 0xee, 0xe0, 0xf4};

const unsigned char zoom_in[] = {0xc5, 0x01, 0x00, 0x20,
                                 0x00, 0x00, 0x21, 0x5c};
const unsigned char zoom_out[] = {0xc5, 0x1, 0x0, 0x40, 0x0, 0x0, 0x41, 0x5c};
const unsigned char cancel[] = {0xc5, 0x1, 0x0, 0x0, 0x0, 0x0, 0x1, 0x5c};

static void send_cmd(int fd, const char *name, const unsigned char *cmd) {
  puts(name);
  write(fd, cmd, 8);
}

#define CMD(name, data)                                                        \
  send_cmd(uart, name, data);                                                  \
  break;

static void DumpHex(const void *data, size_t size) {
  char ascii[17];
  size_t i, j;
  ascii[16] = '\0';
  for (i = 0; i < size; ++i) {
    printf("%02X ", ((unsigned char *)data)[i]);
    if (((unsigned char *)data)[i] >= ' ' &&
        ((unsigned char *)data)[i] <= '~') {
      ascii[i % 16] = ((unsigned char *)data)[i];
    } else {
      ascii[i % 16] = '.';
    }
    if ((i + 1) % 8 == 0 || i + 1 == size) {
      printf(" ");
      if ((i + 1) % 16 == 0) {
        printf("|  %s \n", ascii);
      } else if (i + 1 == size) {
        ascii[(i + 1) % 16] = '\0';
        if ((i + 1) % 16 <= 8) {
          printf(" ");
        }
        for (j = (i + 1) % 16; j < 16; ++j) {
          printf("   ");
        }
        printf("|  %s \n", ascii);
      }
    }
  }
}

int main() {
  struct termios ctrl;
  tcgetattr(STDIN_FILENO, &ctrl);
  ctrl.c_lflag &= ~ICANON; // turning off canonical mode makes input unbuffered
  ctrl.c_lflag &= ~ECHO;   // disable echo
  tcsetattr(STDIN_FILENO, TCSANOW, &ctrl);

  int uart = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY);
  if (uart == -1) {
    printf("Error no is : %d\n", errno);
    printf("Error description is : %s\n", strerror(errno));
    return (-1);
  };

  struct termios options;
  tcgetattr(uart, &options);
  cfsetspeed(&options, B115200);

  options.c_cflag &= ~CSIZE;  // Mask the character size bits
  options.c_cflag |= CS8;     // 8 bit data
  options.c_cflag &= ~PARENB; // set parity to no
  options.c_cflag &= ~PARODD; // set parity to no
  options.c_cflag &= ~CSTOPB; // set one stop bit

  options.c_cflag |= (CLOCAL | CREAD);

  options.c_oflag &= ~OPOST;

  options.c_lflag &= 0;
  options.c_iflag &= 0; // disable software flow controll
  options.c_oflag &= 0;

  cfmakeraw(&options);
  tcsetattr(uart, TCSANOW, &options);

  int flags = fcntl(uart, F_GETFL, 0);
  fcntl(uart, F_SETFL, flags | O_NONBLOCK);
  write(uart, init, 8);

  printf("Xingongmai Motors, get in a car and fasten your safety belt\n");
  printf("Commands: + - Enter (to cancel)\n");

  while (1) {

    struct pollfd pfds[2] = {
        {.fd = 0, .events = POLLIN},
        {.fd = uart, .events = POLLIN},
    };

    poll(pfds, 2, -1);

    if (pfds[0].revents & POLLIN) {
      char ch;
      int i = read(0, &ch, 1);

      if (!i) {

        printf("stdin closed\n");
        return 0;
      }
      switch (ch) {
      case '-':
        CMD("Zoom out", zoom_out);

      case '+':
        CMD("Zoom in", zoom_in);

      case '\n':
        CMD("Cancel", cancel);

      default:
        printf("Unknown command %c\n", ch);
      }
    }

    if (pfds[1].revents & POLLIN) {
      unsigned char rbuf[1024];

      int i = read(uart, rbuf, sizeof(rbuf));

      if (!i) {
        printf("UART closed\n");
        return 0;
      }

      if (i != -1)
        DumpHex(rbuf, i);
    }
  }
  write(uart, cancel, 8);
  close(uart);
}
