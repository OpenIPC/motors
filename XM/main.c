#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

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

  printf("Xingongmai Motors, get in a car and fasten your safety belt\n");
  printf("Commands: +/-\n");

  unsigned char rbuf[1024];
  char ch;

  while (read(STDIN_FILENO, &ch, 1)) {

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
  close(uart);
}
