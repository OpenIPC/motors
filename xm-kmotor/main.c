#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#define MOTOR_STATUS 0x80184D02
#define MOTOR_MAXSTEPS 0x80184D03
#define MOTOR_CMD 0x40184D01u

/*
        s[0] = 1; //left
        s[0] = 2; //right
        s[0] = 4; //up
        s[0] = 8; //down
        s[0] = 18; //stop
        s[0] = 6; //rightup
        s[0] = 10; //rightdown
        s[0] = 5; //leftup
        s[0] = 9; //leftdown
        s[0] = 17; //goto position
        s[0] = 20; //set position
        s[0] = 16; //x scan
        s[0] = 19; //x y steps
*/

void motor_ioctl(int cmd, int *arg) {
  int fd = open("/dev/motor", O_WRONLY);
  ioctl(fd, cmd, arg);
  close(fd);
}

void motor_status_get(int *cmd) { motor_ioctl(MOTOR_STATUS, cmd); }

void motor_get_maxsteps(int *cmd) { motor_ioctl(MOTOR_MAXSTEPS, cmd); }

void motor_steps(int xpos, int ypos, int sspeed) {
  int s[5];
  s[0] = 19;
  s[1] = sspeed; // xspeed
  s[2] = sspeed; // yspeed
  s[3] = xpos;   // steps_x
  s[4] = ypos;   // steps_y
  motor_ioctl(MOTOR_CMD, s);
}

void motor_scan(int sspeed, int ypos) {
  int s[5];
  int maxsteps[2];
  motor_get_maxsteps(maxsteps);
  s[0] = 16;
  s[1] = sspeed;      // xspeed
  s[2] = 0;           // left_limit_x
  s[3] = ypos;        // maxsteps[2]; //200; //y_position
  s[4] = maxsteps[1]; // 3300; //right_limit_x
  motor_ioctl(MOTOR_CMD, s);
}

void motor_xy_position(int xpos, int ypos, int sspeed) {
  int s[5];
  s[0] = 17;
  s[1] = xpos;
  s[2] = ypos;
  s[3] = sspeed; // xspeed
  s[4] = sspeed; // yspeed
  motor_ioctl(MOTOR_CMD, s);
}

void motor_set_position(int xpos, int ypos) {
  int s[3];
  s[0] = 20;
  s[1] = xpos;
  s[2] = ypos;
  motor_ioctl(MOTOR_CMD, s);
}

void show_sage() {
  int steps[6];
  int maxsteps[3];

  motor_get_maxsteps(maxsteps);
  printf("Max X Steps %d.\n", maxsteps[1]);
  printf("Max Y Steps %d.\n", maxsteps[2]);

  motor_status_get(steps);
  printf("Status Move: %d.\n", steps[0]);
  printf("X Steps %d.\n", steps[1]);
  printf("Y Steps %d.\n", steps[2]);
  printf("Unknow: %d.\n", steps[3]);
  printf("Unknow: %d.\n", steps[4]);
  printf("Unknow: %d.\n", steps[5]);
}

void JSON_status(){
  //return xpos,ypos and status in JSON string
  //allows passing straight back to async call
  //with little effort and ability to track x,y position
  int steps[6];

  motor_status_get(steps);
  printf("{");
  printf("\"status\":\"%d\"", steps[0]);
  printf(",");
  printf("\"xpos\":\"%d\"", steps[1]);
  printf(",");
  printf("\"ypos\":\"%d\"", steps[2]);
  printf(",");
  printf("\"unkown\":\"%d\"", steps[3]);
  printf(",");
  printf("\"unkown\":\"%d\"", steps[4]);
  printf(",");
  printf("\"unkown\":\"%d\"", steps[5]);
  printf("}");
}

void JSON_initial(){
  //return all known parameters in JSON string
  //idea is when client page loads in browser we
  //get current details from camera
  int steps[6];
  int maxsteps[3];

  motor_status_get(steps);
  printf("{");
  printf("\"status\":\"%d\"", steps[0]);
  printf(",");
  printf("\"xpos\":\"%d\"", steps[1]);
  printf(",");
  printf("\"ypos\":\"%d\"", steps[2]);

  motor_get_maxsteps(maxsteps);
  printf(",");
  printf("\"xmax\":\"%d\"", maxsteps[1]);
  printf(",");
  printf("\"ymax\":\"%d\"", maxsteps[2]);
  printf(",");
  printf("\"maxstep 0 is \":\"%d\"", maxsteps[0]);

  printf("}");

}


void sendCommand(int cmd, int sspeed) {
  int s[3];
  s[0] = cmd;
  s[1] = sspeed; // xspeed
  s[2] = sspeed; // yspeed
  motor_ioctl(MOTOR_CMD, s);

  show_sage();
}

int main(int argc, char *argv[]) {
  char direction = 's';
  int stepspeed = 5;
  int xpos = 0;
  int ypos = 0;
  int c;

  while ((c = getopt(argc, argv, "d:s:x:y:")) != -1) {
    switch (c) {
    case 'd':
      direction = optarg[0];
      break;
    case 's':
      if (atoi(optarg) > 10) {
        stepspeed = 10;
      } else {
        stepspeed = atoi(optarg);
      }

      break;
    case 'x':
      xpos = atoi(optarg);
      break;
    case 'y':
      ypos = atoi(optarg);
      break;
    case 'j':
      //get x and y current positions and status
      JSON_status();
      exit (EXIT_SUCCESS);
      break;
    case 'i':
      //get all initial values
      JSON_initial();
      exit (EXIT_SUCCESS);
      break;
    default:
      printf("Invalid Argument %c\n", c);
      printf("Usage : %s\n"
             "\t -d Direction step\n"
             "\t -s Speed step (default 5)\n"
             "\t -x X position/step (default 0)\n"
             "\t -y Y position/step (default 0) .\n"
             "\t -j return json string xpos,ypos,status.\n"
             "\t -i return all camera parameters\n",
             argv[0]);
      exit(EXIT_FAILURE);
    }
  }

  switch (direction) {
  case 'u': // up
    sendCommand(4, stepspeed);
    break;

  case 'd': // down
    sendCommand(8, stepspeed);
    break;

  case 'l': // left
    sendCommand(1, stepspeed);
    break;

  case 'r': // right
    sendCommand(2, stepspeed);
    break;

  case 'e': // rightup
    sendCommand(6, stepspeed);
    break;

  case 'c': // rightdown
    sendCommand(10, stepspeed);
    break;

  case 'q': // leftup
    sendCommand(5, stepspeed);
    break;

  case 'z': // leftdown
    sendCommand(9, stepspeed);
    break;

  case 's': // stop
    sendCommand(18, stepspeed);
    break;
  case 'h': // set position
    motor_set_position(xpos, ypos);
    break;

  case 't': // xy position
    motor_xy_position(xpos, ypos, stepspeed);
    break;

  case 'f': // x scan
    motor_scan(stepspeed, ypos);
    break;

  case 'g': // x y steps
    motor_steps(xpos, ypos, stepspeed);
    break;

  default:
    printf("Invalid Direction Argument %c\n", direction);
    printf("Usage : %s -d\n"
           "\t u (Up)\n"
           "\t d (Down)\n"
           "\t l (Left)\n"
           "\t r (Right)\n"
           "\t e (Right Up)\n"
           "\t c (Right Down)\n"
           "\t q (Left Up)\n"
           "\t z (Left Down)\n"
           "\t s (Stop)\n"
           "\t h (Set position X and Y)\n"
           "\t t (Go to X and Y)\n"
           "\t f (Scan, Y to set)\n"
           "\t g (Steps X and Y)\n",
           argv[0]);
    exit(EXIT_FAILURE);
  }
  return 0;
}
