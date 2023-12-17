#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#define MOTOR_MOVE_STOP 0x0
#define MOTOR_MOVE_RUN 0x1

/* directional_attr */
#define MOTOR_DIRECTIONAL_UP 0x0
#define MOTOR_DIRECTIONAL_DOWN 0x1
#define MOTOR_DIRECTIONAL_LEFT 0x2
#define MOTOR_DIRECTIONAL_RIGHT 0x3

#define MOTOR1_MAX_SPEED 1000
#define MOTOR1_MIN_SPEED 10

/* ioctl cmd */
#define MOTOR_STOP 0x1
#define MOTOR_RESET 0x2
#define MOTOR_MOVE 0x3
#define MOTOR_GET_STATUS 0x4
#define MOTOR_SPEED 0x5
#define MOTOR_GOBACK 0x6
#define MOTOR_CRUISE 0x7

enum motor_status
{
  MOTOR_IS_STOP,
  MOTOR_IS_RUNNING,
};

struct motor_message
{
  int x;
  int y;
  enum motor_status status;
  int speed;
  /* these two members are not standard from the original kernel module */
  unsigned int x_max_steps;
  unsigned int y_max_steps;
};

struct motors_steps
{
  int x;
  int y;
};

struct motor_move_st
{
  int motor_directional;
  int motor_move_steps;
  int motor_move_speed;
};
struct motor_status_st
{
  int directional_attr;
  int total_steps;
  int current_steps;
  int min_speed;
  int cur_speed;
  int max_speed;
  int move_is_min;
  int move_is_max;
};

struct motor_reset_data
{
  unsigned int x_max_steps;
  unsigned int y_max_steps;
  unsigned int x_cur_step;
  unsigned int y_cur_step;
};

int fd = -1;

void motor_ioctl(int cmd, void *arg)
{
  // printf("[IOCTL] %d, %p\n", cmd, arg);
  ioctl(fd, cmd, arg);
}

void motor_status_get(struct motor_message *msg)
{
  motor_ioctl(MOTOR_GET_STATUS, msg);
}

void motor_get_maxsteps(unsigned int *maxx, unsigned int *maxy)
{
  struct motor_message msg;
  motor_status_get(&msg);
  if (maxx)
    *maxx = msg.x_max_steps;
  if (maxy)
    *maxy = msg.y_max_steps;
}

int motor_is_busy()
{
  struct motor_message msg;
  motor_status_get(&msg);
  return msg.status == MOTOR_IS_RUNNING ? 1 : 0;
}

void motor_wait_idle()
{
  while (motor_is_busy())
  {
    usleep(100000);
  }
    printf(" == moving, waiting...\n");
}

void motor_steps(int xsteps, int ysteps, int stepspeed)
{
  struct motors_steps steps;
  steps.x = xsteps;
  steps.y = ysteps;

  printf(" -> steps, X %d, Y %d, speed %d\n", steps.x, steps.y, stepspeed);
  motor_ioctl(MOTOR_SPEED, &stepspeed);
  motor_ioctl(MOTOR_MOVE, &steps);

  motor_wait_idle();
}

void motor_set_position(int xpos, int ypos, int stepspeed)
{
  struct motor_message msg;
  motor_status_get(&msg);

  int deltax = xpos - msg.x;
  int deltay = ypos - msg.y;

  printf(" -> set position current X: %d, Y: %d, steps required X: %d, Y: %d, speed %d\n", msg.x, msg.y, deltax, deltay, stepspeed);
  motor_steps(deltax, deltay, stepspeed);

  motor_wait_idle();
}

void show_status()
{
  unsigned int maxx, maxy;
  struct motor_message steps;

  motor_get_maxsteps(&maxx, &maxy);
  printf("Max X Steps %d.\n", maxx);
  printf("Max Y Steps %d.\n", maxy);

  motor_status_get(&steps);
  printf("Status Move: %d.\n", steps.status);
  printf("X Steps %d.\n", steps.x);
  printf("Y Steps %d.\n", steps.y);
  printf("Speed %d.\n", steps.speed);
}

void JSON_status()
{
  // return xpos,ypos and status in JSON string
  // allows passing straight back to async call from ptzclient.cgi
  // with little effort and ability to track x,y position
  struct motor_message steps;

  motor_status_get(&steps);
  printf("{");
  printf("\"status\":\"%d\"", steps.status);
  printf(",");
  printf("\"xpos\":\"%d\"", steps.x);
  printf(",");
  printf("\"ypos\":\"%d\"", steps.y);
  printf(",");
  printf("\"speed\":\"%d\"", steps.speed);
  printf("}");
}

void JSON_initial()
{
  // return all known parameters in JSON string
  // idea is when client page loads in browser we
  // get current details from camera
  struct motor_message steps;
  unsigned int maxx, maxy;

  motor_status_get(&steps);
  printf("{");
  printf("\"status\":\"%d\"", steps.status);
  printf(",");
  printf("\"xpos\":\"%d\"", steps.x);
  printf(",");
  printf("\"ypos\":\"%d\"", steps.y);

  motor_get_maxsteps(&maxx, &maxy);
  printf(",");
  printf("\"xmax\":\"%d\"", maxx);
  printf(",");
  printf("\"ymax\":\"%d\"", maxy);
  printf(",");
  printf("\"speed\":\"%d\"", steps.speed);

  printf("}");
}

int main(int argc, char *argv[])
{
  char direction = 's';
  int stepspeed = 900;
  int xpos = 0;
  int ypos = 0;
  int got_x = 0;
  int got_y = 0;
  int c;
  struct motor_message pos;

  fd = open("/dev/motor", 0); // T31 sources don't take into account the open mode

  while ((c = getopt(argc, argv, "d:s:x:y:jiSr")) != -1)
  {
    switch (c)
    {
    case 'd':
      direction = optarg[0];
      break;
    case 's':
      if (atoi(optarg) > 900)
      {
        stepspeed = 900;
      }
      else
      {
        stepspeed = atoi(optarg);
      }

      break;
    case 'x':
      xpos = atoi(optarg);
      got_x = 1;
      break;
    case 'y':
      ypos = atoi(optarg);
      got_y = 1;
      break;
    case 'j':
      // get x and y current positions and status
      JSON_status();
      exit(EXIT_SUCCESS);
      break;
    case 'i':
      // get all initial values
      JSON_initial();
      exit(EXIT_SUCCESS);
      break;
    case 'r': // reset
      printf(" == Reset position, please wait\n");
      struct motor_reset_data motor_reset_data;
      memset(&motor_reset_data, 0, sizeof(motor_reset_data));
      ioctl(fd, MOTOR_RESET, &motor_reset_data);
      break;
    case 'S': // status
      show_status();
      break;
    default:
      printf("Invalid Argument %c\n", c);
      printf("Usage : %s\n"
             "\t -d Direction step\n"
             "\t -s Speed step (default 900)\n"
             "\t -x X position/step (default 0)\n"
             "\t -y Y position/step (default 0) .\n"
             "\t -r reset to default pos.\n"
             "\t -j return json string xpos,ypos,status.\n"
             "\t -i return json string for all camera parameters\n"
             "\t -S show status\n",
             argv[0]);
      exit(EXIT_FAILURE);
    }
  }

  switch (direction)
  {
  case 's': // stop
    motor_ioctl(MOTOR_STOP, NULL);
    break;

  case 'c': // cruise
    motor_ioctl(MOTOR_CRUISE, NULL);
    motor_wait_idle();
    break;

  case 'b': // go back
    motor_status_get(&pos);
    printf("Going from X %d, Y %d...\n", pos.x, pos.y);
    motor_ioctl(MOTOR_GOBACK, NULL);
    motor_wait_idle();
    motor_status_get(&pos);
    printf("To X %d, Y %d...\n", pos.x, pos.y);

    break;

  case 'h': // set position
    motor_status_get(&pos);
    if (got_x == 0)
      xpos = pos.x;
    if (got_y == 0)
      ypos = pos.y;
    motor_set_position(xpos, ypos, stepspeed);
    break;

  case 'g': // x y steps
    motor_steps(xpos, ypos, stepspeed);
    break;

  default:
    printf("Invalid Direction Argument %c\n", direction);
    printf("Usage : %s -d\n"
           "\t s (Stop)\n"
           "\t c (Cruise)\n"
           "\t b (Go to previous position)\n"
           "\t h (Set position X and Y)\n"
           "\t g (Steps X and Y)\n",
           argv[0]);
    exit(EXIT_FAILURE);
  }
  return 0;
}
