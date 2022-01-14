#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

#define MOTOR_STATUS   		0xC0046D02
#define MOTOR_CMD   		0xC0046D01



void motor_ioctl(int cmd, int *arg) {
    int fd = open("/dev/motor", O_WRONLY);
    ioctl(fd, cmd, arg);
    close(fd);
}

void motor_status_get(int *cmd) {
	motor_ioctl(MOTOR_STATUS, cmd);
}

void sendCommand(int cmd, int sspeed) {
	int s[3];	
	s[0] = cmd;
    s[1] = sspeed; //xspeed
	s[2] = sspeed; //yspeed
	motor_ioctl(MOTOR_CMD, s);	
	
//	show_sage();
}

int main (int argc, char *argv[]) {
	int steps[5];
	char direction = 's';
    int stepspeed = 20;
	int xpos = 0;
	int ypos = 0;
	int c;
	//sendCommand(0, stepspeed);	//Stop
	//sendCommand(1, stepspeed);	//Zoom-
	//sendCommand(2, stepspeed);	//Zoom+	
	//sendCommand(3, stepspeed);	//Focus+
	//sendCommand(4, stepspeed);	//Focus-
	
	while ((c = getopt(argc, argv, "d:s:")) != -1) {
        switch (c) {
            case 'd':
                direction = optarg[0];
                break;
            case 's':
                if (atoi(optarg) > 100)
				{
					stepspeed = 10;
				 }else {
					stepspeed =atoi(optarg);
				}
					
                break;
            default:
                printf("Invalid Argument %c\n", c);
				printf("Usage : %s\n"				
				"\t -d Direction step\n"
				"\t -s Speed step (default 5)\n" , argv[0]);
                exit(EXIT_FAILURE);
        }
    }
	
	switch (direction) {
        case 'u':			//Zoom+
            sendCommand(2, stepspeed);
            break;
        case 'd':			//Zoom-
            sendCommand(1, stepspeed);
            break;
        case 'l':			//Focus-
            sendCommand(4, stepspeed);
            break;
        case 'r':			//Focus+
            sendCommand(3, stepspeed);
            break;
        case 's':			//stop
            sendCommand(0, stepspeed);
            break;

        default:
            printf("Invalid Direction Argument %c\n", direction);
			printf("Usage : %s -d\n" 
			"\t u (Zoom+)\n"
			"\t d (Zoom-)\n" 
			"\t l (Focus-)\n"
			"\t r (Focus+)\n"
			"\t s (Stop)\n", argv[0]);
            exit(EXIT_FAILURE);
    }
/* 	sendCommand(3, stepspeed);
	
	motor_status_get(steps);
	printf("Status Move: %d.\n", steps[0]);
	printf("X Steps %d.\n", steps[1]);
	printf("Y Steps %d.\n", steps[2]); */
	return 0;
}
