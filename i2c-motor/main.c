/*
 * I2C Motor driver
 *
 * Copyright 2022 Aleksander Volkov <swit.939@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>

static inline __s32 i2c_smbus_access(int file, char read_write, __u8 command,
                                     int size, union i2c_smbus_data *data)
{
	struct i2c_smbus_ioctl_data args;

	args.read_write = read_write;
	args.command = command;
	args.size = size;
	args.data = data;
	return ioctl(file,I2C_SMBUS,&args);
}


static inline __s32 i2c_smbus_read_byte_data(int file, __u8 command)
{
	union i2c_smbus_data data;
	if (i2c_smbus_access(file,I2C_SMBUS_READ,command,
	                     I2C_SMBUS_BYTE_DATA,&data))
		return -1;
	else
		return 0x0FF & data.byte;
}


static inline __s32 i2c_smbus_write_byte_data(int file, __u8 command, __u8 value)
{
	union i2c_smbus_data data;
	data.byte = value;
	return i2c_smbus_access(file, I2C_SMBUS_WRITE, command,
				I2C_SMBUS_BYTE_DATA, &data);
}


void initMotor()
{
uint8_t data, addr = 0x10;
const char *path = "/dev/i2c-2";
int fd, rc;

  int v3; // r4
  int v4; // r0
  int v5; // r8
  int v7; // r1
  int v8[10]; // [sp+0h] [bp-28h] BYREF

  v3 = 3;
  while ( 1 )
  {
    usleep(0x7A120u);
    v4 = open("/dev/pwm", 0);
    v5 = v4;
    if ( v4 >= 0 )
      break;
    if ( !--v3 )
    {
      puts("open pwm fails. ");
    }
  }
  v8[0] = 0;
  v8[1] = 1;
  v8[3] = 1;
  v8[2] = 2;
  v7 = ioctl(v4, 1u, v8);
  if ( v7 )
    printf("HI_PWM_Ctrl: PWM_CMD_WRITE failed: %d\n", v7);
  close(v5);

fd = open(path, O_RDWR);
rc = ioctl(fd, I2C_SLAVE, addr);

data = i2c_smbus_write_byte_data(fd, 0x00, 0x01);
data = i2c_smbus_write_byte_data(fd, 0x01, 0x52);
data = i2c_smbus_write_byte_data(fd, 0x02, 0x80);
data = i2c_smbus_write_byte_data(fd, 0x03, 0x08);
data = i2c_smbus_write_byte_data(fd, 0x04, 0x00);
data = i2c_smbus_write_byte_data(fd, 0x05, 0x52);
data = i2c_smbus_write_byte_data(fd, 0x06, 0x80);
data = i2c_smbus_write_byte_data(fd, 0x07, 0x08);
data = i2c_smbus_write_byte_data(fd, 0x08, 0x00);
data = i2c_smbus_write_byte_data(fd, 0x09, 0x0f);
data = i2c_smbus_write_byte_data(fd, 0x0a, 0x08);

close(fd);
}


void sendCommand(int cmd, int sspeed)
{
        uint8_t data, addr = 0x10, reg = 0x00, value = 0x00;
        const char *path = "/dev/i2c-2";
        int fd,fp, rc,rp,sa,sb,s;

s = sspeed*20;
sb = s / 256;
sa = s - sb * 256;

fd = open(path, O_RDWR);
rc = ioctl(fd, I2C_SLAVE, addr);
        switch (cmd) {
        case 0:                 //stop
                data = i2c_smbus_write_byte_data(fd, 0x09, 0x07);
                data = i2c_smbus_write_byte_data(fd, 0x00, 0x07);
                data = i2c_smbus_write_byte_data(fd, 0x00, 0x00);
                data = i2c_smbus_write_byte_data(fd, 0x00, 0x01);
            break;
        case 1:                 //Zoom-
		data = i2c_smbus_write_byte_data(fd, 0x03, sa);
		data = i2c_smbus_write_byte_data(fd, 0x04, 0xc0 + sb);
		data = i2c_smbus_write_byte_data(fd, 0x09, 0xcf);
            break;
        case 2:                 //Zoom+
                data = i2c_smbus_write_byte_data(fd, 0x03, sa);
                data = i2c_smbus_write_byte_data(fd, 0x04, 0xb0 + sb);
                data = i2c_smbus_write_byte_data(fd, 0x09, 0xcf);
            break;
        case 3:                 //Focus-
                data = i2c_smbus_write_byte_data(fd, 0x07, sa);
                data = i2c_smbus_write_byte_data(fd, 0x08, 0xc0 + sb);
                data = i2c_smbus_write_byte_data(fd, 0x09, 0xcf);
            break;
        case 4:                 //Focus+
                data = i2c_smbus_write_byte_data(fd, 0x07, sa);
                data = i2c_smbus_write_byte_data(fd, 0x08, 0xb0 + sb);
                data = i2c_smbus_write_byte_data(fd, 0x09, 0xcf);
            break;
    }
close(fd);
}

int main(int argc, char **argv)
{
	uint8_t data, addr = 0x10, reg = 0x00, value = 0x00;
	const char *path = "/dev/i2c-2";
	int file, rc;
	int steps[5];
	char direction = 's';
        int stepspeed = 5;
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
            sendCommand(1, stepspeed);
            break;
        case 'd':			//Zoom-
            sendCommand(2, stepspeed);
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
        case 'i':                       //stop
            initMotor();
            break;
        default:
            printf("Invalid Direction Argument %c\n", direction);
			printf("Usage : %s -d\n"
                        "\t i (Init)\n"
			"\t u (Zoom+)\n"
			"\t d (Zoom-)\n"
			"\t l (Focus-)\n"
			"\t r (Focus+)\n"
			"\t s (Stop)\n", argv[0]);
            exit(EXIT_FAILURE);
    }
	return 0;
}
