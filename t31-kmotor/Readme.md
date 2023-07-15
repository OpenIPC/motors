
## Load kernel module before use
t31-kmotor is a command line tool to be able to send commands to the sample_motor.ko camera module. By default this module is not loaded and so it is necessary to enter the following commands:

```
modprobe sample_pwm_core
modprobe sample_pwm_hal
modprobe sample_motor hmaxstep=2130 vmaxstep=1600
```

Note that the maximum steps for the horizontal and vertical motors are passed as arguments when inserting the `sample_motor` module.

### Examples for Wyze Pan Cam v3

1) connect to the camera via ssh
```
ssh root@ip.of.your.camera
```
2) load the kernel module:

check if the sample_pwm_core / sample_pwm_hal / sample_motor module are loaded:
```
lsmod
```
if any of them are on the list then unload them first:
```
rmmod sample_pwm_core; rmmod sample_pwm_hal; rmmod sample_motor
```
load the modules with parameters (you may need to experiment with the hmaxstep and vmaxstep values for your specific camera):

```
insmod /path/to/sample_pwm_core.ko
insmod /path/to/sample_pwm_hal.ko
insmod /path/to/sample_motor.ko hmaxstep=2130 vmaxstep=1600
```

3) testing

By passing the -S command line argument, the current status and x,y parameters will be returned:
```
t31-kmotor -S
```
it will look like this:
```
Max X Steps 2130.
Max Y Steps 1600.
Status Move: 0.
X Steps 1065.
Y Steps 800.
Speed 900.
```
Note: Seems like `900` is the maximum speed of the kernel module, hence why it can't be set further than that value.

### Command line options
```
Usage : t31-kmotor
         -d Direction step.
         -s Speed step (default 900).
         -x X position/step (default 0).
         -y Y position/step (default 0).
         -r reset to default pos.
         -j return json string xpos,ypos,status,speed.
         -i return json string for all camera parameters
         -S show status
```          

## Examples

* go to mid position of X and Y (assuming max X steps 2130 and max y steps 1600):
```
t31-kmotor -d t -x 1065 -y 800
```
* go to position of begining of X and Y
```
t31-kmotor -d h -x 0 -y 0 
```
* go to x 1065 and y 0
```
t31-kmotor -d h -x 1992 -y 0 
```
* get camera details as json string
```
t31-kmotor -i
```
* stop the motors
```
t31-kmotor -d s
```
* reset the motors (to the center)
```
t31-kmotor -r
```
