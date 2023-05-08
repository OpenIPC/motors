
## Load kernel module before use
xm-kmotor is a command line tool to be able to send commands to the kmotor.ko camera module. By default this module is not loaded and so it is necessary to enter the following command.   

```
insmod kmotor.ko gpio_pin=3,70,75,77,74,76,69,71,-1,-1,-1,-1 auto_test=0
```
The auto_test option can be set to 0 or 1 where auto_test=1 will get the camera to do a startup test and rotate along both the x and y axis to get the min/max x and y parameters. Setting auto_test=0 will not. 

After the autotest has completed be aware the camera will NOT return to its original position. 

-----

### ХМ 00030695, xm-pt817-20w (tested)

```
insmod kmotor.ko gpio_pin=3,70,75,77,74,76,69,71,-1,-1,-1,-1 auto_test=1 MAX_DEGREE_X=350 MAX_DEGREE_Y=125
```

-----

### Examples for xm-pt817-20w

1) connect to the camera via ssh
```
ssh root@ip.of.your.camera
```
2) load the kernel module:

check if the kmotor module is loaded:
```
lsmod
```
if kmotor is in the list then unload it first:
```
rmmod kmotor
```
load the kmotor module with parameters ( you may need to experiment with the MAX X and MAX Y values for your specific camera):

```
insmod /lib/modules/3.10.103\+/xiongmai/kmotor.ko gpio_pin=3,70,75,77,74,76,69,71,-1,-1,-1,-1 auto_test=1 MAX_DEGREE_X=350 MAX_DEGREE_Y=125
```

3) testing

By entering no command line arguments the current status and x,y parameters will be returned:
```
xm-kmotor
```
it will look like this:
```
Max X Steps 3982.       (value of X cordinate for camera from 0 to 3982)
Max Y Steps 1422.       (value of Y cordinate for camera from 0 to 1422)
Status Move: 65535.     (65535 means idle)
X Steps 1991.           (current x position )
Y Steps 0.              (current Y position)
```

### Command line options
```
Usage : xm-kmotor -d Direction to step 
                      u (Up)
                      d (Down)
                      l (Left)
                      r (Right)
                      e (Right Up)
                      c (Right Down)
                      q (Left Up)
                      z (Left Down)
                      s (Stop)
                      h (Set position X and Y)
                      t (Go to X and Y)
                      f (Scan, Y to set)
                      g (Steps X and Y)             
                   -s Speed step (1-10, default 5)
                   -x X position/step (default 0)
                   -y Y position/step (default 0)
                   
                   -j get camera status as a json formatted string (currentX,currentY & status)
                   -i get initial camera details as json string (maxX, maxY, status, currentX, currentY)
```          

examples:

go to mid position of X and Y:
```
xm-kmotor -d t -x 1992 -y 712   ("direction go to x=1992 y=712")
```
go to position of begining of X and Y
```
xm-kmotor -d t -x 0 -y 0 
```
go to x 1992 and y 0
```
xm-kmotor -d t -x 1992 -y 0 
```
get camera details as json string
```
xm-kmotor -i
```
scan up & stop
```
xm-kmotor -d u
xm-kmotor -d s
```

