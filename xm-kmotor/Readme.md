
## Load kernel module before usage


```
insmod kmotor.ko gpio_pin=3,70,75,77,74,76,69,71,-1,-1,-1,-1 auto_test=0
```
auto_test=0   don't make autotest
auto_test=1   make autotest

if you make autotest camera will rotate to find min/max value of coordinates. after autotest camera will NOT return to position before. 

-----

### ХМ 00030695, xm-pt817-20w (tested)

```
insmod kmotor.ko gpio_pin=3,70,75,77,74,76,69,71,-1,-1,-1,-1 auto_test=1 MAX_DEGREE_X=350 MAX_DEGREE_Y=125
```

-----

### Examples for xm-pt817-20w

1) connect co camera via ssh
```
ssh root@ip.of.your.camera
```
2) load kernel module:

check if module kmotor is loaded:
```
lsmod
```
if kmotor in list it need to be unload:
```
rmmod kmotor
```
loading kmotor module with parameters:

```
insmod /lib/modules/3.10.103\+/xiongmai/kmotor.ko gpio_pin=3,70,75,77,74,76,69,71,-1,-1,-1,-1 auto_test=1 MAX_DEGREE_X=350 MAX_DEGREE_Y=125
```

3) test of motors

make info:
```
xm-kmotor
```
it will list like:
```
Max X Steps 3982.       (value of X cordinate for camera from 0 to 3982)
Max Y Steps 1422.       (value of Y cordinate for camera from 0 to 1422)
Status Move: 65535.
X Steps 1991.           (current of X cordinate)
Y Steps 0.              (current of Y cordinate)
```

go to mid position of X and Y:
```
xm-kmotor -d t -x 1992 -y 712   ("direction go to x=1992 y=712")
```
go to position of begining of X and Y
```
xm-kmotor -d t -x 0 -y 0 
```
go to example position
```
xm-kmotor -d t -x 1992 -y 0 
```
