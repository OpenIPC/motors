
## Load kernel module before usage


```
insmod kmotor.ko gpio_pin=3,70,75,77,74,76,69,71,-1,-1,-1,-1 auto_test=0
```

-----

### ХМ 00030695, xm-pt817-20w (tested)

```
insmod kmotor.ko gpio_pin=3,70,75,77,74,76,69,71,-1,-1,-1,-1 auto_test=1 MAX_DEGREE_X=350 MAX_DEGREE_Y=125
```
