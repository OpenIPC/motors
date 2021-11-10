## Example usage

```
insmod /lib/modules/4.9.37/hisilicon/camhi-motor.ko
```

```
-d Direction step (  u (Zoom+),  d (Zoom-),  l (Focus-),  r (Focus+),  s (Stop))
-s Speed step (default 5)
```

```
camhi-motor -d r -s 50
camhi-motor -d l -s 50
camhi-motor -d d -s 50
camhi-motor -d u -s 50
```


