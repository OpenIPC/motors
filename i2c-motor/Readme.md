## Experimental usage for MS32006 motors driver

### Basic init
```
devmem 0x112C0058 32 0x1032         # Set I2C2_SCL, connected to GPIO7_0 (gpio56)
devmem 0x112C005C 32 0x1032         # Set I2C2_SDA, connected to GPIO7_1 (gpio57)
devmem 0x100C000C 32 0x1
devmem 0x120101BC 32 0x801282
```

### General commands
```
-d Direction step ( i (Init),  u (Zoom+),  d (Zoom-),  l (Focus-),  r (Focus+),  s (Stop))
-s Speed step (default 5)
```

### Example of use
```
i2c-motor -d i           # init motors
i2c-motor -d r -s 50
i2c-motor -d l -s 50
i2c-motor -d d -s 50
i2c-motor -d u -s 50
i2c-motor -d s           # stop motors
```


