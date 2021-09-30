all: xm-kmotor-orig xm-kmotor-openipc

xm-kmotor-openipc: CC=arm-openipc-linux-musleabi-gcc

xm-kmotor-orig: CC=arm-xm-linux-gcc

xm-%: main.o
	$(CC) -o $@ $^

clean:
	-rm -rf xm-kmotor-* *.o
