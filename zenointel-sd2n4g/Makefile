# Zenointel SD-2N-4G (Goke GK7205V510) pan/tilt stepper tool. musl / OpenIPC.
all: sd2n4g-motor-openipc

sd2n4g-motor-openipc: CC=arm-openipc-linux-musleabi-gcc

sd2n4g-motor-%: main.o
	$(CC) -static -o $@ $^

clean:
	-rm -rf sd2n4g-motor-* *.o
