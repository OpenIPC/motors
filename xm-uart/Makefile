all: xm-uart-motors-orig xm-uart-motors-openipc

xm-uart-motors-openipc: CC=arm-openipc-linux-musleabi-gcc

xm-uart-motors-orig: CC=arm-hisiv510-linux-gcc

xm-%: main.o
	$(CC) -o $@ $^

clean:
	-rm -rf xm-uart-motors-* *.o
