all: anjoy-motor-openipc

anjoy-motor-openipc: CC=arm-openipc-linux-musleabi-gcc

anjoy-motor-%: main.o
	$(CC) -static -o $@ $^

clean:
	-rm -rf anjoy-motor-* *.o
