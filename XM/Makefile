#CC=arm-hisiv510-linux-gcc
CC=arm-openipc-linux-musleabi-gcc

xm-motors: main.o
	$(CC) -o $@ $^
	sudo cp $@ /mnt/noc/sdk

clean:
	-rm -rf xm-motors *.o
