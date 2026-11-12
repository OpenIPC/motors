# Xiongmai HI3516D_N81820 (HiSilicon Hi3516A V100) PTZ-port bit-bang sender.
# Self-contained: PL061 GPIO + IOCONFIG via /dev/mem, no SDK. musl / OpenIPC.
all: pelco-bb-openipc

pelco-bb-openipc: CC=arm-openipc-linux-musleabi-gcc

pelco-bb-%: main.c
	$(CC) -O2 -Wall -static -o $@ $^

clean:
	-rm -f pelco-bb-*
