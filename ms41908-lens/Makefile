# Xiongmai HI3516D_N81820 (HiSilicon Hi3516A V100) MS41908M lens tool.
# Self-contained: SPI over /dev/spidev1.0 + PL061 GPIO via /dev/mem, no SDK. musl / OpenIPC.
all: ms41908-lens-openipc

ms41908-lens-openipc: CC=arm-openipc-linux-musleabi-gcc

ms41908-lens-%: main.o
	$(CC) -static -o $@ $^

clean:
	-rm -rf ms41908-lens-* *.o
