all: xm-uart-motors-orig xm-uart-motors-openipc

xm-uart-motors-openipc: CC=arm-openipc-linux-musleabi-gcc

xm-uart-motors-orig: CC=arm-hisiv510-linux-gcc

# Native build, for running under uart-bridge on a lab host (see PROTOCOL.md).
xm-uart-motors-host: CC=cc

CFLAGS ?= -Wall -O2

# Each variant compiles main.c itself. A shared main.o would be built once
# with whichever variant's CC came first and then linked into all of them.
xm-uart-motors-%: main.c
	$(CC) $(CFLAGS) -o $@ $<

clean:
	-rm -rf xm-uart-motors-* *.o

.PHONY: all clean
