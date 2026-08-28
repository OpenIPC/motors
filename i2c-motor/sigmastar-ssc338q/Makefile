CC ?= $(CROSS_COMPILE)gcc
CFLAGS ?= -O2 -Wall
LDFLAGS ?= -lm

TARGET = motor

all: $(TARGET)

$(TARGET): main.o
	$(CC) -o $@ $^ $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(TARGET) *.o

.PHONY: all clean
