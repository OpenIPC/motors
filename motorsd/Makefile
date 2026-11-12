CC ?= cc
JSON_CFLAGS ?= $(shell pkg-config --cflags json-c 2>/dev/null || echo "-I../../json-c/include -I../../json-c/build")
CPPFLAGS += -Iinclude $(JSON_CFLAGS)
CFLAGS ?= -O2 -g
CFLAGS += -std=c11 -Wall -Wextra -Werror
LDLIBS_JSON ?= $(shell pkg-config --libs json-c 2>/dev/null || echo "-L../../json-c/build -ljson-c")

BUILD ?= build
CAMERA_BUILD := build-camera
SIBLING_CAMERA_CC := ../../firmware/output/host/bin/arm-openipc-linux-musleabi-gcc
CAMERA_CC ?= $(if $(wildcard $(SIBLING_CAMERA_CC)),$(SIBLING_CAMERA_CC),arm-openipc-linux-musleabi-gcc)
PROGRAMS := $(BUILD)/motorsd $(BUILD)/motorsctl $(BUILD)/motors-cli \
	$(BUILD)/mock_driver \
	$(BUILD)/pelcod_driver $(BUILD)/anjoy_driver \
	$(BUILD)/libmotors.a $(BUILD)/test_libmotors \
	$(BUILD)/test_libmotors_preemption

.PHONY: all clean test camera

all: $(PROGRAMS)

$(BUILD):
	mkdir -p $@

$(BUILD)/motorsd: src/motorsd.c include/motors_driver.h | $(BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ src/motorsd.c $(LDLIBS_JSON)

$(BUILD)/motorsctl: src/motorsctl.c | $(BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ src/motorsctl.c

$(BUILD)/motors-cli: src/motors_cli.c $(BUILD)/libmotors.a | $(BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ src/motors_cli.c \
		$(BUILD)/libmotors.a $(LDLIBS_JSON)

$(BUILD)/libmotors.o: src/libmotors.c include/libmotors.h | $(BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c -o $@ src/libmotors.c

$(BUILD)/libmotors.a: $(BUILD)/libmotors.o
	$(AR) rcs $@ $<

$(BUILD)/test_libmotors: tests/libmotors_client.c $(BUILD)/libmotors.a | $(BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ tests/libmotors_client.c \
		$(BUILD)/libmotors.a $(LDLIBS_JSON)

$(BUILD)/test_libmotors_preemption: tests/libmotors_preemption.c \
		$(BUILD)/libmotors.a | $(BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $< \
		$(BUILD)/libmotors.a $(LDLIBS_JSON)

$(BUILD)/mock_driver: drivers/mock_driver.c include/motors_driver.h | $(BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ drivers/mock_driver.c $(LDLIBS_JSON)

$(BUILD)/pelcod_driver: drivers/pelcod_driver.c include/motors_driver.h | $(BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ drivers/pelcod_driver.c $(LDLIBS_JSON)

$(BUILD)/anjoy_driver: drivers/anjoy_driver.c include/motors_driver.h | $(BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ drivers/anjoy_driver.c $(LDLIBS_JSON) -lm

test: all
	python3 tests/test_architecture.py

camera:
	$(MAKE) BUILD=$(CAMERA_BUILD) CC=$(CAMERA_CC) JSON_CFLAGS= all

clean:
	rm -rf $(BUILD)
