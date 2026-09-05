CC ?= cc
CFLAGS ?= -O2 -Wall -Wextra -Werror
CPPFLAGS += -D_DEFAULT_SOURCE -D_POSIX_C_SOURCE=200809L -Isrc
NCURSES_LIBS ?= -lncurses
PREFIX ?= /usr
SYSCONFDIR ?= /etc
BUILD := build
CORE := $(BUILD)/profile.o $(BUILD)/pelco.o $(BUILD)/state.o
UI := $(BUILD)/tui.o $(BUILD)/tui_menus.o
CAMERA_BUILD := build-camera
SIBLING_CAMERA_CC := ../../firmware/output/host/bin/arm-openipc-linux-musleabi-gcc
CAMERA_CC ?= $(if $(wildcard $(SIBLING_CAMERA_CC)),$(SIBLING_CAMERA_CC),arm-openipc-linux-musleabi-gcc)
CAMERA_HOST ?= 192.168.1.199
CAMERA_USER ?= root

.PHONY: all clean test install camera deploy
all: $(BUILD)/pelcodtui

$(BUILD):
	mkdir -p $@

$(BUILD)/%.o: src/%.c src/pelcodtui.h | $(BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

$(BUILD)/main.o $(UI): src/tui_internal.h

$(BUILD)/pelcodtui: $(BUILD)/main.o $(UI) $(CORE)
	$(CC) $(LDFLAGS) $^ $(NCURSES_LIBS) -o $@

$(BUILD)/test_core: tests/test_core.c $(CORE)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) $^ -lutil -o $@

test: $(BUILD)/pelcodtui $(BUILD)/test_core
	@for profile in cameras/*.conf; do \
		$(BUILD)/pelcodtui --validate "$$profile" | grep -qx 'profile valid' || exit 1; \
		$(BUILD)/test_core "$$profile" || exit 1; \
	done
	! grep -Eq 'preset_(set|call) (88|95|96|97|98|150|151|153|158|159)(;|$$)' cameras/*.conf

install: $(BUILD)/pelcodtui
	install -D -m 0755 $< $(DESTDIR)$(PREFIX)/bin/pelcodtui
	install -d $(DESTDIR)$(SYSCONFDIR)/pelcodtui/cameras
	install -m 0644 cameras/*.conf $(DESTDIR)$(SYSCONFDIR)/pelcodtui/cameras/

clean:
	rm -rf $(BUILD)

camera:
	$(MAKE) BUILD=$(CAMERA_BUILD) CC=$(CAMERA_CC) all

deploy: camera
	CAMERA_HOST='$(CAMERA_HOST)' CAMERA_USER='$(CAMERA_USER)' \
		CAMERA_PASSWORD='$(CAMERA_PASSWORD)' CAMERA_BUILD='$(CAMERA_BUILD)' \
		CAMERA_CC='$(CAMERA_CC)' CAMERA_INSECURE_SSH='$(CAMERA_INSECURE_SSH)' \
		./scripts/deploy-camera.sh
