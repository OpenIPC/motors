#!/bin/sh
set -eu

host=${CAMERA_HOST:-192.168.1.199}
user=${CAMERA_USER:-root}
build=${CAMERA_BUILD:-build-camera}
camera_cc=${CAMERA_CC:-arm-openipc-linux-musleabi-gcc}
target="$user@$host"

if ! command -v "$camera_cc" >/dev/null 2>&1; then
	echo "camera compiler not found: $camera_cc" >&2
	echo "set CAMERA_CC to the OpenIPC cross-compiler path" >&2
	exit 1
fi

sysroot=${CAMERA_SYSROOT:-$("$camera_cc" -print-sysroot)}
terminfo_dir=${CAMERA_TERMINFO_DIR:-$sysroot/usr/share/terminfo}
ncurses_lib=${CAMERA_NCURSES_LIB:-}
if [ -z "$ncurses_lib" ]; then
	for candidate in "$sysroot"/usr/lib/libncurses.so.6.* "$sysroot"/usr/lib/libncurses.so.6; do
		if [ -f "$candidate" ]; then
			ncurses_lib=$candidate
			break
		fi
	done
fi

if [ ! -x "$build/pelcodtui" ]; then
	echo "missing camera binary: $build/pelcodtui" >&2
	exit 1
fi

if [ ! -f "$ncurses_lib" ]; then
	echo "missing camera ncurses library under: $sysroot/usr/lib" >&2
	echo "set CAMERA_NCURSES_LIB to override it" >&2
	exit 1
fi

for entry in x/xterm-256color x/xterm v/vt100; do
	if [ ! -f "$terminfo_dir/$entry" ]; then
		echo "missing camera terminfo entry: $terminfo_dir/$entry" >&2
		exit 1
	fi
done

if [ -n "${CAMERA_PASSWORD:-}" ]; then
	command -v sshpass >/dev/null 2>&1 || {
		echo "CAMERA_PASSWORD requires sshpass" >&2
		exit 1
	}
fi

run_ssh() {
	if [ "${CAMERA_INSECURE_SSH:-0}" = 1 ]; then
		set -- -o StrictHostKeyChecking=no "$@"
	fi
	if [ -n "${CAMERA_PASSWORD:-}" ]; then
		sshpass -p "$CAMERA_PASSWORD" ssh "$@"
	else
		ssh "$@"
	fi
}

run_scp() {
	if [ "${CAMERA_INSECURE_SSH:-0}" = 1 ]; then
		set -- -o StrictHostKeyChecking=no "$@"
	fi
	if [ -n "${CAMERA_PASSWORD:-}" ]; then
		sshpass -p "$CAMERA_PASSWORD" scp -O "$@"
	else
		scp -O "$@"
	fi
}

run_ssh "$target" \
	'rm -rf /tmp/pelcodtui-cameras.new /tmp/pelcodtui-runtime /tmp/pelcodtui-backup && mkdir -p /etc/pelcodtui /tmp/pelcodtui-cameras.new /tmp/pelcodtui-runtime/terminfo/x /tmp/pelcodtui-runtime/terminfo/v && rm -f /tmp/pelcodtui.new /tmp/pelcodtui-launcher.new'
run_scp "$build/pelcodtui" "$target:/tmp/pelcodtui.new"
run_scp "$ncurses_lib" "$target:/tmp/pelcodtui-runtime/libncurses.so.6.4"
run_scp "$terminfo_dir/x/xterm-256color" "$target:/tmp/pelcodtui-runtime/terminfo/x/xterm-256color"
run_scp "$terminfo_dir/x/xterm" "$target:/tmp/pelcodtui-runtime/terminfo/x/xterm"
run_scp "$terminfo_dir/v/vt100" "$target:/tmp/pelcodtui-runtime/terminfo/v/vt100"
run_scp cameras/*.conf "$target:/tmp/pelcodtui-cameras.new/"
run_scp scripts/pelcodtui-camera "$target:/tmp/pelcodtui-launcher.new"
run_ssh "$target" '
	chmod 0755 /tmp/pelcodtui.new &&
	ln -sf libncurses.so.6.4 /tmp/pelcodtui-runtime/libncurses.so.6 &&
	for profile in /tmp/pelcodtui-cameras.new/*.conf; do
		LD_LIBRARY_PATH=/tmp/pelcodtui-runtime /tmp/pelcodtui.new --validate "$profile" || exit 1
	done || exit 1
	mkdir -p /tmp/pelcodtui-backup
	[ ! -e /usr/lib/pelcodtui ] || cp -a /usr/lib/pelcodtui /tmp/pelcodtui-backup/runtime
	[ ! -e /usr/bin/pelcodtui ] || cp -a /usr/bin/pelcodtui /tmp/pelcodtui-backup/launcher
	[ ! -e /etc/pelcodtui/cameras ] || cp -a /etc/pelcodtui/cameras /tmp/pelcodtui-backup/cameras
	install_new() {
		rm -rf /usr/lib/pelcodtui || return 1
		mkdir -p /usr/lib/pelcodtui || return 1
		mv /tmp/pelcodtui.new /usr/lib/pelcodtui/pelcodtui || return 1
		mv /tmp/pelcodtui-runtime/libncurses.so.6.4 /usr/lib/pelcodtui/libncurses.so.6.4 || return 1
		mv /tmp/pelcodtui-runtime/terminfo /usr/lib/pelcodtui/terminfo || return 1
		ln -sf libncurses.so.6.4 /usr/lib/pelcodtui/libncurses.so.6 || return 1
		mv /tmp/pelcodtui-launcher.new /usr/bin/pelcodtui || return 1
		chmod 0755 /usr/lib/pelcodtui/pelcodtui /usr/bin/pelcodtui || return 1
		rm -rf /etc/pelcodtui/cameras || return 1
		mv /tmp/pelcodtui-cameras.new /etc/pelcodtui/cameras || return 1
		chmod 0644 /etc/pelcodtui/cameras/*.conf || return 1
		for profile in /etc/pelcodtui/cameras/*.conf; do /usr/bin/pelcodtui --validate "$profile" || return 1; done
	}
	if ! install_new; then
		rm -rf /usr/lib/pelcodtui /usr/bin/pelcodtui /etc/pelcodtui/cameras
		[ ! -e /tmp/pelcodtui-backup/runtime ] || mv /tmp/pelcodtui-backup/runtime /usr/lib/pelcodtui
		[ ! -e /tmp/pelcodtui-backup/launcher ] || mv /tmp/pelcodtui-backup/launcher /usr/bin/pelcodtui
		[ ! -e /tmp/pelcodtui-backup/cameras ] || mv /tmp/pelcodtui-backup/cameras /etc/pelcodtui/cameras
		echo "deployment failed; previous installation restored" >&2
		exit 1
	fi
	rm -rf /tmp/pelcodtui-backup /tmp/pelcodtui-cameras.new /tmp/pelcodtui-runtime
'

echo "deployed pelcodtui to $target"
echo "saved state preserved at /etc/pelcodtui/state.conf"
