#define _GNU_SOURCE

#include "motors_driver.h"

#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <json-c/json.h>
#include <math.h>
#include <poll.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#define MESSAGE_MAX 32768
#define RX_BUFFER_MAX 1024

/* Anjoy axes bitmask: Pan(1), Tilt(2), Zoom(4), Focus(8), Iris(16) = 0x1f */
#define ANJOY_ALL_AXES 0x1f

struct configuration {
    char device[128];
    unsigned baud;
    unsigned pan_speed;
    unsigned tilt_speed;
    unsigned keepalive_ms;
};

struct driver_state {
    int control_fd;
    int uart_fd;
    struct configuration config;
    uint32_t active;
    bool zoom_was_active;
    uint64_t deadlines[MOTORS_AXIS_COUNT];
    uint64_t next_keepalive_ms;
    uint64_t cancel_mcu_af_at_ms;
    uint64_t cancel_mcu_af_stop_ms;
    unsigned cancel_mcu_af_passes;
    uint8_t rx_buf[RX_BUFFER_MAX];
    size_t rx_len;
};

static uint64_t now_ms(void) {
    struct timespec value;
    clock_gettime(CLOCK_MONOTONIC, &value);
    return (uint64_t)value.tv_sec * 1000U + (uint64_t)value.tv_nsec / 1000000U;
}

static speed_t baud_value(unsigned baud) {
    switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    default: return 0;
    }
}

static int parse_uint(const char *text, unsigned minimum, unsigned maximum,
                      unsigned *result) {
    char *end = NULL;
    errno = 0;
    unsigned long value = strtoul(text, &end, 10);
    if (errno || !end || *end || value < minimum || value > maximum)
        return -1;
    *result = (unsigned)value;
    return 0;
}

static char *trim(char *text) {
    while (*text && isspace((unsigned char)*text)) ++text;
    char *end = text + strlen(text);
    while (end > text && isspace((unsigned char)end[-1])) --end;
    *end = '\0';
    return text;
}

static int load_configuration(const char *path, struct configuration *config) {
    *config = (struct configuration){
        .baud = 57600,
        .pan_speed = 20,    /* default ~speed 3 in UI (5*3+5 = 20 = 0x14) */
        .tilt_speed = 20,
        .keepalive_ms = 280,
    };
    snprintf(config->device, sizeof(config->device), "/dev/ttyS2");

    FILE *file = fopen(path, "r");
    if (!file) return -1;
    char line[512];
    while (fgets(line, sizeof(line), file)) {
        char *end = strpbrk(line, "\r\n");
        if (end) *end = '\0';
        char *text = trim(line);
        if (!*text || *text == '#') continue;
        char *comment = strchr(text, '#');
        if (comment) *comment = '\0';
        text = trim(text);
        if (!*text) continue;

        char *separator = strchr(text, '=');
        if (!separator) {
            errno = EINVAL;
            goto fail;
        }
        *separator++ = '\0';
        char *key = trim(text);
        char *value = trim(separator);

        if (!strcmp(key, "device")) {
            if (!*value || strlen(value) >= sizeof(config->device)) {
                errno = EINVAL;
                goto fail;
            }
            snprintf(config->device, sizeof(config->device), "%s", value);
        } else if (!strcmp(key, "baud")) {
            if (parse_uint(value, 1, 115200, &config->baud)) goto invalid;
        } else if (!strcmp(key, "pan_speed")) {
            if (parse_uint(value, 1, 63, &config->pan_speed)) goto invalid;
        } else if (!strcmp(key, "tilt_speed")) {
            if (parse_uint(value, 1, 63, &config->tilt_speed)) goto invalid;
        } else if (!strcmp(key, "keepalive_ms")) {
            if (parse_uint(value, 50, 5000, &config->keepalive_ms)) goto invalid;
        }
    }
    fclose(file);
    return 0;

invalid:
    errno = EINVAL;
fail:
    fclose(file);
    return -1;
}

static int open_uart(const struct configuration *config) {
    speed_t speed = baud_value(config->baud);
    if (!speed) {
        errno = EINVAL;
        return -1;
    }
    int fd = open(config->device, O_RDWR | O_NOCTTY | O_CLOEXEC | O_NONBLOCK);
    if (fd < 0) return -1;
    struct termios settings;
    if (tcgetattr(fd, &settings) != 0) goto fail;
    cfmakeraw(&settings);
    cfsetispeed(&settings, speed);
    cfsetospeed(&settings, speed);
    settings.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
    settings.c_cflag |= CLOCAL | CREAD | CS8;
#ifdef CRTSCTS
    settings.c_cflag &= ~CRTSCTS;
#endif
    if (tcsetattr(fd, TCSANOW, &settings) != 0) goto fail;
    return fd;
fail:
    close(fd);
    return -1;
}

static int write_all(int fd, const void *data, size_t length) {
    const uint8_t *bytes = (const uint8_t *)data;
    size_t written = 0;
    while (written < length) {
        ssize_t result = write(fd, bytes + written, length - written);
        if (result < 0) {
            if (errno == EINTR) continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                struct pollfd pfd = {.fd = fd, .events = POLLOUT};
                if (poll(&pfd, 1, 100) > 0) continue;
            }
            return -1;
        }
        written += (size_t)result;
    }
    return 0;
}

/* 16-byte state frame sent every ~280ms to keep Anjoy MCU session alive */
static int send_keepalive(struct driver_state *state) {
    uint8_t frame[16] = {
        0x51, 0x01, 0x04, 0x78, 0x01, 0x0d, 0x59, 0x56,
        0x07, 0x20, 0x2f, 0x00, 0x20, 0x00, 0x00, 0x00
    };
    uint16_t sum = 0;
    for (int i = 0; i < 15; ++i) sum += frame[i];
    frame[15] = (uint8_t)(sum & 0xff);

    state->next_keepalive_ms = now_ms() + state->config.keepalive_ms;
    return write_all(state->uart_fd, frame, sizeof(frame));
}

/* Send 7-byte Pelco-D style command frame */
static int send_cmd_frame(struct driver_state *state, uint8_t c1, uint8_t c2,
                          uint8_t d1, uint8_t d2) {
    uint8_t frame[7];
    frame[0] = 0xff;
    frame[1] = 0x01;
    frame[2] = c1;
    frame[3] = c2;
    frame[4] = d1;
    frame[5] = d2;
    frame[6] = (uint8_t)((0x01 + c1 + c2 + d1 + d2) & 0xff);
    return write_all(state->uart_fd, frame, sizeof(frame));
}

static int send_stop(struct driver_state *state) {
    state->active = 0;
    memset(state->deadlines, 0, sizeof(state->deadlines));
    int rc = send_cmd_frame(state, 0x00, 0x00, 0x00, 0x00);
    if (state->zoom_was_active) {
        state->zoom_was_active = false;
        /* Anjoy MCU enters an internal AF hunting state after zoom stops.
         * Schedule a two-pass focus nudge (near 65ms + stop, then far 65ms + stop)
         * to switch the MCU back to manual focus mode with zero net focus shift,
         * finishing well before majestic-af's 1350ms post-zoom settle window.
         */
        state->cancel_mcu_af_stop_ms = 0;
        state->cancel_mcu_af_passes = 2;
        state->cancel_mcu_af_at_ms = now_ms() + 180;
    }
    return rc;
}

static int send_json(int fd, struct json_object *object) {
    const char *text = json_object_to_json_string_ext(object, JSON_C_TO_STRING_PLAIN);
    return send(fd, text, strlen(text), MSG_NOSIGNAL) < 0 ? -1 : 0;
}

static int send_reply(int fd, const char *id, bool ok, const char *detail) {
    struct json_object *reply = json_object_new_object();
    json_object_object_add(reply, "version",
                           json_object_new_int(MOTORS_DRIVER_PROTOCOL_VERSION));
    json_object_object_add(reply, "id", json_object_new_string(id ? id : ""));
    json_object_object_add(reply, "ok", json_object_new_boolean(ok));
    json_object_object_add(reply, "completed_mono_ms",
                           json_object_new_int64((int64_t)now_ms()));
    if (detail)
        json_object_object_add(reply, ok ? "state" : "error",
                               json_object_new_string(detail));
    int result = send_json(fd, reply);
    json_object_put(reply);
    return result;
}

static int send_movement_event(int fd, uint32_t axes) {
    struct json_object *event = json_object_new_object();
    json_object_object_add(event, "version",
                           json_object_new_int(MOTORS_DRIVER_PROTOCOL_VERSION));
    json_object_object_add(event, "event", json_object_new_string("movement_ended"));
    json_object_object_add(event, "axes", json_object_new_int64(axes));
    json_object_object_add(event, "completed_mono_ms",
                           json_object_new_int64((int64_t)now_ms()));
    int result = send_json(fd, event);
    json_object_put(event);
    return result;
}

static int send_zoom_magnification(const struct driver_state *state, double value) {
    fprintf(stderr, "anjoy_driver: zoom_mag=%.2f\n", value);
    struct json_object *event = json_object_new_object();
    json_object_object_add(event, "version", json_object_new_int(1));
    json_object_object_add(event, "event", json_object_new_string("telemetry"));
    json_object_object_add(event, "name",
                           json_object_new_string("zoom_magnification"));
    json_object_object_add(event, "value", json_object_new_double(value));
    json_object_object_add(event, "observed_mono_ms",
                           json_object_new_int64((int64_t)now_ms()));
    int result = send_json(state->control_fd, event);
    json_object_put(event);
    return result;
}

static const char *json_string(struct json_object *object, const char *key) {
    struct json_object *value = NULL;
    if (!json_object_object_get_ex(object, key, &value) ||
        !json_object_is_type(value, json_type_string))
        return NULL;
    return json_object_get_string(value);
}

static unsigned json_uint(struct json_object *object, const char *key,
                          unsigned fallback) {
    struct json_object *value = NULL;
    if (!json_object_object_get_ex(object, key, &value) ||
        !json_object_is_type(value, json_type_int))
        return fallback;
    int64_t integer = json_object_get_int64(value);
    return integer < 0 || integer > UINT32_MAX ? fallback : (unsigned)integer;
}

static int send_capabilities(const struct driver_state *state, const char *id) {
    struct json_object *reply = json_object_new_object();
    struct json_object *domains = json_object_new_array();
    for (unsigned axis = 0; axis < MOTORS_AXIS_COUNT; ++axis)
        json_object_array_add(domains, json_object_new_int(ANJOY_ALL_AXES));
    json_object_object_add(reply, "version", json_object_new_int(1));
    json_object_object_add(reply, "id", json_object_new_string(id));
    json_object_object_add(reply, "ok", json_object_new_boolean(true));
    json_object_object_add(reply, "name", json_object_new_string("anjoy"));
    json_object_object_add(reply, "axes", json_object_new_int(ANJOY_ALL_AXES));
    json_object_object_add(reply, "stop_domains", domains);
    json_object_object_add(reply, "raw", json_object_new_boolean(true));
    json_object_object_add(reply, "settings", json_object_new_boolean(false));
    json_object_object_add(reply, "completed_mono_ms",
                           json_object_new_int64((int64_t)now_ms()));
    int result = send_json(state->control_fd, reply);
    json_object_put(reply);
    return result;
}

static bool valid_direction(unsigned axis, const char *direction) {
    if (!direction) return false;
    switch ((enum motors_axis)axis) {
    case MOTORS_AXIS_PAN:
        return !strcmp(direction, "left") || !strcmp(direction, "right");
    case MOTORS_AXIS_TILT:
        return !strcmp(direction, "up") || !strcmp(direction, "down");
    case MOTORS_AXIS_ZOOM:
        return !strcmp(direction, "tele") || !strcmp(direction, "wide");
    case MOTORS_AXIS_FOCUS:
        return !strcmp(direction, "near") || !strcmp(direction, "far");
    case MOTORS_AXIS_IRIS:
        return !strcmp(direction, "open") || !strcmp(direction, "close");
    default:
        return false;
    }
}

static int send_move(struct driver_state *state, unsigned axis, const char *dir) {
    uint8_t c1 = 0, c2 = 0, d1 = 0, d2 = 0;
    state->cancel_mcu_af_at_ms = 0;
    state->cancel_mcu_af_stop_ms = 0;
    state->cancel_mcu_af_passes = 0;
    state->zoom_was_active = (axis == MOTORS_AXIS_ZOOM);
    switch ((enum motors_axis)axis) {
    case MOTORS_AXIS_PAN:
        d1 = (uint8_t)state->config.pan_speed;
        c2 = !strcmp(dir, "left") ? 0x04 : 0x02;
        break;
    case MOTORS_AXIS_TILT:
        d2 = (uint8_t)state->config.tilt_speed;
        c2 = !strcmp(dir, "up") ? 0x08 : 0x10;
        break;
    case MOTORS_AXIS_ZOOM:
        d1 = d2 = 0x14;
        c2 = !strcmp(dir, "tele") ? 0x20 : 0x40;
        break;
    case MOTORS_AXIS_FOCUS:
        d1 = d2 = 0x30;
        if (!strcmp(dir, "near")) c1 = 0x01;
        else c2 = 0x80;
        break;
    case MOTORS_AXIS_IRIS:
        d1 = d2 = 0x30;
        if (!strcmp(dir, "open")) c1 = 0x02;
        else c1 = 0x04;
        break;
    default:
        return -1;
    }
    return send_cmd_frame(state, c1, c2, d1, d2);
}

static int send_anjoy_ircut(struct driver_state *state, bool night) {
    /* 1. Anjoy proprietary Day/Night mode 10-byte serial packet */
    uint8_t frame[10] = {
        0x51, 0x01, 0x04, 0x79, 0x77, 0x03, 0x85, 0x01,
        night ? 0x01 : 0x00, 0x00
    };
    uint16_t sum = 0;
    for (int i = 0; i < 9; ++i) sum += frame[i];
    frame[9] = (uint8_t)(sum & 0xff);
    int rc = write_all(state->uart_fd, frame, sizeof(frame));

    /* 2. Anjoy proprietary IR LED control 10-byte serial packet (Cmd 0x04) */
    uint8_t led_frame[10] = {
        0x51, 0x01, 0x04, 0x79, 0x77, 0x03, 0x85, 0x04,
        night ? 0x01 : 0x00, 0x00
    };
    sum = 0;
    for (int i = 0; i < 9; ++i) sum += led_frame[i];
    led_frame[9] = (uint8_t)(sum & 0xff);
    (void)write_all(state->uart_fd, led_frame, sizeof(led_frame));

    /* 3. Pelco-D Preset 105 (Night) / 106 & 104 (Day) as sent by stock firmware */
    if (night) {
        (void)send_cmd_frame(state, 0x00, 0x07, 0x00, 105);
    } else {
        (void)send_cmd_frame(state, 0x00, 0x07, 0x00, 106);
        (void)send_cmd_frame(state, 0x00, 0x07, 0x00, 104);
    }

    /* 4. Board GPIO 23 (IRCUT A / night) / 24 (IRCUT B / day) pulse */
    const char *gpio_path = night ? "/sys/class/gpio/gpio23/value" : "/sys/class/gpio/gpio24/value";
    int gpio_fd = open(gpio_path, O_WRONLY);
    if (gpio_fd >= 0) {
        (void)write(gpio_fd, "1\n", 2);
        usleep(100000); /* 100ms pulse */
        (void)write(gpio_fd, "0\n", 2);
        close(gpio_fd);
    }

    return rc;
}

static int handle_raw(struct driver_state *state, struct json_object *request) {
    struct json_object *payload = NULL;
    if (!json_object_object_get_ex(request, "payload", &payload) ||
        !json_object_is_type(payload, json_type_object))
        return -1;
    const char *hex = json_string(payload, "bytes");
    if (!hex) return -1;
    size_t length = strlen(hex);
    if (!length || (length % 2) != 0 || length > 128) return -1;
    uint8_t bytes[64];
    for (size_t i = 0; i < length; i += 2) {
        char octet[3] = {hex[i], hex[i + 1], '\0'};
        char *end = NULL;
        unsigned long value = strtoul(octet, &end, 16);
        if (!end || *end) return -1;
        bytes[i / 2] = (uint8_t)value;
    }
    return write_all(state->uart_fd, bytes, length / 2);
}

static int handle_request(struct driver_state *state, struct json_object *request) {
    const char *id = json_string(request, "id");
    const char *operation = json_string(request, "op");
    if (!id || !operation)
        return send_reply(state->control_fd, id, false, "invalid request");

    if (!strcmp(operation, "capabilities"))
        return send_capabilities(state, id);

    if (!strcmp(operation, "move")) {
        unsigned axis = json_uint(request, "axis", MOTORS_AXIS_COUNT);
        const char *direction = json_string(request, "direction");
        unsigned duration = json_uint(request, "duration_ms", 0);
        if (!valid_direction(axis, direction))
            return send_reply(state->control_fd, id, false, "invalid movement");
        if (send_move(state, axis, direction) != 0) {
            char error[160];
            snprintf(error, sizeof(error), "transport write failed: %s",
                     strerror(errno));
            (void)send_reply(state->control_fd, id, false, error);
            return -1;
        }
        state->active = ANJOY_ALL_AXES;
        memset(state->deadlines, 0, sizeof(state->deadlines));
        if (duration) state->deadlines[axis] = now_ms() + duration;
        return send_reply(state->control_fd, id, true, "sent");
    }

    if (!strcmp(operation, "stop")) {
        unsigned axes = json_uint(request, "axes", 0);
        if (!(axes & ANJOY_ALL_AXES))
            return send_reply(state->control_fd, id, false, "invalid stop axes");
        if (send_stop(state) != 0) {
            char error[160];
            snprintf(error, sizeof(error), "transport write failed: %s",
                     strerror(errno));
            (void)send_reply(state->control_fd, id, false, error);
            return -1;
        }
        return send_reply(state->control_fd, id, true, "sent");
    }

    if (!strcmp(operation, "raw")) {
        if (handle_raw(state, request) != 0)
            return send_reply(state->control_fd, id, false, "raw write failed");
        return send_reply(state->control_fd, id, true, "sent");
    }

    if (!strcmp(operation, "command")) {
        const char *name = json_string(request, "name");
        if (!name) return send_reply(state->control_fd, id, false, "command name required");
        if (!strcmp(name, "preset")) {
            const char *val_str = json_string(request, "value");
            unsigned preset = val_str ? (unsigned)atoi(val_str) : json_uint(request, "value", 0);
            if (preset > 255) return send_reply(state->control_fd, id, false, "invalid preset");
            if (send_cmd_frame(state, 0x00, 0x07, 0x00, (uint8_t)preset) != 0)
                return send_reply(state->control_fd, id, false, "preset write failed");
            return send_reply(state->control_fd, id, true, "preset sent");
        }
        if (!strcmp(name, "ircut")) {
            const char *val = json_string(request, "value");
            bool night = false;
            if (val && (!strcmp(val, "night") || !strcmp(val, "on") || !strcmp(val, "1")))
                night = true;
            if (send_anjoy_ircut(state, night) != 0)
                return send_reply(state->control_fd, id, false, "ircut write failed");
            return send_reply(state->control_fd, id, true, "ircut sent");
        }
        return send_reply(state->control_fd, id, false, "unsupported command");
    }

    if (!strcmp(operation, "shutdown")) {
        int result = state->active ? send_stop(state) : 0;
        (void)send_reply(state->control_fd, id, result == 0,
                         result == 0 ? "stopped" : "stop failed");
        return 1;
    }

    return send_reply(state->control_fd, id, false, "unsupported operation");
}

/* Parse reports:
 * Variable length format (MC800S SSC338Q):
 * 14-byte (payload_len=7): 51 01 04 78 01 07 0B 1A ZZ 21 26 26 26 CK
 * 15-byte (payload_len=8): 51 01 04 78 01 08 0B 1A ZZ ZZ 21 26 26 26 CK
 * 17-byte (payload_len=10): 51 01 04 78 01 0A 0A 10 3B ZZ ZZ 21 3B 14 0F 3B CK
 */
static int parse_telemetry(struct driver_state *state) {
    while (state->rx_len >= 7) {
        if (state->rx_buf[0] != 0x51 || state->rx_buf[1] != 0x01 ||
            state->rx_buf[2] != 0x04 || state->rx_buf[3] != 0x78) {
            memmove(state->rx_buf, state->rx_buf + 1, --state->rx_len);
            continue;
        }

        uint8_t subtype = state->rx_buf[4];
        uint8_t payload_len = state->rx_buf[5];
        size_t frame_len = 6 + payload_len + 1;
        if (payload_len < 1 || payload_len > 32) {
            memmove(state->rx_buf, state->rx_buf + 1, --state->rx_len);
            continue;
        }
        if (state->rx_len < frame_len) {
            break;
        }

        uint16_t sum = 0;
        for (size_t i = 0; i < frame_len - 1; ++i) sum += state->rx_buf[i];
        if ((sum & 0xff) != state->rx_buf[frame_len - 1]) {
            memmove(state->rx_buf, state->rx_buf + 1, --state->rx_len);
            continue;
        }

        if (subtype == 0x01 && payload_len >= 7 &&
            state->rx_buf[6] == 0x0b && state->rx_buf[7] == 0x1a) {
            double mag = -1.0;
            if (payload_len == 7 && state->rx_buf[8] <= 9) {
                /* Sub-1.0x overshoot frame */
                mag = 1.0;
            } else if (payload_len >= 8 && state->rx_buf[8] <= 9 && state->rx_buf[9] <= 9) {
                /* Anjoy zoom telemetry: byte 8 is tens (0..9), byte 9 is units (0..9) e.g. 0x04, 0x01 = 41x */
                mag = (double)state->rx_buf[8] * 10.0 + (double)state->rx_buf[9];
            }
            if (mag >= 0.0) {
                if (mag < 1.0) mag = 1.0;
                if (mag > 100.0) mag = 100.0;
                (void)send_zoom_magnification(state, mag);
            }
        } else if (subtype == 0x79) {
            if (payload_len == 3 && state->rx_buf[6] == 0x01 &&
                state->rx_buf[7] == 0x04 && state->rx_buf[8] == 0x00) {
                /* MCU reported ZOOM OVER (01 04 00): cancel MCU internal AF hunting */
                if (!state->active &&
                    (state->zoom_was_active || state->cancel_mcu_af_passes > 0)) {
                    state->zoom_was_active = false;
                    state->cancel_mcu_af_stop_ms = 0;
                    state->cancel_mcu_af_passes = 2;
                    state->cancel_mcu_af_at_ms = now_ms() + 120;
                }
            } else if (payload_len >= 2 && state->rx_buf[6] == 0x82 &&
                       state->rx_buf[7] == 0x01) {
                static const uint8_t ack1[10] = {
                    0x51, 0x01, 0x04, 0x79, 0x78, 0x03, 0x02, 0x01, 0x01, 0x4e
                };
                (void)write_all(state->uart_fd, ack1, sizeof(ack1));
            } else if (payload_len >= 2 && state->rx_buf[6] == 0x86 &&
                       state->rx_buf[7] == 0x01) {
                static const uint8_t ack2[10] = {
                    0x51, 0x01, 0x04, 0x79, 0x78, 0x03, 0x06, 0x01, 0x01, 0x52
                };
                (void)write_all(state->uart_fd, ack2, sizeof(ack2));
            }
        }

        state->rx_len -= frame_len;
        memmove(state->rx_buf, state->rx_buf + frame_len, state->rx_len);
    }
    return 0;
}

static int read_uart(struct driver_state *state) {
    uint8_t input[128];
    for (;;) {
        ssize_t count = read(state->uart_fd, input, sizeof(input));
        if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return 0;
        if (count < 0 && errno == EINTR) continue;
        if (count <= 0) return count == 0 ? 0 : -1;

        if (state->rx_len + (size_t)count <= sizeof(state->rx_buf)) {
            memcpy(state->rx_buf + state->rx_len, input, (size_t)count);
            state->rx_len += (size_t)count;
        } else {
            /* Overflow guard */
            state->rx_len = 0;
        }
        if (parse_telemetry(state) != 0) return -1;
    }
}

static int next_timeout(const struct driver_state *state) {
    uint64_t now = now_ms(), nearest = state->next_keepalive_ms;
    for (unsigned axis = 0; axis < MOTORS_AXIS_COUNT; ++axis) {
        uint64_t deadline = state->deadlines[axis];
        if (deadline && (!nearest || deadline < nearest)) nearest = deadline;
    }
    if (state->cancel_mcu_af_at_ms &&
        (!nearest || state->cancel_mcu_af_at_ms < nearest))
        nearest = state->cancel_mcu_af_at_ms;
    if (state->cancel_mcu_af_stop_ms &&
        (!nearest || state->cancel_mcu_af_stop_ms < nearest))
        nearest = state->cancel_mcu_af_stop_ms;
    if (!nearest) return -1;
    if (nearest <= now) return 0;
    uint64_t delay = nearest - now;
    return delay > INT32_MAX ? INT32_MAX : (int)delay;
}

static int check_deadlines(struct driver_state *state) {
    uint64_t now = now_ms();
    uint32_t ended = 0;
    for (unsigned axis = 0; axis < MOTORS_AXIS_COUNT; ++axis) {
        if (state->deadlines[axis] && state->deadlines[axis] <= now)
            ended |= MOTORS_AXIS_BIT(axis);
    }
    if (ended) {
        if (send_stop(state) != 0) return -1;
        if (send_movement_event(state->control_fd, ANJOY_ALL_AXES) != 0)
            return -1;
        now = now_ms();
    }
    if (state->cancel_mcu_af_stop_ms && now >= state->cancel_mcu_af_stop_ms) {
        state->cancel_mcu_af_stop_ms = 0;
        if (!state->active) {
            if (send_cmd_frame(state, 0x00, 0x00, 0x00, 0x00) != 0) return -1;
            if (state->cancel_mcu_af_passes > 0) {
                state->cancel_mcu_af_at_ms = now_ms() + 200;
            }
        }
    }
    if (state->cancel_mcu_af_at_ms && now >= state->cancel_mcu_af_at_ms) {
        state->cancel_mcu_af_at_ms = 0;
        if (!state->active && state->cancel_mcu_af_passes > 0) {
            /* Pass 2 sends focus near (c1=0x01), Pass 1 sends focus far (c2=0x80) */
            bool near_dir = (state->cancel_mcu_af_passes >= 2);
            state->cancel_mcu_af_passes--;
            if (send_cmd_frame(state, near_dir ? 0x01 : 0x00,
                               near_dir ? 0x00 : 0x80, 0x30, 0x30) != 0)
                return -1;
            state->cancel_mcu_af_stop_ms = now_ms() + 65;
        }
    }
    if (now >= state->next_keepalive_ms) {
        if (send_keepalive(state) != 0) return -1;
    }
    return 0;
}

static void usage(const char *name) {
    fprintf(stderr, "Usage: %s --fd NUMBER --config FILE\n", name);
}

int main(int argc, char **argv) {
    int control_fd = -1;
    const char *configuration = NULL;
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--fd") && i + 1 < argc)
            control_fd = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--config") && i + 1 < argc)
            configuration = argv[++i];
        else {
            usage(argv[0]);
            return 2;
        }
    }
    if (control_fd < 0 || !configuration) {
        usage(argv[0]);
        return 2;
    }

    static struct driver_state state;
    memset(&state, 0, sizeof(state));
    state.control_fd = control_fd;
    state.uart_fd = -1;

    if (load_configuration(configuration, &state.config) != 0) {
        fprintf(stderr, "Anjoy configuration failed: %s\n", strerror(errno));
        return 2;
    }

    state.uart_fd = open_uart(&state.config);
    if (state.uart_fd < 0) {
        fprintf(stderr, "open %s: %s\n", state.config.device, strerror(errno));
        return 2;
    }

    /* Prime session with initial keepalive frame */
    if (send_keepalive(&state) != 0) {
        fprintf(stderr, "initial keepalive failed: %s\n", strerror(errno));
        close(state.uart_fd);
        return 2;
    }

    /* Send safe stop frame and schedule boot-time MCU internal AF cancel */
    state.zoom_was_active = true;
    if (send_stop(&state) != 0) {
        fprintf(stderr, "safe startup failed: %s\n", strerror(errno));
        close(state.uart_fd);
        return 2;
    }

    int exit_code = 0;
    for (;;) {
        struct pollfd descriptors[2] = {
            {.fd = control_fd, .events = POLLIN},
            {.fd = state.uart_fd, .events = POLLIN},
        };
        int result = poll(descriptors, 2, next_timeout(&state));
        if (result < 0) {
            if (errno == EINTR) continue;
            exit_code = 1;
            break;
        }
        if (check_deadlines(&state) != 0) {
            exit_code = 1;
            break;
        }
        if (!result) continue;
        if (descriptors[1].revents & POLLIN && read_uart(&state) != 0) {
            exit_code = 1;
            break;
        }
        if (!(descriptors[0].revents & POLLIN)) continue;

        char message[MESSAGE_MAX + 1];
        ssize_t length = recv(control_fd, message, MESSAGE_MAX, 0);
        if (length <= 0) {
            if (state.active && send_stop(&state) != 0) exit_code = 1;
            break;
        }
        message[length] = '\0';
        struct json_object *request = json_tokener_parse(message);
        if (!request) {
            if (send_reply(control_fd, "", false, "invalid JSON") != 0) break;
            continue;
        }
        int handled = handle_request(&state, request);
        json_object_put(request);
        if (handled != 0) {
            if (handled < 0) exit_code = 1;
            break;
        }
    }

    close(state.uart_fd);
    close(control_fd);
    return exit_code;
}
