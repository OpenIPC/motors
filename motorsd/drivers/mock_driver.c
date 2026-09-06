#define _GNU_SOURCE

#include "motors_driver.h"

#include <errno.h>
#include <json-c/json.h>
#include <poll.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#define MESSAGE_MAX 32768

static uint64_t now_ms(void) {
    struct timespec value;
    clock_gettime(CLOCK_MONOTONIC, &value);
    return (uint64_t)value.tv_sec * 1000U + (uint64_t)value.tv_nsec / 1000000U;
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
    const char *text = json_object_to_json_string_ext(reply, JSON_C_TO_STRING_PLAIN);
    int result = send(fd, text, strlen(text), MSG_NOSIGNAL) < 0 ? -1 : 0;
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
    const char *text = json_object_to_json_string_ext(event, JSON_C_TO_STRING_PLAIN);
    int result = send(fd, text, strlen(text), MSG_NOSIGNAL) < 0 ? -1 : 0;
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
    int64_t number = json_object_get_int64(value);
    return number >= 0 && number <= UINT32_MAX ? (unsigned)number : fallback;
}

static int read_trace_path(const char *configuration, char *path, size_t path_size) {
    FILE *file = fopen(configuration, "r");
    if (!file) return -1;
    char line[512];
    int found = 0;
    while (fgets(line, sizeof(line), file)) {
        char *newline = strpbrk(line, "\r\n");
        if (newline) *newline = '\0';
        if (!strncmp(line, "trace=", 6) && line[6]) {
            size_t length = strlen(line + 6);
            if (length >= path_size) {
                fclose(file);
                errno = ENAMETOOLONG;
                return -1;
            }
            memcpy(path, line + 6, length + 1);
            found = 1;
        }
    }
    fclose(file);
    if (!found) errno = EINVAL;
    return found ? 0 : -1;
}

static int handle_request(int fd, FILE *trace, uint32_t *active,
                          uint64_t deadlines[MOTORS_AXIS_COUNT],
                          struct json_object *request) {
    const char *id = json_string(request, "id");
    const char *operation = json_string(request, "op");
    if (!id || !operation)
        return send_reply(fd, id, false, "invalid driver request");

    if (!strcmp(operation, "capabilities")) {
        struct json_object *reply = json_object_new_object();
        struct json_object *domains = json_object_new_array();
        for (unsigned axis = 0; axis < MOTORS_AXIS_COUNT; ++axis)
            json_object_array_add(domains,
                                  json_object_new_int((int)MOTORS_AXIS_BIT(axis)));
        json_object_object_add(reply, "version",
                               json_object_new_int(MOTORS_DRIVER_PROTOCOL_VERSION));
        json_object_object_add(reply, "id", json_object_new_string(id));
        json_object_object_add(reply, "ok", json_object_new_boolean(true));
        json_object_object_add(reply, "name", json_object_new_string("mock"));
        json_object_object_add(reply, "axes", json_object_new_int(0x0f));
        json_object_object_add(reply, "stop_domains", domains);
        json_object_object_add(reply, "raw", json_object_new_boolean(true));
        json_object_object_add(reply, "completed_mono_ms",
                               json_object_new_int64((int64_t)now_ms()));
        const char *text = json_object_to_json_string_ext(reply, JSON_C_TO_STRING_PLAIN);
        int result = send(fd, text, strlen(text), MSG_NOSIGNAL) < 0 ? -1 : 0;
        json_object_put(reply);
        return result;
    }

    if (!strcmp(operation, "describe")) {
        struct json_object *reply = json_object_new_object();
        struct json_object *controls = json_object_new_array();
        struct json_object *control = json_object_new_object();
        json_object_object_add(control, "name", json_object_new_string("test.level"));
        json_object_object_add(control, "label", json_object_new_string("Test level"));
        json_object_object_add(control, "type", json_object_new_string("number"));
        json_object_object_add(control, "min", json_object_new_int(1));
        json_object_object_add(control, "max", json_object_new_int(10));
        json_object_array_add(controls, control);
        json_object_object_add(reply, "version", json_object_new_int(1));
        json_object_object_add(reply, "id", json_object_new_string(id));
        json_object_object_add(reply, "ok", json_object_new_boolean(true));
        json_object_object_add(reply, "controls", controls);
        int result = send(fd, json_object_to_json_string_ext(
            reply, JSON_C_TO_STRING_PLAIN), strlen(json_object_to_json_string_ext(
            reply, JSON_C_TO_STRING_PLAIN)), MSG_NOSIGNAL) < 0 ? -1 : 0;
        json_object_put(reply);
        return result;
    }

    if (!strcmp(operation, "command")) {
        const char *name = json_string(request, "name");
        struct json_object *value = NULL;
        if (!name || !json_object_object_get_ex(request, "value", &value))
            return send_reply(fd, id, false, "invalid command");
        fprintf(trace, "COMMAND name=%s value=%s\n", name,
                json_object_to_json_string_ext(value, JSON_C_TO_STRING_PLAIN));
        return send_reply(fd, id, true, "sent");
    }

    if (!strcmp(operation, "move")) {
        unsigned axis = json_uint(request, "axis", MOTORS_AXIS_COUNT);
        const char *direction = json_string(request, "direction");
        unsigned duration = json_uint(request, "duration_ms", 0);
        if (axis >= MOTORS_AXIS_COUNT || !direction)
            return send_reply(fd, id, false, "invalid move");
        fprintf(trace, "MOVE axis=%u direction=%s duration_ms=%u\n",
                axis, direction, duration);
        *active |= MOTORS_AXIS_BIT(axis);
        deadlines[axis] = duration ? now_ms() + duration : 0;
        return send_reply(fd, id, true, "sent");
    }

    if (!strcmp(operation, "stop")) {
        unsigned axes = json_uint(request, "axes", 0);
        if (!axes) return send_reply(fd, id, false, "invalid stop");
        fprintf(trace, "STOP axes=0x%02x\n", axes);
        *active &= ~axes;
        for (unsigned axis = 0; axis < MOTORS_AXIS_COUNT; ++axis) {
            if (axes & MOTORS_AXIS_BIT(axis)) deadlines[axis] = 0;
        }
        return send_reply(fd, id, true, "sent");
    }

    if (!strcmp(operation, "raw")) {
        struct json_object *payload = NULL;
        if (!json_object_object_get_ex(request, "payload", &payload))
            return send_reply(fd, id, false, "raw payload required");
        struct json_object *crash = NULL;
        if (json_object_is_type(payload, json_type_object) &&
            json_object_object_get_ex(payload, "crash", &crash) &&
            json_object_get_boolean(crash)) {
            fprintf(trace, "CRASH\n");
            fflush(trace);
            _exit(42);
        }
        fprintf(trace, "RAW payload=%s\n",
                json_object_to_json_string_ext(payload, JSON_C_TO_STRING_PLAIN));
        return send_reply(fd, id, true, "sent");
    }

    if (!strcmp(operation, "shutdown")) {
        if (*active) fprintf(trace, "STOP axes=0x%02x\n", *active);
        *active = 0;
        send_reply(fd, id, true, "stopped");
        return 1;
    }
    return send_reply(fd, id, false, "unsupported driver operation");
}

static int deadline_timeout(const uint64_t deadlines[MOTORS_AXIS_COUNT]) {
    uint64_t now = now_ms();
    uint64_t nearest = 0;
    for (unsigned axis = 0; axis < MOTORS_AXIS_COUNT; ++axis) {
        if (deadlines[axis] && (!nearest || deadlines[axis] < nearest))
            nearest = deadlines[axis];
    }
    if (!nearest) return -1;
    if (nearest <= now) return 0;
    uint64_t delay = nearest - now;
    return delay > INT32_MAX ? INT32_MAX : (int)delay;
}

static int finish_timed_movements(int fd, FILE *trace, uint32_t *active,
                                  uint64_t deadlines[MOTORS_AXIS_COUNT]) {
    uint64_t now = now_ms();
    uint32_t ended = 0;
    for (unsigned axis = 0; axis < MOTORS_AXIS_COUNT; ++axis) {
        if (deadlines[axis] && deadlines[axis] <= now) {
            ended |= MOTORS_AXIS_BIT(axis);
            deadlines[axis] = 0;
        }
    }
    if (!ended) return 0;
    fprintf(trace, "STOP axes=0x%02x\n", ended);
    *active &= ~ended;
    return send_movement_event(fd, ended);
}

static void usage(const char *name) {
    fprintf(stderr, "Usage: %s --fd NUMBER --config FILE\n", name);
}

int main(int argc, char **argv) {
    int fd = -1;
    const char *configuration = NULL;
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--fd") && i + 1 < argc)
            fd = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--config") && i + 1 < argc)
            configuration = argv[++i];
        else {
            usage(argv[0]);
            return 2;
        }
    }
    char trace_path[384] = "";
    if (fd < 0 || !configuration ||
        read_trace_path(configuration, trace_path, sizeof(trace_path)) != 0) {
        fprintf(stderr, "mock driver configuration failed: %s\n", strerror(errno));
        return 2;
    }
    FILE *trace = fopen(trace_path, "a");
    if (!trace) {
        fprintf(stderr, "open trace: %s\n", strerror(errno));
        return 2;
    }
    setvbuf(trace, NULL, _IOLBF, 0);
    fprintf(trace, "OPEN configuration=%s\n", configuration);
    fprintf(trace, "SAFE_START\n");

    uint32_t active = 0;
    uint64_t deadlines[MOTORS_AXIS_COUNT] = {0};
    int exit_code = 0;
    for (;;) {
        struct pollfd descriptor = {.fd = fd, .events = POLLIN};
        int poll_result = poll(&descriptor, 1, deadline_timeout(deadlines));
        if (poll_result < 0) {
            if (errno == EINTR) continue;
            exit_code = 1;
            break;
        }
        if (finish_timed_movements(fd, trace, &active, deadlines) != 0) {
            exit_code = 1;
            break;
        }
        if (poll_result == 0) continue;
        char message[MESSAGE_MAX + 1];
        ssize_t length = recv(fd, message, MESSAGE_MAX, 0);
        if (length <= 0) {
            if (active) fprintf(trace, "STOP axes=0x%02x\n", active);
            fprintf(trace, "CONTROL_LOST\n");
            break;
        }
        message[length] = '\0';
        struct json_object *request = json_tokener_parse(message);
        if (!request) {
            if (send_reply(fd, "", false, "invalid JSON") != 0) break;
            continue;
        }
        int result = handle_request(fd, trace, &active, deadlines, request);
        json_object_put(request);
        if (result < 0) {
            exit_code = 1;
            break;
        }
        if (result > 0) break;
    }
    fprintf(trace, "CLOSE\n");
    fclose(trace);
    close(fd);
    return exit_code;
}
