#define _POSIX_C_SOURCE 200809L

#include "libmotors.h"

#include <errno.h>
#include <json-c/json.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

#define MESSAGE_MAX 32768

struct motors_client {
    int fd;
    unsigned request_id;
    unsigned revoked_axes;
    unsigned completed_axes;
    uint64_t completed_mono_ms[MOTORS_IRIS + 1];
};

static const char *const axes[] = {"pan", "tilt", "zoom", "focus", "iris"};
static const char *const roles[] = {"automation", "af", "manual"};
static const char *const directions[] = {
    "left", "right", "up", "down", "tele", "wide", "near", "far"
};

static void set_error(char *error, size_t size, const char *message) {
    if (error && size) snprintf(error, size, "%s", message);
}

static int process_event(struct motors_client *client,
                         struct json_object *object) {
    struct json_object *event = NULL;
    if (!json_object_object_get_ex(object, "event", &event) ||
        !json_object_is_type(event, json_type_string))
        return 0;
    if (!strcmp(json_object_get_string(event), "lease_revoked")) {
        struct json_object *axes = NULL;
        if (json_object_object_get_ex(object, "axes", &axes) &&
            json_object_is_type(axes, json_type_int))
            client->revoked_axes |= (unsigned)json_object_get_int64(axes);
    } else if (!strcmp(json_object_get_string(event), "movement_ended")) {
        struct json_object *axes = NULL;
        struct json_object *completed = NULL;
        if (json_object_object_get_ex(object, "axes", &axes) &&
            json_object_is_type(axes, json_type_int)) {
            unsigned mask = (unsigned)json_object_get_int64(axes);
            uint64_t timestamp = 0;
            if (json_object_object_get_ex(object, "driver_completed_mono_ms",
                                          &completed) &&
                json_object_is_type(completed, json_type_int))
                timestamp = (uint64_t)json_object_get_int64(completed);
            client->completed_axes |= mask;
            for (unsigned axis = 0; axis <= MOTORS_IRIS; ++axis) {
                if (mask & (1U << axis))
                    client->completed_mono_ms[axis] = timestamp;
            }
        }
    }
    return 1;
}

int motors_open(struct motors_client **out, const char *socket_path,
                char *error, size_t error_size) {
    if (!out || !socket_path ||
        strlen(socket_path) >= sizeof(((struct sockaddr_un *)0)->sun_path)) {
        set_error(error, error_size, "invalid socket path");
        return -1;
    }
    struct motors_client *client = calloc(1, sizeof(*client));
    if (!client) {
        set_error(error, error_size, "out of memory");
        return -1;
    }
    client->fd = socket(AF_UNIX, SOCK_SEQPACKET, 0);
    if (client->fd < 0) {
        set_error(error, error_size, strerror(errno));
        free(client);
        return -1;
    }
    struct sockaddr_un address;
    memset(&address, 0, sizeof(address));
    address.sun_family = AF_UNIX;
    snprintf(address.sun_path, sizeof(address.sun_path), "%s", socket_path);
    if (connect(client->fd, (struct sockaddr *)&address, sizeof(address)) != 0) {
        set_error(error, error_size, strerror(errno));
        close(client->fd);
        free(client);
        return -1;
    }
    *out = client;
    return 0;
}

void motors_close(struct motors_client *client) {
    if (!client) return;
    close(client->fd);
    free(client);
}

int motors_request(struct motors_client *client, const char *request,
                   char *response, size_t response_size,
                   char *error, size_t error_size) {
    if (!client || !request || !response || response_size < 2) {
        set_error(error, error_size, "invalid request");
        return -1;
    }
    struct json_object *request_object = json_tokener_parse(request);
    struct json_object *request_id = NULL;
    if (!request_object ||
        !json_object_object_get_ex(request_object, "id", &request_id) ||
        !json_object_is_type(request_id, json_type_string)) {
        if (request_object) json_object_put(request_object);
        set_error(error, error_size, "request id required");
        return -1;
    }
    const char *expected_id = json_object_get_string(request_id);
    size_t length = strlen(request);
    if (length > MESSAGE_MAX || send(client->fd, request, length, MSG_NOSIGNAL) < 0) {
        set_error(error, error_size, strerror(errno));
        json_object_put(request_object);
        return -1;
    }
    for (;;) {
        ssize_t received = recv(client->fd, response, response_size - 1, 0);
        if (received <= 0) {
            set_error(error, error_size, "service disconnected");
            json_object_put(request_object);
            return -1;
        }
        response[received] = '\0';
        struct json_object *object = json_tokener_parse(response);
        if (object && process_event(client, object)) {
            json_object_put(object);
            continue;
        }
        struct json_object *ok = NULL;
        struct json_object *response_id = NULL;
        if (!object ||
            !json_object_object_get_ex(object, "id", &response_id) ||
            !json_object_is_type(response_id, json_type_string) ||
            strcmp(json_object_get_string(response_id), expected_id) ||
            !json_object_object_get_ex(object, "ok", &ok) ||
            !json_object_get_boolean(ok)) {
            struct json_object *detail = NULL;
            if (object && json_object_object_get_ex(object, "error", &detail))
                set_error(error, error_size, json_object_get_string(detail));
            else
                set_error(error, error_size, "invalid service response");
            if (object) json_object_put(object);
            json_object_put(request_object);
            return -1;
        }
        json_object_put(object);
        json_object_put(request_object);
        return 0;
    }
}

static int send_object(struct motors_client *client, struct json_object *request,
                       char *error, size_t error_size) {
    char response[MESSAGE_MAX + 1];
    const char *text = json_object_to_json_string_ext(request, JSON_C_TO_STRING_PLAIN);
    int result = motors_request(client, text, response, sizeof(response),
                                error, error_size);
    json_object_put(request);
    return result;
}

static struct json_object *new_request(struct motors_client *client,
                                       const char *operation) {
    char id[32];
    snprintf(id, sizeof(id), "libmotors-%u", ++client->request_id);
    struct json_object *request = json_object_new_object();
    json_object_object_add(request, "version", json_object_new_int(1));
    json_object_object_add(request, "id", json_object_new_string(id));
    json_object_object_add(request, "op", json_object_new_string(operation));
    return request;
}

int motors_get_capabilities(struct motors_client *client,
                            struct motors_capabilities *capabilities,
                            char *error, size_t error_size) {
    if (!client || !capabilities) {
        set_error(error, error_size, "invalid capabilities arguments");
        return -1;
    }

    struct json_object *request = new_request(client, "capabilities");
    const char *text = json_object_to_json_string_ext(request, JSON_C_TO_STRING_PLAIN);
    char response[MESSAGE_MAX + 1];
    int result = motors_request(client, text, response, sizeof(response),
                                error, error_size);
    json_object_put(request);
    if (result != 0) return -1;

    struct json_object *object = json_tokener_parse(response);
    struct json_object *driver = NULL;
    struct json_object *axis_list = NULL;
    struct json_object *raw = NULL;
    struct json_object *available = NULL;
    if (!object ||
        !json_object_object_get_ex(object, "driver", &driver) ||
        !json_object_is_type(driver, json_type_string) ||
        !json_object_object_get_ex(object, "axes", &axis_list) ||
        !json_object_is_type(axis_list, json_type_array) ||
        !json_object_object_get_ex(object, "raw", &raw) ||
        !json_object_is_type(raw, json_type_boolean) ||
        !json_object_object_get_ex(object, "available", &available) ||
        !json_object_is_type(available, json_type_boolean)) {
        if (object) json_object_put(object);
        set_error(error, error_size, "invalid capabilities response");
        return -1;
    }

    struct motors_capabilities parsed = {0};
    snprintf(parsed.driver, sizeof(parsed.driver), "%s",
             json_object_get_string(driver));
    size_t count = json_object_array_length(axis_list);
    for (size_t item = 0; item < count; ++item) {
        struct json_object *axis = json_object_array_get_idx(axis_list, item);
        if (!json_object_is_type(axis, json_type_string)) {
            json_object_put(object);
            set_error(error, error_size, "invalid capabilities response");
            return -1;
        }
        const char *name = json_object_get_string(axis);
        for (unsigned index = 0; index <= MOTORS_IRIS; ++index) {
            if (!strcmp(name, axes[index])) {
                parsed.axes |= MOTORS_AXIS_MASK(index);
                break;
            }
        }
    }
    parsed.raw = json_object_get_boolean(raw);
    parsed.available = json_object_get_boolean(available);
    json_object_put(object);
    *capabilities = parsed;
    return 0;
}

int motors_describe(struct motors_client *client,
                    char *response, size_t response_size,
                    char *error, size_t error_size) {
    if (!client || !response || response_size < 2) {
        set_error(error, error_size, "invalid describe arguments");
        return -1;
    }
    struct json_object *request = new_request(client, "describe");
    const char *text = json_object_to_json_string_ext(request,
                                                       JSON_C_TO_STRING_PLAIN);
    int result = motors_request(client, text, response, response_size,
                                error, error_size);
    json_object_put(request);
    return result;
}

int motors_command(struct motors_client *client, const char *name,
                   const char *value, char *error, size_t error_size) {
    if (!client || !name || !*name) {
        set_error(error, error_size, "invalid command arguments");
        return -1;
    }
    struct json_object *request = new_request(client, "command");
    json_object_object_add(request, "name", json_object_new_string(name));
    if (value)
        json_object_object_add(request, "value", json_object_new_string(value));
    return send_object(client, request, error, error_size);
}

int motors_acquire(struct motors_client *client, enum motors_client_role role,
                   enum motors_client_axis axis, unsigned lease_ms,
                   char *error, size_t error_size) {
    if (!client || role > MOTORS_MANUAL || axis > MOTORS_IRIS || !lease_ms) {
        set_error(error, error_size, "invalid acquire arguments");
        return -1;
    }
    struct json_object *request = new_request(client, "acquire");
    json_object_object_add(request, "role", json_object_new_string(roles[role]));
    json_object_object_add(request, "axis", json_object_new_string(axes[axis]));
    json_object_object_add(request, "lease_ms", json_object_new_int64(lease_ms));
    int result = send_object(client, request, error, error_size);
    if (result == 0) client->revoked_axes &= ~(1U << (unsigned)axis);
    return result;
}

int motors_move(struct motors_client *client, enum motors_client_axis axis,
                enum motors_direction direction, unsigned duration_ms,
                char *error, size_t error_size) {
    if (!client || axis > MOTORS_IRIS || direction > MOTORS_FAR) {
        set_error(error, error_size, "invalid move arguments");
        return -1;
    }
    struct json_object *request = new_request(client, "move");
    json_object_object_add(request, "axis", json_object_new_string(axes[axis]));
    json_object_object_add(request, "direction",
                           json_object_new_string(directions[direction]));
    json_object_object_add(request, "duration_ms", json_object_new_int64(duration_ms));
    int result = send_object(client, request, error, error_size);
    if (result == 0 && duration_ms)
        client->completed_axes &= ~(1U << (unsigned)axis);
    return result;
}

int motors_stop(struct motors_client *client, enum motors_client_axis axis,
                char *error, size_t error_size) {
    if (!client || axis > MOTORS_IRIS) {
        set_error(error, error_size, "invalid stop arguments");
        return -1;
    }
    struct json_object *request = new_request(client, "stop");
    json_object_object_add(request, "axis", json_object_new_string(axes[axis]));
    return send_object(client, request, error, error_size);
}

int motors_stop_all(struct motors_client *client,
                    char *error, size_t error_size) {
    if (!client) {
        set_error(error, error_size, "invalid stop arguments");
        return -1;
    }
    struct json_object *request = new_request(client, "stop");
    json_object_object_add(request, "axis", json_object_new_string("all"));
    return send_object(client, request, error, error_size);
}

int motors_release(struct motors_client *client, char *error, size_t error_size) {
    if (!client) {
        set_error(error, error_size, "invalid client");
        return -1;
    }
    return send_object(client, new_request(client, "release"), error, error_size);
}

int motors_subscribe(struct motors_client *client, char *error, size_t error_size) {
    if (!client) {
        set_error(error, error_size, "invalid client");
        return -1;
    }
    return send_object(client, new_request(client, "subscribe"), error, error_size);
}

int motors_poll(struct motors_client *client, int timeout_ms,
                char *error, size_t error_size) {
    if (!client || timeout_ms < 0) {
        set_error(error, error_size, "invalid poll arguments");
        return -1;
    }
    struct pollfd descriptor = {.fd = client->fd, .events = POLLIN};
    int result;
    do {
        result = poll(&descriptor, 1, timeout_ms);
    } while (result < 0 && errno == EINTR);
    if (result <= 0) {
        if (result < 0) set_error(error, error_size, strerror(errno));
        return result;
    }
    char message[MESSAGE_MAX + 1];
    ssize_t received = recv(client->fd, message, MESSAGE_MAX, 0);
    if (received <= 0) {
        set_error(error, error_size, "service disconnected");
        return -1;
    }
    message[received] = '\0';
    struct json_object *object = json_tokener_parse(message);
    int is_event = object ? process_event(client, object) : 0;
    if (object) json_object_put(object);
    if (!is_event) {
        set_error(error, error_size, "unexpected service response");
        return -1;
    }
    return 1;
}

bool motors_lease_revoked(const struct motors_client *client,
                          enum motors_client_axis axis) {
    return client && axis <= MOTORS_IRIS &&
           (client->revoked_axes & (1U << (unsigned)axis));
}

static uint64_t monotonic_ms(void) {
    struct timespec value;
    clock_gettime(CLOCK_MONOTONIC, &value);
    return (uint64_t)value.tv_sec * 1000U + (uint64_t)value.tv_nsec / 1000000U;
}

int motors_wait_movement(struct motors_client *client,
                         enum motors_client_axis axis, int timeout_ms,
                         uint64_t *completed_mono_ms,
                         char *error, size_t error_size) {
    if (!client || axis > MOTORS_IRIS || timeout_ms < 0) {
        set_error(error, error_size, "invalid movement wait arguments");
        return -1;
    }
    uint64_t deadline = monotonic_ms() + (unsigned)timeout_ms;
    unsigned bit = 1U << (unsigned)axis;
    for (;;) {
        if (client->revoked_axes & bit) {
            set_error(error, error_size, "lease revoked");
            return -1;
        }
        if (client->completed_axes & bit) {
            client->completed_axes &= ~bit;
            if (completed_mono_ms)
                *completed_mono_ms = client->completed_mono_ms[axis];
            return 0;
        }
        uint64_t now = monotonic_ms();
        if (now >= deadline) {
            set_error(error, error_size, "movement timed out");
            return -1;
        }
        uint64_t remaining = deadline - now;
        int wait = remaining > INT32_MAX ? INT32_MAX : (int)remaining;
        int result = motors_poll(client, wait, error, error_size);
        if (result < 0) return -1;
        if (result == 0) {
            set_error(error, error_size, "movement timed out");
            return -1;
        }
    }
}
