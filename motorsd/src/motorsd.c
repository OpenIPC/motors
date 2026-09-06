#define _GNU_SOURCE

#include "motors_driver.h"

#include <errno.h>
#include <json-c/json.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

#define CLIENT_MAX 16
#define MESSAGE_MAX 32768
#define POLL_INTERVAL_MS 20
#define DRIVER_RESTART_MAX 3

enum client_role {
    ROLE_NONE = 0,
    ROLE_AUTOMATION = 1,
    ROLE_AF = 2,
    ROLE_MANUAL = 3,
    ROLE_SAFETY = 4,
};

struct client {
    int fd;
    enum client_role role;
    bool subscribed;
    uint32_t leases;
    uint32_t active;
    uint64_t lease_deadline_ms[MOTORS_AXIS_COUNT];
};

struct daemon_state {
    int server;
    char socket_path[sizeof(((struct sockaddr_un *)0)->sun_path)];
    int driver_fd;
    pid_t driver_pid;
    const char *driver_path;
    const char *driver_configuration;
    bool driver_ready;
    struct motors_driver_caps caps;
    struct client clients[CLIENT_MAX];
};

static volatile sig_atomic_t stop_requested;
static unsigned driver_request_id;

static void handle_driver_event(struct daemon_state *state,
                                struct json_object *event);

static uint64_t now_ms(void) {
    struct timespec value;
    clock_gettime(CLOCK_MONOTONIC, &value);
    return (uint64_t)value.tv_sec * 1000U + (uint64_t)value.tv_nsec / 1000000U;
}

static void on_signal(int signal_number) {
    (void)signal_number;
    stop_requested = 1;
}

static const char *axis_name(enum motors_axis axis) {
    static const char *const names[] = {"pan", "tilt", "zoom", "focus", "iris"};
    return axis < MOTORS_AXIS_COUNT ? names[axis] : "unknown";
}

static int parse_axis(const char *name, enum motors_axis *axis) {
    if (!name || !axis) return -1;
    for (unsigned i = 0; i < MOTORS_AXIS_COUNT; ++i) {
        if (!strcmp(name, axis_name((enum motors_axis)i))) {
            *axis = (enum motors_axis)i;
            return 0;
        }
    }
    return -1;
}

static enum client_role parse_role(const char *name) {
    if (!name) return ROLE_NONE;
    if (!strcmp(name, "automation")) return ROLE_AUTOMATION;
    if (!strcmp(name, "af")) return ROLE_AF;
    if (!strcmp(name, "manual")) return ROLE_MANUAL;
    if (!strcmp(name, "safety")) return ROLE_SAFETY;
    return ROLE_NONE;
}

static const char *role_name(enum client_role role) {
    switch (role) {
    case ROLE_AUTOMATION: return "automation";
    case ROLE_AF: return "af";
    case ROLE_MANUAL: return "manual";
    case ROLE_SAFETY: return "safety";
    default: return "none";
    }
}

static const char *json_string(struct json_object *request, const char *key) {
    struct json_object *value = NULL;
    if (!json_object_object_get_ex(request, key, &value) ||
        !json_object_is_type(value, json_type_string))
        return NULL;
    return json_object_get_string(value);
}

static int json_uint(struct json_object *request, const char *key, unsigned *result) {
    struct json_object *value = NULL;
    if (!json_object_object_get_ex(request, key, &value) ||
        !json_object_is_type(value, json_type_int))
        return -1;
    int64_t number = json_object_get_int64(value);
    if (number < 0 || number > UINT32_MAX) return -1;
    *result = (unsigned)number;
    return 0;
}

static void send_response(struct client *client, const char *id, bool ok,
                          const char *format, ...) {
    struct json_object *response = json_object_new_object();
    json_object_object_add(response, "version", json_object_new_int(1));
    json_object_object_add(response, "id", json_object_new_string(id ? id : ""));
    json_object_object_add(response, "ok", json_object_new_boolean(ok));
    json_object_object_add(response, "t_mono_ms", json_object_new_int64((int64_t)now_ms()));
    if (format && *format) {
        char detail[512];
        va_list arguments;
        va_start(arguments, format);
        vsnprintf(detail, sizeof(detail), format, arguments);
        va_end(arguments);
        json_object_object_add(response, ok ? "state" : "error",
                               json_object_new_string(detail));
    }
    const char *text = json_object_to_json_string_ext(response, JSON_C_TO_STRING_PLAIN);
    (void)send(client->fd, text, strlen(text), MSG_NOSIGNAL);
    json_object_put(response);
}

static void send_driver_success(struct client *client, const char *id,
                                const char *state,
                                struct json_object *driver_reply) {
    struct json_object *response = json_object_new_object();
    struct json_object *completed = NULL;
    json_object_object_add(response, "version", json_object_new_int(1));
    json_object_object_add(response, "id", json_object_new_string(id));
    json_object_object_add(response, "ok", json_object_new_boolean(true));
    json_object_object_add(response, "state", json_object_new_string(state));
    json_object_object_add(response, "t_mono_ms",
                           json_object_new_int64((int64_t)now_ms()));
    if (json_object_object_get_ex(driver_reply, "completed_mono_ms", &completed))
        json_object_object_add(response, "driver_completed_mono_ms",
                               json_object_get(completed));
    const char *text = json_object_to_json_string_ext(response, JSON_C_TO_STRING_PLAIN);
    (void)send(client->fd, text, strlen(text), MSG_NOSIGNAL);
    json_object_put(response);
}

static void forward_driver_reply(struct client *client, const char *id,
                                 struct json_object *driver_reply) {
    struct json_object *response = json_object_get(driver_reply);
    json_object_object_del(response, "id");
    json_object_object_add(response, "id", json_object_new_string(id));
    json_object_object_del(response, "version");
    json_object_object_add(response, "version", json_object_new_int(1));
    const char *text = json_object_to_json_string_ext(response,
                                                       JSON_C_TO_STRING_PLAIN);
    (void)send(client->fd, text, strlen(text), MSG_NOSIGNAL);
    json_object_put(response);
}

static struct json_object *driver_request(struct daemon_state *state,
                                          struct json_object *request,
                                          char *error, size_t error_size) {
    const char *text = json_object_to_json_string_ext(request, JSON_C_TO_STRING_PLAIN);
    if (send(state->driver_fd, text, strlen(text), MSG_NOSIGNAL) < 0) {
        state->driver_ready = false;
        snprintf(error, error_size, "driver send: %s", strerror(errno));
        return NULL;
    }
    char message[MESSAGE_MAX + 1];
    struct json_object *reply = NULL;
    for (;;) {
        ssize_t length = recv(state->driver_fd, message, MESSAGE_MAX, 0);
        if (length <= 0) {
            state->driver_ready = false;
            snprintf(error, error_size, "driver disconnected");
            return NULL;
        }
        message[length] = '\0';
        reply = json_tokener_parse(message);
        struct json_object *event = NULL;
        if (reply && json_object_object_get_ex(reply, "event", &event)) {
            handle_driver_event(state, reply);
            json_object_put(reply);
            continue;
        }
        break;
    }
    struct json_object *ok = NULL;
    if (!reply || !json_object_object_get_ex(reply, "ok", &ok) ||
        !json_object_get_boolean(ok)) {
        struct json_object *detail = NULL;
        if (reply && json_object_object_get_ex(reply, "error", &detail))
            snprintf(error, error_size, "%s", json_object_get_string(detail));
        else
            snprintf(error, error_size, "invalid driver reply");
        if (reply) json_object_put(reply);
        return NULL;
    }
    return reply;
}

static struct json_object *new_driver_request(const char *operation) {
    char id[32];
    snprintf(id, sizeof(id), "driver-%u", ++driver_request_id);
    struct json_object *request = json_object_new_object();
    json_object_object_add(request, "version",
                           json_object_new_int(MOTORS_DRIVER_PROTOCOL_VERSION));
    json_object_object_add(request, "id", json_object_new_string(id));
    json_object_object_add(request, "op", json_object_new_string(operation));
    return request;
}

static uint32_t stop_domain(const struct daemon_state *state, enum motors_axis axis) {
    uint32_t domain = state->caps.stop_domain[axis];
    return domain ? domain : MOTORS_AXIS_BIT(axis);
}

static void clear_domain(struct daemon_state *state, uint32_t domain, bool clear_leases) {
    for (unsigned i = 0; i < CLIENT_MAX; ++i) {
        state->clients[i].active &= ~domain;
        if (clear_leases) state->clients[i].leases &= ~domain;
        for (unsigned axis = 0; axis < MOTORS_AXIS_COUNT; ++axis) {
            if (domain & MOTORS_AXIS_BIT(axis)) {
                if (clear_leases) state->clients[i].lease_deadline_ms[axis] = 0;
            }
        }
    }
}

static void handle_driver_event(struct daemon_state *state,
                                struct json_object *event) {
    struct json_object *name = NULL;
    struct json_object *axes = NULL;
    if (!json_object_object_get_ex(event, "event", &name) ||
        !json_object_object_get_ex(event, "axes", &axes))
        return;
    uint32_t domain = (uint32_t)json_object_get_int64(axes);
    clear_domain(state, domain, false);

    struct json_object *message = json_object_new_object();
    json_object_object_add(message, "version", json_object_new_int(1));
    json_object_object_add(message, "event", json_object_get(name));
    json_object_object_add(message, "axes", json_object_new_int64(domain));
    struct json_object *completed = NULL;
    if (json_object_object_get_ex(event, "completed_mono_ms", &completed))
        json_object_object_add(message, "driver_completed_mono_ms",
                               json_object_get(completed));
    const char *text = json_object_to_json_string_ext(message, JSON_C_TO_STRING_PLAIN);
    for (unsigned i = 0; i < CLIENT_MAX; ++i) {
        struct client *client = &state->clients[i];
        if (client->fd >= 0 && client->subscribed)
            (void)send(client->fd, text, strlen(text), MSG_NOSIGNAL | MSG_DONTWAIT);
    }
    json_object_put(message);
}

static void receive_driver_event(struct daemon_state *state) {
    char message[MESSAGE_MAX + 1];
    ssize_t length = recv(state->driver_fd, message, MESSAGE_MAX, MSG_DONTWAIT);
    if (length <= 0) return;
    message[length] = '\0';
    struct json_object *event = json_tokener_parse(message);
    if (event) {
        handle_driver_event(state, event);
        json_object_put(event);
    }
}

static void send_movement_ended(struct daemon_state *state, uint32_t domain,
                                struct json_object *driver_reply) {
    struct json_object *message = json_object_new_object();
    json_object_object_add(message, "version", json_object_new_int(1));
    json_object_object_add(message, "event",
                           json_object_new_string("movement_ended"));
    json_object_object_add(message, "axes", json_object_new_int64(domain));
    struct json_object *completed = NULL;
    if (json_object_object_get_ex(driver_reply, "completed_mono_ms", &completed))
        json_object_object_add(message, "driver_completed_mono_ms",
                               json_object_get(completed));
    const char *text = json_object_to_json_string_ext(message,
                                                       JSON_C_TO_STRING_PLAIN);
    for (unsigned i = 0; i < CLIENT_MAX; ++i) {
        struct client *client = &state->clients[i];
        if (client->fd >= 0 && client->subscribed)
            (void)send(client->fd, text, strlen(text),
                       MSG_NOSIGNAL | MSG_DONTWAIT);
    }
    json_object_put(message);
}

static int driver_stop(struct daemon_state *state, uint32_t domain,
                       char *error, size_t error_size, bool clear_leases) {
    struct json_object *request = new_driver_request("stop");
    json_object_object_add(request, "axes", json_object_new_int64(domain));
    struct json_object *reply = driver_request(state, request, error, error_size);
    json_object_put(request);
    if (!reply) return -1;
    if (!clear_leases) send_movement_ended(state, domain, reply);
    json_object_put(reply);
    clear_domain(state, domain, clear_leases);
    return 0;
}

static struct client *owner_for(struct daemon_state *state, uint32_t domain,
                                const struct client *except) {
    for (unsigned i = 0; i < CLIENT_MAX; ++i) {
        struct client *candidate = &state->clients[i];
        if (candidate != except && candidate->fd >= 0 && (candidate->leases & domain))
            return candidate;
    }
    return NULL;
}

static void send_lease_revoked(struct client *client, uint32_t domain,
                               const char *reason) {
    struct json_object *event = json_object_new_object();
    json_object_object_add(event, "version", json_object_new_int(1));
    json_object_object_add(event, "event", json_object_new_string("lease_revoked"));
    json_object_object_add(event, "axes", json_object_new_int64(domain));
    json_object_object_add(event, "reason", json_object_new_string(reason));
    json_object_object_add(event, "t_mono_ms",
                           json_object_new_int64((int64_t)now_ms()));
    const char *text = json_object_to_json_string_ext(event, JSON_C_TO_STRING_PLAIN);
    (void)send(client->fd, text, strlen(text), MSG_NOSIGNAL | MSG_DONTWAIT);
    json_object_put(event);
}

static void handle_capabilities(struct daemon_state *state, struct client *client,
                                const char *id) {
    struct json_object *response = json_object_new_object();
    struct json_object *axes = json_object_new_array();
    for (unsigned axis = 0; axis < MOTORS_AXIS_COUNT; ++axis) {
        if (state->caps.axes & MOTORS_AXIS_BIT(axis))
            json_object_array_add(axes, json_object_new_string(axis_name((enum motors_axis)axis)));
    }
    json_object_object_add(response, "version", json_object_new_int(1));
    json_object_object_add(response, "id", json_object_new_string(id));
    json_object_object_add(response, "ok", json_object_new_boolean(true));
    json_object_object_add(response, "driver", json_object_new_string(state->caps.name));
    json_object_object_add(response, "axes", axes);
    json_object_object_add(response, "raw", json_object_new_boolean(state->caps.supports_raw));
    json_object_object_add(response, "available", json_object_new_boolean(state->driver_ready));
    const char *text = json_object_to_json_string_ext(response, JSON_C_TO_STRING_PLAIN);
    (void)send(client->fd, text, strlen(text), MSG_NOSIGNAL);
    json_object_put(response);
}

static void handle_acquire(struct daemon_state *state, struct client *client,
                           struct json_object *request, const char *id) {
    enum motors_axis axis;
    const char *axis_text = json_string(request, "axis");
    enum client_role role = parse_role(json_string(request, "role"));
    if (parse_axis(axis_text, &axis) != 0 || role == ROLE_NONE ||
        !(state->caps.axes & MOTORS_AXIS_BIT(axis))) {
        send_response(client, id, false, "invalid acquire request");
        return;
    }
    unsigned lease_ms;
    if (json_uint(request, "lease_ms", &lease_ms) != 0 || lease_ms == 0) {
        send_response(client, id, false, "positive lease_ms required");
        return;
    }
    uint32_t domain = stop_domain(state, axis);
    struct client *owner = owner_for(state, domain, client);
    bool preempted = false;
    if (owner) {
        if (role <= owner->role) {
            send_response(client, id, false, "busy:%s", role_name(owner->role));
            return;
        }
        char error[160] = "";
        if (driver_stop(state, domain, error, sizeof(error), true) != 0) {
            send_response(client, id, false, "preempt stop failed:%s", error);
            return;
        }
        if (owner->subscribed) send_lease_revoked(owner, domain, "preempted");
        preempted = true;
    }
    client->role = role;
    client->leases |= domain;
    uint64_t deadline = now_ms() + lease_ms;
    for (unsigned i = 0; i < MOTORS_AXIS_COUNT; ++i) {
        if (domain & MOTORS_AXIS_BIT(i)) client->lease_deadline_ms[i] = deadline;
    }
    send_response(client, id, true, preempted ? "acquired:preempted" : "acquired");
}

static void handle_move(struct daemon_state *state, struct client *client,
                        struct json_object *request, const char *id) {
    enum motors_axis axis;
    const char *axis_text = json_string(request, "axis");
    const char *direction = json_string(request, "direction");
    if (parse_axis(axis_text, &axis) != 0 || !direction || !*direction) {
        send_response(client, id, false, "invalid move request");
        return;
    }
    uint32_t bit = MOTORS_AXIS_BIT(axis);
    if (!(client->leases & bit)) {
        send_response(client, id, false, "lease required");
        return;
    }
    unsigned duration_ms = 0;
    struct json_object *duration = NULL;
    if (json_object_object_get_ex(request, "duration_ms", &duration) &&
        json_uint(request, "duration_ms", &duration_ms) != 0) {
        send_response(client, id, false, "invalid duration_ms");
        return;
    }
    char error[160] = "";
    struct json_object *driver_message = new_driver_request("move");
    json_object_object_add(driver_message, "axis", json_object_new_int(axis));
    json_object_object_add(driver_message, "direction", json_object_new_string(direction));
    json_object_object_add(driver_message, "duration_ms", json_object_new_int64(duration_ms));
    struct json_object *driver_reply = driver_request(state, driver_message,
                                                       error, sizeof(error));
    json_object_put(driver_message);
    if (!driver_reply) {
        send_response(client, id, false, "%s", error[0] ? error : "driver move failed");
        return;
    }
    client->active |= bit;
    send_driver_success(client, id, "start_sent", driver_reply);
    json_object_put(driver_reply);
}

static void handle_stop(struct daemon_state *state, struct client *client,
                        struct json_object *request, const char *id) {
    const char *axis_text = json_string(request, "axis");
    uint32_t domain = state->caps.axes;
    if (axis_text && strcmp(axis_text, "all")) {
        enum motors_axis axis;
        if (parse_axis(axis_text, &axis) != 0) {
            send_response(client, id, false, "invalid stop axis");
            return;
        }
        domain = stop_domain(state, axis);
    }
    char error[160] = "";
    if (driver_stop(state, domain, error, sizeof(error), false) != 0) {
        send_response(client, id, false, "%s", error[0] ? error : "driver stop failed");
        return;
    }
    send_response(client, id, true, "stop_sent");
}

static void handle_raw(struct daemon_state *state, struct client *client,
                       struct json_object *request, const char *id) {
    if (!state->caps.supports_raw) {
        send_response(client, id, false, "raw unsupported");
        return;
    }
    for (unsigned i = 0; i < CLIENT_MAX; ++i) {
        if (&state->clients[i] != client && state->clients[i].fd >= 0 &&
            state->clients[i].leases) {
            send_response(client, id, false, "driver owned by another client");
            return;
        }
    }
    struct json_object *payload = NULL;
    if (!json_object_object_get_ex(request, "payload", &payload)) {
        send_response(client, id, false, "raw payload required");
        return;
    }
    char error[160] = "";
    struct json_object *driver_message = new_driver_request("raw");
    json_object_object_add(driver_message, "payload", json_object_get(payload));
    struct json_object *driver_reply = driver_request(state, driver_message,
                                                       error, sizeof(error));
    json_object_put(driver_message);
    if (!driver_reply) {
        send_response(client, id, false, "%s", error[0] ? error : "driver raw failed");
        return;
    }
    send_driver_success(client, id, "raw_sent", driver_reply);
    json_object_put(driver_reply);
}

static void handle_describe(struct daemon_state *state, struct client *client,
                            const char *id) {
    char error[160] = "";
    struct json_object *driver_message = new_driver_request("describe");
    struct json_object *driver_reply = driver_request(state, driver_message,
                                                       error, sizeof(error));
    json_object_put(driver_message);
    if (!driver_reply) {
        send_response(client, id, false, "%s",
                      error[0] ? error : "driver description failed");
        return;
    }
    forward_driver_reply(client, id, driver_reply);
    json_object_put(driver_reply);
}

static void handle_command(struct daemon_state *state, struct client *client,
                           struct json_object *request, const char *id) {
    const char *name = json_string(request, "name");
    if (!name || !*name) {
        send_response(client, id, false, "command name required");
        return;
    }
    for (unsigned i = 0; i < CLIENT_MAX; ++i) {
        if (&state->clients[i] != client && state->clients[i].fd >= 0 &&
            state->clients[i].leases) {
            send_response(client, id, false, "motor busy");
            return;
        }
    }
    struct json_object *driver_message = new_driver_request("command");
    json_object_object_add(driver_message, "name", json_object_new_string(name));
    struct json_object *value = NULL;
    if (json_object_object_get_ex(request, "value", &value))
        json_object_object_add(driver_message, "value", json_object_get(value));
    char error[160] = "";
    struct json_object *driver_reply = driver_request(state, driver_message,
                                                       error, sizeof(error));
    json_object_put(driver_message);
    if (!driver_reply) {
        send_response(client, id, false, "%s",
                      error[0] ? error : "driver command failed");
        return;
    }
    send_driver_success(client, id, "command_sent", driver_reply);
    json_object_put(driver_reply);
}

static void handle_request(struct daemon_state *state, struct client *client,
                           const char *message) {
    struct json_object *request = json_tokener_parse(message);
    if (!request || !json_object_is_type(request, json_type_object)) {
        send_response(client, "", false, "invalid JSON");
        if (request) json_object_put(request);
        return;
    }
    const char *id = json_string(request, "id");
    const char *operation = json_string(request, "op");
    unsigned version = 0;
    (void)json_uint(request, "version", &version);
    if (!id || !*id || !operation || version != 1) {
        send_response(client, id, false, version == 1 ? "invalid request" : "unsupported version");
    } else if (!strcmp(operation, "capabilities")) {
        handle_capabilities(state, client, id);
    } else if (!state->driver_ready) {
        send_response(client, id, false, "driver unavailable");
    } else if (!strcmp(operation, "acquire")) {
        handle_acquire(state, client, request, id);
    } else if (!strcmp(operation, "move")) {
        handle_move(state, client, request, id);
    } else if (!strcmp(operation, "stop")) {
        handle_stop(state, client, request, id);
    } else if (!strcmp(operation, "raw")) {
        handle_raw(state, client, request, id);
    } else if (!strcmp(operation, "describe")) {
        handle_describe(state, client, id);
    } else if (!strcmp(operation, "command")) {
        handle_command(state, client, request, id);
    } else if (!strcmp(operation, "subscribe")) {
        client->subscribed = true;
        send_response(client, id, true, "subscribed");
    } else if (!strcmp(operation, "release")) {
        uint32_t domain = client->leases;
        char error[160] = "";
        if (domain && driver_stop(state, domain, error, sizeof(error), true) != 0)
            send_response(client, id, false, "%s", error);
        else
            send_response(client, id, true, "released");
    } else {
        send_response(client, id, false, "unsupported operation");
    }
    json_object_put(request);
}

static void close_client(struct daemon_state *state, struct client *client) {
    if (client->fd < 0) return;
    uint32_t active_domains = 0;
    for (unsigned axis = 0; axis < MOTORS_AXIS_COUNT; ++axis) {
        if (client->active & MOTORS_AXIS_BIT(axis))
            active_domains |= stop_domain(state, (enum motors_axis)axis);
    }
    if (active_domains) {
        char error[160] = "";
        (void)driver_stop(state, active_domains, error, sizeof(error), true);
    }
    close(client->fd);
    memset(client, 0, sizeof(*client));
    client->fd = -1;
}

static void process_deadlines(struct daemon_state *state) {
    uint64_t now = now_ms();
    for (unsigned client_index = 0; client_index < CLIENT_MAX; ++client_index) {
        struct client *client = &state->clients[client_index];
        if (client->fd < 0) continue;
        for (unsigned axis = 0; axis < MOTORS_AXIS_COUNT; ++axis) {
            uint32_t bit = MOTORS_AXIS_BIT(axis);
            bool lease_expired = (client->leases & bit) && client->lease_deadline_ms[axis] &&
                                 now >= client->lease_deadline_ms[axis];
            if (!lease_expired) continue;
            char error[160] = "";
            uint32_t domain = stop_domain(state, (enum motors_axis)axis);
            if (driver_stop(state, domain, error, sizeof(error), true) == 0 &&
                client->subscribed)
                send_lease_revoked(client, domain, "expired");
        }
    }
}

static int start_driver(struct daemon_state *state, const char *path,
                        const char *configuration) {
    int sockets[2];
    if (socketpair(AF_UNIX, SOCK_SEQPACKET, 0, sockets) != 0) return -1;
    pid_t pid = fork();
    if (pid < 0) {
        close(sockets[0]);
        close(sockets[1]);
        return -1;
    }
    if (pid == 0) {
        close(sockets[0]);
        char fd_text[24];
        snprintf(fd_text, sizeof(fd_text), "%d", sockets[1]);
        execl(path, path, "--fd", fd_text, "--config", configuration, (char *)NULL);
        fprintf(stderr, "exec driver %s: %s\n", path, strerror(errno));
        _exit(127);
    }
    close(sockets[1]);
    state->driver_fd = sockets[0];
    state->driver_pid = pid;
    struct timeval timeout = {.tv_sec = 1, .tv_usec = 0};
    (void)setsockopt(state->driver_fd, SOL_SOCKET, SO_RCVTIMEO,
                     &timeout, sizeof(timeout));
    (void)setsockopt(state->driver_fd, SOL_SOCKET, SO_SNDTIMEO,
                     &timeout, sizeof(timeout));

    char error[160] = "";
    struct json_object *request = new_driver_request("capabilities");
    struct json_object *reply = driver_request(state, request, error, sizeof(error));
    json_object_put(request);
    if (!reply) {
        fprintf(stderr, "driver startup failed: %s\n", error);
        return -1;
    }
    struct json_object *name = NULL, *axes = NULL, *raw = NULL, *domains = NULL;
    if (!json_object_object_get_ex(reply, "name", &name) ||
        !json_object_object_get_ex(reply, "axes", &axes) ||
        !json_object_object_get_ex(reply, "raw", &raw) ||
        !json_object_object_get_ex(reply, "stop_domains", &domains) ||
        !json_object_is_type(domains, json_type_array) ||
        json_object_array_length(domains) != MOTORS_AXIS_COUNT) {
        fprintf(stderr, "invalid driver capabilities\n");
        json_object_put(reply);
        return -1;
    }
    memset(&state->caps, 0, sizeof(state->caps));
    snprintf(state->caps.name, sizeof(state->caps.name), "%s",
             json_object_get_string(name));
    state->caps.axes = (uint32_t)json_object_get_int64(axes);
    state->caps.supports_raw = json_object_get_boolean(raw);
    for (unsigned axis = 0; axis < MOTORS_AXIS_COUNT; ++axis)
        state->caps.stop_domain[axis] = (uint32_t)json_object_get_int64(
            json_object_array_get_idx(domains, axis));
    json_object_put(reply);
    state->driver_ready = true;
    return 0;
}

static void reap_driver(struct daemon_state *state) {
    state->driver_ready = false;
    if (state->driver_fd >= 0) {
        close(state->driver_fd);
        state->driver_fd = -1;
    }
    if (state->driver_pid > 0) {
        int status;
        if (waitpid(state->driver_pid, &status, WNOHANG) == 0) {
            kill(state->driver_pid, SIGTERM);
            (void)waitpid(state->driver_pid, &status, 0);
        }
        state->driver_pid = -1;
    }
}

static void recover_driver(struct daemon_state *state) {
    uint32_t lost_axes = state->caps.axes;
    fprintf(stderr, "motor driver connection lost; cancelling leases\n");
    for (unsigned i = 0; i < CLIENT_MAX; ++i) {
        struct client *client = &state->clients[i];
        uint32_t lost = client->leases & lost_axes;
        if (lost && client->subscribed)
            send_lease_revoked(client, lost, "driver_failure");
    }
    reap_driver(state);
    clear_domain(state, lost_axes, true);
    for (unsigned attempt = 0; attempt < DRIVER_RESTART_MAX; ++attempt) {
        if (start_driver(state, state->driver_path,
                         state->driver_configuration) == 0) {
            fprintf(stderr, "motor driver recovered on attempt %u\n",
                    attempt + 1);
            return;
        }
        reap_driver(state);
    }
    fprintf(stderr, "driver recovery failed after %u attempts\n",
            DRIVER_RESTART_MAX);
}

static int open_server(struct daemon_state *state) {
    state->server = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK, 0);
    if (state->server < 0) return -1;
    struct sockaddr_un address;
    memset(&address, 0, sizeof(address));
    address.sun_family = AF_UNIX;
    snprintf(address.sun_path, sizeof(address.sun_path), "%s", state->socket_path);
    unlink(state->socket_path);
    if (bind(state->server, (struct sockaddr *)&address, sizeof(address)) != 0) {
        fprintf(stderr, "bind %s: %s\n", state->socket_path, strerror(errno));
        return -1;
    }
    if (chmod(state->socket_path, 0660) != 0) {
        fprintf(stderr, "chmod %s: %s\n", state->socket_path, strerror(errno));
        return -1;
    }
    if (listen(state->server, CLIENT_MAX) != 0) {
        fprintf(stderr, "listen %s: %s\n", state->socket_path, strerror(errno));
        return -1;
    }
    return 0;
}

static void shutdown_daemon(struct daemon_state *state) {
    for (unsigned i = 0; i < CLIENT_MAX; ++i) close_client(state, &state->clients[i]);
    if (state->server >= 0) close(state->server);
    if (state->socket_path[0]) unlink(state->socket_path);
    if (state->driver_fd >= 0) {
        char error[160] = "";
        struct json_object *request = new_driver_request("shutdown");
        struct json_object *reply = driver_request(state, request, error, sizeof(error));
        json_object_put(request);
        if (reply) json_object_put(reply);
        close(state->driver_fd);
        state->driver_fd = -1;
    }
    if (state->driver_pid > 0) {
        (void)waitpid(state->driver_pid, NULL, 0);
        state->driver_pid = -1;
    }
}

static void usage(const char *name) {
    fprintf(stderr, "Usage: %s --socket PATH --driver FILE --driver-config FILE\n", name);
}

int main(int argc, char **argv) {
    const char *driver_path = NULL;
    const char *configuration = NULL;
    struct daemon_state state;
    memset(&state, 0, sizeof(state));
    state.server = -1;
    state.driver_fd = -1;
    state.driver_pid = -1;
    for (unsigned i = 0; i < CLIENT_MAX; ++i) state.clients[i].fd = -1;
    signal(SIGPIPE, SIG_IGN);

    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--socket") && i + 1 < argc) {
            const char *path = argv[++i];
            if (strlen(path) >= sizeof(state.socket_path)) {
                fprintf(stderr, "socket path is too long\n");
                return 2;
            }
            snprintf(state.socket_path, sizeof(state.socket_path), "%s", path);
        }
        else if (!strcmp(argv[i], "--driver") && i + 1 < argc)
            driver_path = argv[++i];
        else if (!strcmp(argv[i], "--driver-config") && i + 1 < argc)
            configuration = argv[++i];
        else {
            usage(argv[0]);
            return 2;
        }
    }
    if (!state.socket_path[0] || !driver_path || !configuration) {
        usage(argv[0]);
        return 2;
    }
    state.driver_path = driver_path;
    state.driver_configuration = configuration;
    if (start_driver(&state, driver_path, configuration) != 0) {
        shutdown_daemon(&state);
        return 1;
    }
    if (open_server(&state) != 0) {
        fprintf(stderr, "open server %s: %s\n", state.socket_path, strerror(errno));
        shutdown_daemon(&state);
        return 1;
    }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    fprintf(stderr, "motorsd: driver=%s socket=%s\n", state.caps.name, state.socket_path);

    while (!stop_requested) {
        struct pollfd descriptors[CLIENT_MAX + 2];
        descriptors[0].fd = state.server;
        descriptors[0].events = POLLIN;
        descriptors[1].fd = state.driver_fd;
        descriptors[1].events = POLLIN | POLLHUP | POLLERR;
        for (unsigned i = 0; i < CLIENT_MAX; ++i) {
            descriptors[i + 2].fd = state.clients[i].fd;
            descriptors[i + 2].events = POLLIN;
        }
        int result = poll(descriptors, CLIENT_MAX + 2, POLL_INTERVAL_MS);
        if (result < 0) {
            if (errno == EINTR) continue;
            break;
        }
        if (descriptors[0].revents & POLLIN) {
            int client_fd = accept4(state.server, NULL, NULL, SOCK_NONBLOCK);
            if (client_fd >= 0) {
                unsigned slot = CLIENT_MAX;
                for (unsigned i = 0; i < CLIENT_MAX; ++i) {
                    if (state.clients[i].fd < 0) {
                        slot = i;
                        break;
                    }
                }
                if (slot < CLIENT_MAX) state.clients[slot].fd = client_fd;
                else close(client_fd);
            }
        }
        if (descriptors[1].revents & (POLLHUP | POLLERR | POLLNVAL))
            recover_driver(&state);
        else if (descriptors[1].revents & POLLIN)
            receive_driver_event(&state);
        for (unsigned i = 0; i < CLIENT_MAX; ++i) {
            struct client *client = &state.clients[i];
            short events = descriptors[i + 2].revents;
            if (client->fd < 0 || !events) continue;
            if (events & (POLLHUP | POLLERR | POLLNVAL)) {
                close_client(&state, client);
                continue;
            }
            if (events & POLLIN) {
                char message[MESSAGE_MAX + 1];
                struct iovec vector = {.iov_base = message, .iov_len = MESSAGE_MAX};
                struct msghdr header;
                memset(&header, 0, sizeof(header));
                header.msg_iov = &vector;
                header.msg_iovlen = 1;
                ssize_t length = recvmsg(client->fd, &header, MSG_TRUNC);
                if (length <= 0) close_client(&state, client);
                else if ((size_t)length > MESSAGE_MAX)
                    send_response(client, "", false, "message too large");
                else {
                    message[length] = '\0';
                    handle_request(&state, client, message);
                }
            }
        }
        process_deadlines(&state);
    }
    shutdown_daemon(&state);
    return 0;
}
