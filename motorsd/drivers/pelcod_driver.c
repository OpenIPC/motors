#define _GNU_SOURCE

#include "motors_driver.h"

#include <errno.h>
#include <ctype.h>
#include <fcntl.h>
#include <json-c/json.h>
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
#define PROFILE_ENTRY_MAX 512

struct profile_entry {
    char section[96];
    char key[96];
    char value[512];
};

struct configuration {
    char device[128];
    unsigned baud;
    unsigned address;
    unsigned pan_speed;
    unsigned tilt_speed;
    unsigned stop_repeat;
    unsigned stop_delay_ms;
    unsigned sequence_delay_ms;
    bool has_profile;
    char profile_id[96];
    char profile_name[128];
    struct profile_entry entries[PROFILE_ENTRY_MAX];
    size_t entry_count;
};

struct driver_state {
    int control_fd;
    int uart_fd;
    struct configuration config;
    uint32_t active;
    uint64_t deadlines[MOTORS_AXIS_COUNT];
};

static uint64_t now_ms(void) {
    struct timespec value;
    clock_gettime(CLOCK_MONOTONIC, &value);
    return (uint64_t)value.tv_sec * 1000U + (uint64_t)value.tv_nsec / 1000000U;
}

static void sleep_ms(unsigned milliseconds) {
    struct timespec delay = {milliseconds / 1000,
                             (long)(milliseconds % 1000) * 1000000L};
    while (nanosleep(&delay, &delay) != 0 && errno == EINTR) {}
}

static speed_t baud_value(unsigned baud) {
    switch (baud) {
    case 1200: return B1200;
    case 2400: return B2400;
    case 4800: return B4800;
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

static const char *profile_get(const struct configuration *config,
                               const char *section, const char *key) {
    for (size_t i = 0; i < config->entry_count; ++i) {
        const struct profile_entry *entry = &config->entries[i];
        if (!strcmp(entry->section, section) && !strcmp(entry->key, key))
            return entry->value;
    }
    return NULL;
}

static int profile_add(struct configuration *config, const char *section,
                       const char *key, const char *value) {
    if (config->entry_count >= PROFILE_ENTRY_MAX) return -1;
    struct profile_entry *entry = &config->entries[config->entry_count++];
    if (strlen(section) >= sizeof(entry->section) ||
        strlen(key) >= sizeof(entry->key) ||
        strlen(value) >= sizeof(entry->value))
        return -1;
    snprintf(entry->section, sizeof(entry->section), "%s", section);
    snprintf(entry->key, sizeof(entry->key), "%s", key);
    snprintf(entry->value, sizeof(entry->value), "%s", value);
    return 0;
}

static int load_configuration(const char *path, struct configuration *config) {
    *config = (struct configuration){.baud = 2400, .address = 1,
                                    .pan_speed = 32, .tilt_speed = 32,
                                    .stop_repeat = 1, .stop_delay_ms = 2,
                                    .sequence_delay_ms = 150};
    FILE *file = fopen(path, "r");
    if (!file) return -1;
    char line[512];
    char section[96] = "";
    while (fgets(line, sizeof(line), file)) {
        char *end = strpbrk(line, "\r\n");
        if (end) *end = '\0';
        char *text = trim(line);
        if (!*text || *text == '#') continue;
        char *comment = strchr(text, '#');
        if (comment) *comment = '\0';
        text = trim(text);
        if (!*text) continue;
        if (*text == '[') {
            char *close = strchr(text, ']');
            if (!close || close[1]) {
                errno = EINVAL;
                goto fail;
            }
            *close = '\0';
            if (strlen(text + 1) >= sizeof(section)) {
                errno = EINVAL;
                goto fail;
            }
            snprintf(section, sizeof(section), "%s", text + 1);
            config->has_profile = true;
            continue;
        }
        char *separator = strchr(text, '=');
        if (!separator) {
            errno = EINVAL;
            goto fail;
        }
        *separator++ = '\0';
        char *key = trim(text);
        char *value = trim(separator);
        if (*section && profile_add(config, section, key, value) != 0) {
            errno = E2BIG;
            goto fail;
        }
        bool uart = !*section || !strcmp(section, "uart");
        bool driver = !*section || !strcmp(section, "driver");
        if (uart && !strcmp(key, "device")) {
            if (!*value || strlen(value) >= sizeof(config->device)) {
                errno = EINVAL;
                goto fail;
            }
            snprintf(config->device, sizeof(config->device), "%s", value);
        } else if (uart && !strcmp(key, "baud")) {
            if (parse_uint(value, 1, 115200, &config->baud)) goto invalid;
        } else if (uart && !strcmp(key, "address")) {
            if (parse_uint(value, 1, 255, &config->address)) goto invalid;
        } else if (driver && !strcmp(key, "pan_speed")) {
            if (parse_uint(value, 1, 63, &config->pan_speed)) goto invalid;
        } else if (driver && !strcmp(key, "tilt_speed")) {
            if (parse_uint(value, 1, 63, &config->tilt_speed)) goto invalid;
        } else if (driver && !strcmp(key, "stop_repeat")) {
            if (parse_uint(value, 1, 10, &config->stop_repeat)) goto invalid;
        } else if (driver && !strcmp(key, "stop_delay_ms")) {
            if (parse_uint(value, 0, 1000, &config->stop_delay_ms)) goto invalid;
        } else if (driver && !strcmp(key, "sequence_delay_ms")) {
            if (parse_uint(value, 0, 5000, &config->sequence_delay_ms)) goto invalid;
        } else if (!*section) {
            errno = EINVAL;
            goto fail;
        }
        continue;
invalid:
        errno = EINVAL;
        goto fail;
    }
    fclose(file);
    if (!config->device[0] || !baud_value(config->baud)) {
        errno = EINVAL;
        return -1;
    }
    if (config->has_profile) {
        const char *schema = profile_get(config, "profile", "schema_version");
        const char *id = profile_get(config, "profile", "id");
        const char *name = profile_get(config, "profile", "name");
        if (!schema || strcmp(schema, "1") || !id || !*id) {
            errno = EINVAL;
            return -1;
        }
        snprintf(config->profile_id, sizeof(config->profile_id), "%s", id);
        snprintf(config->profile_name, sizeof(config->profile_name), "%s",
                 name ? name : id);
    }
    return 0;
fail:
    fclose(file);
    return -1;
}

static int open_uart(const struct configuration *config) {
    speed_t speed = baud_value(config->baud);
    int fd = open(config->device, O_WRONLY | O_NOCTTY | O_CLOEXEC);
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

static void make_frame(unsigned address, uint8_t command1, uint8_t command2,
                       uint8_t data1, uint8_t data2, uint8_t output[7]) {
    output[0] = 0xff;
    output[1] = (uint8_t)address;
    output[2] = command1;
    output[3] = command2;
    output[4] = data1;
    output[5] = data2;
    output[6] = (uint8_t)(address + command1 + command2 + data1 + data2);
}

static int write_all(int fd, const uint8_t *data, size_t length) {
    size_t offset = 0;
    while (offset < length) {
        ssize_t count = write(fd, data + offset, length - offset);
        if (count < 0 && errno == EINTR) continue;
        if (count <= 0) return -1;
        offset += (size_t)count;
    }
    return tcdrain(fd);
}

static int reopen_uart(struct driver_state *state) {
    if (state->uart_fd >= 0) close(state->uart_fd);
    state->uart_fd = open_uart(&state->config);
    return state->uart_fd >= 0 ? 0 : -1;
}

static int send_stop(struct driver_state *state) {
    uint8_t frame[7];
    make_frame(state->config.address, 0, 0, 0, 0, frame);
    unsigned sent = 0;
    int saved_error = EIO;
    for (unsigned attempt = 0; attempt < state->config.stop_repeat; ++attempt) {
        if (state->uart_fd < 0 && reopen_uart(state) != 0) {
            saved_error = errno;
            if (attempt + 1 < state->config.stop_repeat)
                sleep_ms(state->config.stop_delay_ms);
            continue;
        }
        if (write_all(state->uart_fd, frame, sizeof(frame)) == 0)
            ++sent;
        else {
            saved_error = errno;
            close(state->uart_fd);
            state->uart_fd = -1;
        }
        if (attempt + 1 < state->config.stop_repeat)
            sleep_ms(state->config.stop_delay_ms);
    }
    if (!sent) {
        errno = saved_error;
        return -1;
    }
    state->active = 0;
    memset(state->deadlines, 0, sizeof(state->deadlines));
    return 0;
}

static int send_move(struct driver_state *state, unsigned axis,
                     const char *direction) {
    uint8_t command1 = 0, command2 = 0, data1 = 0, data2 = 0;
    switch (axis) {
    case MOTORS_AXIS_PAN:
        if (!strcmp(direction, "left")) command2 = 0x04;
        else if (!strcmp(direction, "right")) command2 = 0x02;
        else return -1;
        data1 = (uint8_t)state->config.pan_speed;
        break;
    case MOTORS_AXIS_TILT:
        if (!strcmp(direction, "up")) command2 = 0x08;
        else if (!strcmp(direction, "down")) command2 = 0x10;
        else return -1;
        data2 = (uint8_t)state->config.tilt_speed;
        break;
    case MOTORS_AXIS_ZOOM:
        if (!strcmp(direction, "tele")) command2 = 0x20;
        else if (!strcmp(direction, "wide")) command2 = 0x40;
        else return -1;
        break;
    case MOTORS_AXIS_FOCUS:
        if (!strcmp(direction, "near")) command1 = 0x01;
        else if (!strcmp(direction, "far")) command2 = 0x80;
        else return -1;
        break;
    default:
        return -1;
    }
    uint8_t frame[7];
    make_frame(state->config.address, command1, command2, data1, data2, frame);
    if (write_all(state->uart_fd, frame, sizeof(frame)) == 0) return 0;

    // A P035 UART descriptor can fail while the controller remains healthy.
    // Reopen it once so the next logical command does not require a daemon or
    // driver restart. Repeating a continuous movement frame is idempotent.
    int saved_error = errno;
    if (reopen_uart(state) == 0 &&
        write_all(state->uart_fd, frame, sizeof(frame)) == 0)
        return 0;
    if (errno == 0) errno = saved_error;
    return -1;
}

static bool valid_direction(unsigned axis, const char *direction) {
    if (!direction) return false;
    switch (axis) {
    case MOTORS_AXIS_PAN:
        return !strcmp(direction, "left") || !strcmp(direction, "right");
    case MOTORS_AXIS_TILT:
        return !strcmp(direction, "up") || !strcmp(direction, "down");
    case MOTORS_AXIS_ZOOM:
        return !strcmp(direction, "tele") || !strcmp(direction, "wide");
    case MOTORS_AXIS_FOCUS:
        return !strcmp(direction, "near") || !strcmp(direction, "far");
    default:
        return false;
    }
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

static int send_json(int fd, struct json_object *object) {
    const char *text = json_object_to_json_string_ext(object, JSON_C_TO_STRING_PLAIN);
    return send(fd, text, strlen(text), MSG_NOSIGNAL) < 0 ? -1 : 0;
}

static int send_reply(int fd, const char *id, bool ok, const char *detail) {
    struct json_object *reply = json_object_new_object();
    json_object_object_add(reply, "version", json_object_new_int(1));
    json_object_object_add(reply, "id", json_object_new_string(id ? id : ""));
    json_object_object_add(reply, "ok", json_object_new_boolean(ok));
    json_object_object_add(reply, "completed_mono_ms",
                           json_object_new_int64((int64_t)now_ms()));
    json_object_object_add(reply, ok ? "state" : "error",
                           json_object_new_string(detail));
    int result = send_json(fd, reply);
    json_object_put(reply);
    return result;
}

static int send_event(int fd, uint32_t axes) {
    struct json_object *event = json_object_new_object();
    json_object_object_add(event, "version", json_object_new_int(1));
    json_object_object_add(event, "event", json_object_new_string("movement_ended"));
    json_object_object_add(event, "axes", json_object_new_int64(axes));
    json_object_object_add(event, "completed_mono_ms",
                           json_object_new_int64((int64_t)now_ms()));
    int result = send_json(fd, event);
    json_object_put(event);
    return result;
}

static int send_capabilities(const struct driver_state *state, const char *id) {
    struct json_object *reply = json_object_new_object();
    struct json_object *domains = json_object_new_array();
    for (unsigned axis = 0; axis < MOTORS_AXIS_COUNT; ++axis)
        json_object_array_add(domains, json_object_new_int(0x0f));
    json_object_object_add(reply, "version", json_object_new_int(1));
    json_object_object_add(reply, "id", json_object_new_string(id));
    json_object_object_add(reply, "ok", json_object_new_boolean(true));
    json_object_object_add(reply, "name", json_object_new_string("pelcod"));
    json_object_object_add(reply, "axes", json_object_new_int(0x0f));
    json_object_object_add(reply, "stop_domains", domains);
    json_object_object_add(reply, "raw", json_object_new_boolean(true));
    json_object_object_add(reply, "settings",
                           json_object_new_boolean(state->config.has_profile));
    json_object_object_add(reply, "completed_mono_ms",
                           json_object_new_int64((int64_t)now_ms()));
    int result = send_json(state->control_fd, reply);
    json_object_put(reply);
    return result;
}

static void external_name(const char *id, char *name, size_t size) {
    size_t i = 0;
    for (; id[i] && i + 1 < size; ++i)
        name[i] = id[i] == '_' ? '.' : id[i];
    name[i] = '\0';
}

static bool profile_section(const char *section, const char *prefix,
                            const char **id) {
    size_t length = strlen(prefix);
    if (strncmp(section, prefix, length) || !section[length]) return false;
    *id = section + length;
    return true;
}

static void add_string_if(struct json_object *object, const char *json_key,
                          const struct configuration *config,
                          const char *section, const char *profile_key) {
    const char *value = profile_get(config, section, profile_key);
    if (value) json_object_object_add(object, json_key,
                                      json_object_new_string(value));
}

static void add_uint_if(struct json_object *object, const char *json_key,
                        const struct configuration *config,
                        const char *section, const char *profile_key) {
    const char *value = profile_get(config, section, profile_key);
    unsigned number = 0;
    if (value && parse_uint(value, 0, UINT32_MAX, &number) == 0)
        json_object_object_add(object, json_key, json_object_new_int64(number));
}

static struct json_object *csv_array(const char *text) {
    struct json_object *array = json_object_new_array();
    if (!text) return array;
    char copy[512];
    snprintf(copy, sizeof(copy), "%s", text);
    char *save = NULL;
    for (char *item = strtok_r(copy, ",", &save); item;
         item = strtok_r(NULL, ",", &save))
        json_object_array_add(array, json_object_new_string(trim(item)));
    return array;
}

static int send_description(const struct driver_state *state, const char *id) {
    struct json_object *reply = json_object_new_object();
    struct json_object *menus = json_object_new_array();
    struct json_object *controls = json_object_new_array();
    json_object_object_add(reply, "version", json_object_new_int(1));
    json_object_object_add(reply, "id", json_object_new_string(id));
    json_object_object_add(reply, "ok", json_object_new_boolean(true));
    json_object_object_add(reply, "profile",
                           json_object_new_string(state->config.profile_id));
    json_object_object_add(reply, "label",
                           json_object_new_string(state->config.profile_name));
    add_string_if(reply, "description", &state->config, "profile", "description");
    add_string_if(reply, "dangerous_call", &state->config, "profile",
                  "dangerous_call");
    add_string_if(reply, "dangerous_set", &state->config, "profile",
                  "dangerous_set");

    for (size_t i = 0; i < state->config.entry_count; ++i) {
        const char *section = state->config.entries[i].section;
        const char *item_id = NULL;
        bool setting = profile_section(section, "setting.", &item_id);
        bool action = profile_section(section, "action.", &item_id);
        bool menu = profile_section(section, "menu.", &item_id);
        if (strcmp(state->config.entries[i].key, "label")) continue;
        if (menu) {
            struct json_object *entry = json_object_new_object();
            json_object_object_add(entry, "id", json_object_new_string(item_id));
            add_string_if(entry, "label", &state->config, section, "label");
            json_object_object_add(entry, "items", csv_array(
                profile_get(&state->config, section, "items")));
            json_object_array_add(menus, entry);
        } else if (setting || action) {
            char name[128];
            external_name(item_id, name, sizeof(name));
            struct json_object *entry = json_object_new_object();
            json_object_object_add(entry, "name", json_object_new_string(name));
            json_object_object_add(entry, "id", json_object_new_string(item_id));
            json_object_object_add(entry, "type", json_object_new_string(
                action ? "action" : profile_get(&state->config, section, "type")));
            add_string_if(entry, "label", &state->config, section, "label");
            add_string_if(entry, "description", &state->config, section,
                          "description");
            add_string_if(entry, "default", &state->config, section, "default");
            add_string_if(entry, "unit", &state->config, section, "unit");
            add_string_if(entry, "warning", &state->config, section, "warning");
            add_uint_if(entry, "min", &state->config, section, "min");
            add_uint_if(entry, "max", &state->config, section, "max");
            add_uint_if(entry, "step", &state->config, section, "step");
            const char *confirm = profile_get(&state->config, section, "confirm");
            if (confirm) json_object_object_add(entry, "confirm",
                json_object_new_boolean(!strcmp(confirm, "true")));
            if (setting && strcmp(profile_get(&state->config, section, "type"),
                                  "number")) {
                struct json_object *options = json_object_new_array();
                const char *list = profile_get(&state->config, section, "options");
                char copy[512];
                snprintf(copy, sizeof(copy), "%s", list ? list : "");
                char *save = NULL;
                for (char *value = strtok_r(copy, ",", &save); value;
                     value = strtok_r(NULL, ",", &save)) {
                    value = trim(value);
                    char key[192];
                    snprintf(key, sizeof(key), "option.%s.label", value);
                    struct json_object *option = json_object_new_object();
                    json_object_object_add(option, "value",
                                           json_object_new_string(value));
                    json_object_object_add(option, "label", json_object_new_string(
                        profile_get(&state->config, section, key) ?: value));
                    json_object_array_add(options, option);
                }
                json_object_object_add(entry, "options", options);
            }
            json_object_array_add(controls, entry);
        }
    }
    json_object_object_add(reply, "menus", menus);
    json_object_object_add(reply, "controls", controls);
    int result = send_json(state->control_fd, reply);
    json_object_put(reply);
    return result;
}

static bool csv_has(const char *list, const char *wanted) {
    if (!list || !wanted) return false;
    char copy[512];
    snprintf(copy, sizeof(copy), "%s", list);
    char *save = NULL;
    for (char *item = strtok_r(copy, ",", &save); item;
         item = strtok_r(NULL, ",", &save)) {
        if (!strcmp(trim(item), wanted)) return true;
    }
    return false;
}

static const char *find_control_section(const struct configuration *config,
                                        const char *name, bool *is_action) {
    static __thread char found[96];
    for (size_t i = 0; i < config->entry_count; ++i) {
        const char *id = NULL;
        bool action = profile_section(config->entries[i].section, "action.", &id);
        bool setting = profile_section(config->entries[i].section, "setting.", &id);
        if (!action && !setting) continue;
        char candidate[128];
        external_name(id, candidate, sizeof(candidate));
        if (!strcmp(candidate, name)) {
            snprintf(found, sizeof(found), "%s", config->entries[i].section);
            *is_action = action;
            return found;
        }
    }
    return NULL;
}

static int expand_command(const struct configuration *config, const char *name,
                          const char *value, char *output, size_t output_size) {
    if (!strcmp(name, "preset.set") || !strcmp(name, "preset.call") ||
        !strcmp(name, "preset.clear")) {
        unsigned preset = 0;
        if (!value || parse_uint(value, 1, 255, &preset) != 0) return -1;
        const char *operation = !strcmp(name, "preset.set") ? "preset_set" :
            !strcmp(name, "preset.call") ? "preset_call" : "preset_clear";
        return snprintf(output, output_size, "%s %u", operation, preset) <
            (int)output_size ? 0 : -1;
    }
    bool action = false;
    const char *section = find_control_section(config, name, &action);
    if (!section) return -1;
    const char *command = NULL;
    if (action) {
        if (value) return -1;
        command = profile_get(config, section, "command");
    } else {
        const char *type = profile_get(config, section, "type");
        if (!type || !value) return -1;
        if (!strcmp(type, "number")) {
            unsigned number, minimum, maximum;
            if (parse_uint(value, 0, UINT32_MAX, &number) != 0 ||
                parse_uint(profile_get(config, section, "min"), 0,
                           UINT32_MAX, &minimum) != 0 ||
                parse_uint(profile_get(config, section, "max"), 0,
                           UINT32_MAX, &maximum) != 0 ||
                number < minimum || number > maximum)
                return -1;
            command = profile_get(config, section, "command");
        } else {
            if (!csv_has(profile_get(config, section, "options"), value)) return -1;
            char key[192];
            snprintf(key, sizeof(key), "option.%s.command", value);
            command = profile_get(config, section, key);
        }
    }
    if (!command) return -1;
    const char *token = "$value";
    size_t used = 0;
    while (*command) {
        const char *at = strstr(command, token);
        size_t part = at ? (size_t)(at - command) : strlen(command);
        if (used + part + 1 > output_size) return -1;
        memcpy(output + used, command, part);
        used += part;
        if (!at) break;
        size_t value_length = strlen(value);
        if (used + value_length + 1 > output_size) return -1;
        memcpy(output + used, value, value_length);
        used += value_length;
        command = at + strlen(token);
    }
    output[used] = '\0';
    return 0;
}

static int execute_sequence(struct driver_state *state, const char *sequence) {
    char copy[512];
    snprintf(copy, sizeof(copy), "%s", sequence);
    char *save = NULL;
    for (char *part = strtok_r(copy, ";", &save); part;
         part = strtok_r(NULL, ";", &save)) {
        char operation[32], argument[32], extra;
        if (sscanf(trim(part), "%31s %31s %c", operation, argument, &extra) != 2)
            return -1;
        char *end = NULL;
        errno = 0;
        unsigned long preset = strtoul(argument, &end, 10);
        if (errno || !end || *end || preset < 1 || preset > 255) return -1;
        uint8_t command2;
        if (!strcmp(operation, "preset_set")) command2 = 0x03;
        else if (!strcmp(operation, "preset_call")) command2 = 0x07;
        else if (!strcmp(operation, "preset_clear")) command2 = 0x05;
        else return -1;
        uint8_t frame[7];
        make_frame(state->config.address, 0, command2, 0,
                   (uint8_t)preset, frame);
        if (write_all(state->uart_fd, frame, sizeof(frame)) != 0) {
            if (reopen_uart(state) != 0 ||
                write_all(state->uart_fd, frame, sizeof(frame)) != 0)
                return -1;
        }
        if (save && *save) sleep_ms(state->config.sequence_delay_ms);
    }
    return 0;
}

static int handle_raw(struct driver_state *state, struct json_object *request) {
    struct json_object *payload = NULL;
    const char *hex = NULL;
    if (json_object_object_get_ex(request, "payload", &payload) &&
        json_object_is_type(payload, json_type_object))
        hex = json_string(payload, "bytes");
    if (!hex || !*hex || strlen(hex) % 2 || strlen(hex) > 512) return -1;
    uint8_t bytes[256];
    size_t count = strlen(hex) / 2;
    for (size_t i = 0; i < count; ++i) {
        char pair[3] = {hex[i * 2], hex[i * 2 + 1], '\0'};
        char *end = NULL;
        errno = 0;
        unsigned long value = strtoul(pair, &end, 16);
        if (errno || !end || *end) return -1;
        bytes[i] = (uint8_t)value;
    }
    return write_all(state->uart_fd, bytes, count);
}

static int handle_request(struct driver_state *state, struct json_object *request) {
    const char *id = json_string(request, "id");
    const char *operation = json_string(request, "op");
    if (!id || !operation) return send_reply(state->control_fd, id, false, "invalid request");
    if (!strcmp(operation, "capabilities"))
        return send_capabilities(state, id);
    if (!strcmp(operation, "describe")) {
        if (!state->config.has_profile)
            return send_reply(state->control_fd, id, false,
                              "settings unsupported");
        return send_description(state, id);
    }
    if (!strcmp(operation, "command")) {
        const char *name = json_string(request, "name");
        struct json_object *value_object = NULL;
        const char *value = NULL;
        if (json_object_object_get_ex(request, "value", &value_object)) {
            if (json_object_is_type(value_object, json_type_string))
                value = json_object_get_string(value_object);
            else if (json_object_is_type(value_object, json_type_int))
                value = json_object_to_json_string_ext(value_object,
                                                        JSON_C_TO_STRING_PLAIN);
            else
                return send_reply(state->control_fd, id, false,
                                  "invalid command value");
        }
        char sequence[512];
        if (!name || expand_command(&state->config, name, value, sequence,
                                    sizeof(sequence)) != 0)
            return send_reply(state->control_fd, id, false,
                              "invalid named command");
        if (execute_sequence(state, sequence) != 0)
            return send_reply(state->control_fd, id, false,
                              "command write failed");
        return send_reply(state->control_fd, id, true, "sent");
    }
    if (!strcmp(operation, "move")) {
        unsigned axis = json_uint(request, "axis", MOTORS_AXIS_COUNT);
        const char *direction = json_string(request, "direction");
        unsigned duration = json_uint(request, "duration_ms", 0);
        if (!valid_direction(axis, direction))
            return send_reply(state->control_fd, id, false,
                              "invalid movement");
        if (send_move(state, axis, direction) != 0) {
            char error[160];
            snprintf(error, sizeof(error), "transport write failed: %s",
                     strerror(errno));
            (void)send_reply(state->control_fd, id, false, error);
            return -1;
        }
        state->active = 0x0f;
        memset(state->deadlines, 0, sizeof(state->deadlines));
        if (duration) state->deadlines[axis] = now_ms() + duration;
        return send_reply(state->control_fd, id, true, "sent");
    }
    if (!strcmp(operation, "stop")) {
        unsigned axes = json_uint(request, "axes", 0);
        if (!(axes & 0x0f))
            return send_reply(state->control_fd, id, false,
                              "invalid stop axes");
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
    if (!strcmp(operation, "shutdown")) {
        int result = state->active ? send_stop(state) : 0;
        (void)send_reply(state->control_fd, id, result == 0,
                         result == 0 ? "stopped" : "stop failed");
        return 1;
    }
    return send_reply(state->control_fd, id, false, "unsupported operation");
}

static int next_timeout(const struct driver_state *state) {
    uint64_t now = now_ms(), nearest = 0;
    for (unsigned axis = 0; axis < MOTORS_AXIS_COUNT; ++axis) {
        uint64_t deadline = state->deadlines[axis];
        if (deadline && (!nearest || deadline < nearest)) nearest = deadline;
    }
    if (!nearest) return -1;
    if (nearest <= now) return 0;
    uint64_t delay = nearest - now;
    return delay > INT32_MAX ? INT32_MAX : (int)delay;
}

static int finish_movement(struct driver_state *state) {
    uint64_t now = now_ms();
    uint32_t ended = 0;
    for (unsigned axis = 0; axis < MOTORS_AXIS_COUNT; ++axis) {
        if (state->deadlines[axis] && state->deadlines[axis] <= now)
            ended |= MOTORS_AXIS_BIT(axis);
    }
    if (!ended) return 0;
    if (send_stop(state) != 0) return -1;
    return send_event(state->control_fd, 0x0f);
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
    static struct driver_state state;
    state = (struct driver_state){.control_fd = control_fd, .uart_fd = -1};
    if (control_fd < 0 || !configuration ||
        load_configuration(configuration, &state.config) != 0) {
        fprintf(stderr, "Pelco-D configuration failed: %s\n", strerror(errno));
        return 2;
    }
    state.uart_fd = open_uart(&state.config);
    if (state.uart_fd < 0) {
        fprintf(stderr, "open %s: %s\n", state.config.device, strerror(errno));
        return 2;
    }

    /* A neutral frame makes startup safe without starting motion or homing. */
    if (send_stop(&state) != 0) {
        fprintf(stderr, "safe startup failed: %s\n", strerror(errno));
        close(state.uart_fd);
        return 2;
    }

    int exit_code = 0;
    for (;;) {
        struct pollfd descriptor = {.fd = control_fd, .events = POLLIN};
        int result = poll(&descriptor, 1, next_timeout(&state));
        if (result < 0) {
            if (errno == EINTR) continue;
            exit_code = 1;
            break;
        }
        if (finish_movement(&state) != 0) {
            exit_code = 1;
            break;
        }
        if (!result) continue;
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
