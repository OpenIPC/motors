#include "libmotors.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void usage(const char *name) {
    fprintf(stderr,
        "Usage: %s [--socket PATH] [--speed N] [--role automation|af|manual] <action> [arg]\n"
        "Actions: up, down, left, right, tele, wide, near, far, open, close, stop, preset <num>, ircut <day|night>\n",
        name);
}

static int parse_role(const char *s, enum motors_client_role *out) {
    if (!strcmp(s, "automation")) { *out = MOTORS_AUTOMATION; return 0; }
    if (!strcmp(s, "af")) { *out = MOTORS_AF; return 0; }
    if (!strcmp(s, "manual")) { *out = MOTORS_MANUAL; return 0; }
    return -1;
}

int main(int argc, char **argv) {
    const char *socket_path = "/run/motorsd.sock";
    const char *action = NULL;
    const char *arg_val = NULL;
    unsigned duration_ms = 0;
    unsigned speed = 0;
    enum motors_client_role role = MOTORS_MANUAL;

    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--socket") && i + 1 < argc) {
            socket_path = argv[++i];
        } else if (!strcmp(argv[i], "--speed") && i + 1 < argc) {
            speed = (unsigned)atoi(argv[++i]);
        } else if (!strcmp(argv[i], "--role") && i + 1 < argc) {
            if (parse_role(argv[++i], &role) != 0) {
                fprintf(stderr, "unknown role: %s\n", argv[i]);
                return 2;
            }
        } else if (!action) {
            action = argv[i];
        } else if (!arg_val) {
            arg_val = argv[i];
            duration_ms = (unsigned)atoi(argv[i]);
        }
    }

    if (!action) {
        usage(argv[0]);
        return 2;
    }

    char error[256] = "";
    struct motors_client *client = NULL;
    if (motors_open(&client, socket_path, error, sizeof(error)) != 0) {
        fprintf(stderr, "motors_open (%s): %s\n", socket_path, error);
        return 1;
    }

    int rc = 0;
    if (!strcmp(action, "stop")) {
        rc = motors_stop_all(client, error, sizeof(error));
    } else if (!strcmp(action, "preset")) {
        const char *preset_val = arg_val ? arg_val : "1";
        rc = motors_command(client, "preset.call", preset_val, error, sizeof(error));
    } else if (!strcmp(action, "ircut")) {
        const char *ircut_val = arg_val ? arg_val : "day";
        rc = motors_command(client, "ircut", ircut_val, error, sizeof(error));
    } else {
        enum motors_client_axis axis;
        enum motors_direction dir;

        if (!strcmp(action, "left")) {
            axis = MOTORS_PAN; dir = MOTORS_LEFT;
        } else if (!strcmp(action, "right")) {
            axis = MOTORS_PAN; dir = MOTORS_RIGHT;
        } else if (!strcmp(action, "up")) {
            axis = MOTORS_TILT; dir = MOTORS_UP;
        } else if (!strcmp(action, "down")) {
            axis = MOTORS_TILT; dir = MOTORS_DOWN;
        } else if (!strcmp(action, "tele") || !strcmp(action, "zoomin")) {
            axis = MOTORS_ZOOM; dir = MOTORS_TELE;
        } else if (!strcmp(action, "wide") || !strcmp(action, "zoomout")) {
            axis = MOTORS_ZOOM; dir = MOTORS_WIDE;
        } else if (!strcmp(action, "near") || !strcmp(action, "focusnear")) {
            axis = MOTORS_FOCUS; dir = MOTORS_NEAR;
        } else if (!strcmp(action, "far") || !strcmp(action, "focusfar")) {
            axis = MOTORS_FOCUS; dir = MOTORS_FAR;
        } else if (!strcmp(action, "open") || !strcmp(action, "irisopen")) {
            axis = MOTORS_IRIS; dir = MOTORS_OPEN;
        } else if (!strcmp(action, "close") || !strcmp(action, "irisclose")) {
            axis = MOTORS_IRIS; dir = MOTORS_CLOSE;
        } else {
            fprintf(stderr, "unknown action: %s\n", action);
            motors_close(client);
            return 2;
        }

        if (!duration_ms) duration_ms = 1000;
        unsigned lease_ms = duration_ms + 1000;
        rc = motors_subscribe(client, error, sizeof(error));
        if (rc == 0) {
            rc = motors_acquire(client, role, axis, lease_ms, error, sizeof(error));
        }
        if (rc == 0) {
            rc = speed > 0
                ? motors_move_speed(client, axis, dir, speed, duration_ms, error, sizeof(error))
                : motors_move(client, axis, dir, duration_ms, error, sizeof(error));
            if (rc == 0 && duration_ms > 0) {
                rc = motors_wait_movement(client, axis, (int)duration_ms + 1000, NULL, error, sizeof(error));
            }
        }
    }

    if (rc != 0) {
        fprintf(stderr, "command failed: %s\n", error[0] ? error : "unknown error");
    }

    motors_close(client);
    return rc == 0 ? 0 : 1;
}
