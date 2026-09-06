#ifndef OPENIPC_LIBMOTORS_H
#define OPENIPC_LIBMOTORS_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

struct motors_client;

enum motors_client_axis {
    MOTORS_PAN = 0,
    MOTORS_TILT,
    MOTORS_ZOOM,
    MOTORS_FOCUS,
    MOTORS_IRIS,
};

#define MOTORS_AXIS_MASK(axis) (1U << (unsigned)(axis))

struct motors_capabilities {
    char driver[64];
    uint32_t axes;
    bool raw;
    bool available;
};

enum motors_client_role {
    MOTORS_AUTOMATION = 0,
    MOTORS_AF,
    MOTORS_MANUAL,
};

enum motors_direction {
    MOTORS_LEFT = 0,
    MOTORS_RIGHT,
    MOTORS_UP,
    MOTORS_DOWN,
    MOTORS_TELE,
    MOTORS_WIDE,
    MOTORS_NEAR,
    MOTORS_FAR,
};

int motors_open(struct motors_client **client, const char *socket_path,
                char *error, size_t error_size);
void motors_close(struct motors_client *client);

int motors_request(struct motors_client *client, const char *request,
                   char *response, size_t response_size,
                   char *error, size_t error_size);
int motors_get_capabilities(struct motors_client *client,
                            struct motors_capabilities *capabilities,
                            char *error, size_t error_size);
int motors_describe(struct motors_client *client,
                    char *response, size_t response_size,
                    char *error, size_t error_size);
int motors_command(struct motors_client *client, const char *name,
                   const char *value, char *error, size_t error_size);
int motors_acquire(struct motors_client *client, enum motors_client_role role,
                   enum motors_client_axis axis, unsigned lease_ms,
                   char *error, size_t error_size);
int motors_move(struct motors_client *client, enum motors_client_axis axis,
                enum motors_direction direction, unsigned duration_ms,
                char *error, size_t error_size);
int motors_stop(struct motors_client *client, enum motors_client_axis axis,
                char *error, size_t error_size);
int motors_stop_all(struct motors_client *client,
                    char *error, size_t error_size);
int motors_release(struct motors_client *client, char *error, size_t error_size);
int motors_subscribe(struct motors_client *client, char *error, size_t error_size);
int motors_poll(struct motors_client *client, int timeout_ms,
                char *error, size_t error_size);
bool motors_lease_revoked(const struct motors_client *client,
                          enum motors_client_axis axis);
int motors_wait_movement(struct motors_client *client,
                         enum motors_client_axis axis, int timeout_ms,
                         uint64_t *completed_mono_ms,
                         char *error, size_t error_size);

#endif
