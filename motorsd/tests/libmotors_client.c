#include "libmotors.h"

#include <stdio.h>
#include <string.h>

int main(int argc, char **argv) {
    if (argc != 2) return 2;
    char error[160] = "";
    struct motors_client *client = NULL;
    struct motors_capabilities capabilities;
    if (motors_open(&client, argv[1], error, sizeof(error)) != 0 ||
        motors_get_capabilities(client, &capabilities,
                                error, sizeof(error)) != 0 ||
        strcmp(capabilities.driver, "mock") ||
        capabilities.axes != (MOTORS_AXIS_MASK(MOTORS_PAN) |
                              MOTORS_AXIS_MASK(MOTORS_TILT) |
                              MOTORS_AXIS_MASK(MOTORS_ZOOM) |
                              MOTORS_AXIS_MASK(MOTORS_FOCUS)) ||
        !capabilities.raw || !capabilities.available ||
        motors_subscribe(client, error, sizeof(error)) != 0 ||
        motors_acquire(client, MOTORS_AF, MOTORS_FOCUS, 1000,
                       error, sizeof(error)) != 0 ||
        motors_move(client, MOTORS_FOCUS, MOTORS_NEAR, 20,
                    error, sizeof(error)) != 0) {
        fprintf(stderr, "%s\n", error);
        motors_close(client);
        return 1;
    }
    uint64_t completed = 0;
    if (motors_wait_movement(client, MOTORS_FOCUS, 1000, &completed,
                             error, sizeof(error)) != 0 || !completed) {
        fprintf(stderr, "%s\n", error);
        motors_close(client);
        return 1;
    }
    if (motors_release(client, error, sizeof(error)) != 0) {
        fprintf(stderr, "%s\n", error);
        motors_close(client);
        return 1;
    }
    motors_close(client);
    return 0;
}
