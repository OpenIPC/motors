#include "libmotors.h"

#include <stdio.h>

int main(int argc, char **argv) {
    if (argc != 2) return 2;
    char error[160] = "";
    struct motors_client *client = NULL;
    if (motors_open(&client, argv[1], error, sizeof(error)) != 0 ||
        motors_acquire(client, MOTORS_AF, MOTORS_FOCUS, 5000,
                       error, sizeof(error)) != 0 ||
        motors_subscribe(client, error, sizeof(error)) != 0) {
        fprintf(stderr, "%s\n", error);
        motors_close(client);
        return 1;
    }
    puts("READY");
    fflush(stdout);
    int result = motors_poll(client, 2000, error, sizeof(error));
    bool revoked = motors_lease_revoked(client, MOTORS_FOCUS);
    motors_close(client);
    if (result != 1 || !revoked) {
        fprintf(stderr, "%s\n", error[0] ? error : "lease was not revoked");
        return 1;
    }
    return 0;
}
