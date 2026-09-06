#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#define MESSAGE_MAX 32768

static void usage(const char *name) {
    fprintf(stderr, "Usage: %s --socket PATH [--wait-event] JSON [JSON ...]\n", name);
}

int main(int argc, char **argv) {
    const char *socket_path = NULL;
    int first_request = -1;
    int wait_event = 0;
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--socket") && i + 1 < argc)
            socket_path = argv[++i];
        else if (!strcmp(argv[i], "--wait-event"))
            wait_event = 1;
        else if (first_request < 0) {
            first_request = i;
            break;
        }
        else {
            usage(argv[0]);
            return 2;
        }
    }
    if (!socket_path || first_request < 0) {
        usage(argv[0]);
        return 2;
    }
    if (strlen(socket_path) >= sizeof(((struct sockaddr_un *)0)->sun_path)) {
        fprintf(stderr, "socket path is too long\n");
        return 2;
    }

    int fd = socket(AF_UNIX, SOCK_SEQPACKET, 0);
    if (fd < 0) {
        perror("socket");
        return 1;
    }
    struct sockaddr_un address;
    memset(&address, 0, sizeof(address));
    address.sun_family = AF_UNIX;
    snprintf(address.sun_path, sizeof(address.sun_path), "%s", socket_path);
    if (connect(fd, (struct sockaddr *)&address, sizeof(address)) != 0) {
        fprintf(stderr, "connect %s: %s\n", socket_path, strerror(errno));
        close(fd);
        return 1;
    }
    int exit_code = 0;
    for (int i = first_request; i < argc; ++i) {
        const char *request = argv[i];
        size_t length = strlen(request);
        if (length >= MESSAGE_MAX ||
            send(fd, request, length, 0) != (ssize_t)length) {
            fprintf(stderr, "send request: %s\n", strerror(errno));
            exit_code = 1;
            break;
        }
        char response[MESSAGE_MAX + 1];
        ssize_t received = recv(fd, response, MESSAGE_MAX, 0);
        if (received <= 0) {
            fprintf(stderr, "receive response: %s\n", strerror(errno));
            exit_code = 1;
            break;
        }
        response[received] = '\0';
        puts(response);
        if (!strstr(response, "\"ok\":true")) exit_code = 1;
    }
    if (!exit_code && wait_event) {
        char event[MESSAGE_MAX + 1];
        ssize_t received = recv(fd, event, MESSAGE_MAX, 0);
        if (received <= 0) {
            fprintf(stderr, "receive event: %s\n", strerror(errno));
            exit_code = 1;
        } else {
            event[received] = '\0';
            puts(event);
        }
    }
    close(fd);
    return exit_code;
}
