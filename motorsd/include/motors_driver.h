#ifndef OPENIPC_MOTORS_DRIVER_H
#define OPENIPC_MOTORS_DRIVER_H

#include <stdint.h>

#define MOTORS_DRIVER_PROTOCOL_VERSION 1U

enum motors_axis {
    MOTORS_AXIS_PAN = 0,
    MOTORS_AXIS_TILT,
    MOTORS_AXIS_ZOOM,
    MOTORS_AXIS_FOCUS,
    MOTORS_AXIS_IRIS,
    MOTORS_AXIS_COUNT
};

#define MOTORS_AXIS_BIT(axis) (1U << (unsigned)(axis))

struct motors_driver_caps {
    char name[64];
    uint32_t axes;
    uint32_t stop_domain[MOTORS_AXIS_COUNT];
    int supports_raw;
};

#endif
