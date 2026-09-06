# motorsd prototype

`motorsd` gives all motor clients one service and one ownership model. It
starts one persistent driver process. Only that driver opens the hardware.

Clients use one of these interfaces:

- `libmotors` provides C functions.
- `motorsctl` sends socket requests from a shell.
- Other clients can use the [socket protocol](PROTOCOL.md) directly.

The [driver protocol](DRIVER_PROTOCOL.md) defines the private connection between
`motorsd` and a driver. The [prototype roadmap](TODO.md) records the remaining
work and hardware validation.

The socket protocol is the public API. `libmotors` and `motorsctl` contain no
motor protocol or hardware logic.

## Build and test

Install a C compiler, `make`, `pkg-config`, and the development files for
`json-c`. Then run:

```sh
make
make test
```

The test uses a mock driver. It covers leases, priority, timed movement,
events, client loss, raw access, driver loss, and driver restart.

## Start the service

The daemon needs a public socket, a driver program, and a driver configuration
file.

Start the Pelco-D driver for the P035 controller:

```sh
build/motorsd \
  --socket /run/motorsd.sock \
  --driver build/pelcod_driver \
  --driver-config ../pelcodtui/cameras/p35-hieasy.conf
```

The driver opens the UART and sends its safe startup sequence. It does not
start movement or homing.

CAUTION: Check the UART path, baud rate, address, and movement speeds before
you use a hardware driver. Incorrect values can move the wrong device.

The P035 profile contains its transport and driver values:

```ini
[uart]
device=/dev/ttyAMA0
baud=115200
address=1

[driver]
stop_repeat=3
stop_delay_ms=2
```

The selected driver reads and checks this file. `motorsd` does not interpret
driver configuration.

The same file describes optional controller menus and commands. Clients can
request this description without knowing the hardware protocol.

## Use motorsctl

Get the driver state and supported axes:

```sh
build/motorsctl --socket /run/motorsd.sock \
  '{"version":1,"id":"caps","op":"capabilities"}'
```

Send a 70 ms focus movement and wait for its lifecycle event:

```sh
build/motorsctl --socket /run/motorsd.sock --wait-event \
  '{"version":1,"id":"events","op":"subscribe"}' \
  '{"version":1,"id":"lease","op":"acquire","role":"manual","axis":"focus","lease_ms":2000}' \
  '{"version":1,"id":"move","op":"move","axis":"focus","direction":"near","duration_ms":70}'
```

All three requests use one connection. The lease belongs to that connection.
The driver controls the pulse time and reports when its delivery sequence ends.

End focus movement from another authorized client:

```sh
build/motorsctl --socket /run/motorsd.sock \
  '{"version":1,"id":"stop","op":"stop","axis":"focus"}'
```

Use `pan`, `tilt`, `zoom`, `focus`, or `iris` as the axis. The driver reports
which axes exist. Use only a direction that is valid for the selected axis.

The client roles have this priority:

1. `automation`
2. `af`
3. `manual`
4. `safety`

A higher role can preempt a lower role. A client cannot select a custom
priority.

## Use libmotors

Include [`include/libmotors.h`](include/libmotors.h) and link
`build/libmotors.a` with `json-c`.

This example gets capabilities and sends one timed focus movement:

```c
#include <stdio.h>

#include <libmotors.h>

int main(void)
{
    struct motors_client *client = NULL;
    struct motors_capabilities caps;
    char error[160] = "";

    if (motors_open(&client, "/run/motorsd.sock",
                    error, sizeof(error)) != 0)
        goto fail;

    if (motors_get_capabilities(client, &caps,
                                error, sizeof(error)) != 0)
        goto fail;
    if (!caps.available || !(caps.axes & MOTORS_AXIS_MASK(MOTORS_FOCUS))) {
        snprintf(error, sizeof(error), "focus motor is not available");
        goto fail;
    }

    if (motors_subscribe(client, error, sizeof(error)) != 0)
        goto fail;
    if (motors_acquire(client, MOTORS_MANUAL, MOTORS_FOCUS, 2000,
                       error, sizeof(error)) != 0)
        goto fail;
    if (motors_move(client, MOTORS_FOCUS, MOTORS_NEAR, 70,
                    error, sizeof(error)) != 0)
        goto fail;
    if (motors_wait_movement(client, MOTORS_FOCUS, 2000, NULL,
                             error, sizeof(error)) != 0)
        goto fail;

    motors_close(client);
    return 0;

fail:
    fprintf(stderr, "motor error: %s\n", error);
    motors_close(client);
    return 1;
}
```

Compile the example from this directory:

```sh
cc -Iinclude -o example example.c build/libmotors.a $(pkg-config --libs json-c)
```

`motors_close()` ends the connection. The service ends its lease and any active
movement.

## Driver and service loss

If a driver connection closes, `motorsd` cancels its commands and leases. It
then makes up to three restart attempts. Clients must request new leases after
a successful restart.

If all restart attempts fail, `motorsd` remains available. A capability request
then reports `"available":false`, and motor requests return `driver unavailable`.

If a client connection closes, `motorsd` ends movement that belongs to that
client. If the private control connection closes, the driver tries to end its
active movement and exits.

## Raw access

Raw access helps driver development. The payload belongs to the selected
driver. This Pelco-D example sends one complete frame:

```sh
build/motorsctl --socket /run/motorsd.sock \
  '{"version":1,"id":"raw","op":"raw","payload":{"bytes":"ff010000000001"}}'
```

The service rejects raw access while another client has a lease. Normal clients
use movement operations instead.

## Current limits

This code is a host-tested architecture prototype. It is not ready for normal
installation. See the [prototype roadmap](TODO.md) for the remaining work.
