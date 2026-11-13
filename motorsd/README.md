# motorsd prototype

`motorsd` gives motor clients one service and one ownership model. It starts one
persistent driver process. Only that driver opens the hardware.

Clients can use:

- `libmotors`, the C API
- `motorsctl`, the command-line client
- the [public socket protocol](PROTOCOL.md)

The socket protocol is the service API. `libmotors` and `motorsctl` contain no
hardware protocol or motor policy.

The [driver protocol](DRIVER_PROTOCOL.md) connects `motorsd` to the selected
driver. The driver owns transport access, precise movement timing, and hardware
delivery rules.

## Build and test

Install a C compiler, `make`, `pkg-config`, and the `json-c` development files.
Then run:

```sh
make
make test
```

The tests cover leases, preemption, timed movement, events, connection loss,
raw access, telemetry, and driver recovery.

Use `make camera` to cross-compile the service and clients for an OpenIPC
camera.

## Start the service

This example starts the P035 Pelco-D driver:

```sh
build/motorsd \
  --socket /run/motorsd.sock \
  --driver build/pelcod_driver \
  --driver-config ../pelcodtui/cameras/p35-hieasy.conf
```

The selected driver reads and validates its configuration. `motorsd` does not
interpret hardware-specific values.

CAUTION: Check the UART path, baud rate, address, and speed values. Incorrect
values can move the wrong device.

The driver completes a safe startup before it accepts movement. Startup does
not resume an old movement or start homing.

## Use motorsctl

Get the driver state and available axes:

```sh
build/motorsctl --socket /run/motorsd.sock \
  '{"version":1,"id":"caps","op":"capabilities"}'
```

Send a 70 ms focus movement:

```sh
build/motorsctl --socket /run/motorsd.sock --wait-event \
  '{"version":1,"id":"events","op":"subscribe"}' \
  '{"version":1,"id":"lease","op":"acquire","role":"manual","axis":"focus","lease_ms":2000}' \
  '{"version":1,"id":"move","op":"move","axis":"focus","direction":"near","duration_ms":70}'
```

These requests use one connection. The lease belongs to that connection. The
driver controls the pulse and reports when its delivery sequence ends.

The available client roles are `automation`, `af`, `manual`, and `safety`.
Their priority follows that order. A client cannot choose a custom priority.

## Use libmotors

Include `include/libmotors.h`. Link `build/libmotors.a` and `json-c`.

The main movement sequence is:

```c
motors_open(&client, "/run/motorsd.sock", error, sizeof(error));
motors_subscribe(client, error, sizeof(error));
motors_acquire(client, MOTORS_MANUAL, MOTORS_FOCUS, 2000,
               error, sizeof(error));
motors_move(client, MOTORS_FOCUS, MOTORS_NEAR, 70,
            error, sizeof(error));
motors_wait_movement(client, MOTORS_FOCUS, 2000, NULL,
                     error, sizeof(error));
motors_close(client);
```

Production clients must check each return value. They must also check that the
driver reports the requested axis before movement.

## Telemetry

Drivers can publish hardware telemetry. The Pelco-XM driver converts a XiongMai
`X<ratio>` report into a `zoom_magnification` event.

AF2 reads this event through `libmotors`. It does not open the UART.

## Failure behavior

If a client disconnects, `motorsd` ends its lease and active movement.

If the driver disconnects, `motorsd` revokes affected leases and tries to
restart it. Clients must acquire new leases after recovery.

If recovery fails, the service remains available for status requests. It
rejects movement until the driver becomes available again.

## Raw access and device settings

Raw access supports driver development. The service rejects raw access while
another client owns a lease. The selected driver validates every payload.

A driver can also describe named controller settings, such as
`ir.brightness`. Clients display these settings without knowing the hardware
protocol. The driver validates and translates each command.

See the protocol documents for the complete message formats.

## Current limits

The architecture and the P035 path have camera tests. Pelco-XM telemetry has
PTY tests but still needs validation on an 85H50AI controller.

Firmware packaging, permissions, and wider hardware tests remain incomplete.
See the [prototype roadmap](TODO.md).
