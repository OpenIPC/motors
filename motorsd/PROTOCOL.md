# motorsd protocol version 1

`motorsd` listens on `/run/motorsd.sock`. The socket uses Unix
`SOCK_SEQPACKET`, so each JSON object is one complete message.

The socket protocol is the public service API. `libmotors` is a C wrapper for
the same requests. It contains no motor policy or hardware logic.

## Message rules

Each request contains `version`, `id`, and `op`:

```json
{"version":1,"id":"request-1","op":"capabilities"}
```

Each response has the same `id`. An `ok` value of `true` means that the service
accepted the operation. It does not prove that the motor moved.

An error response has `ok:false` and an `error` string. The client must not
reuse an `id` while its request is active.

The maximum message size is 32768 bytes. The service rejects a larger message.

## Capabilities

The `capabilities` operation needs no lease:

```json
{"version":1,"id":"caps","op":"capabilities"}
```

The response shows the selected driver, its axes, raw access, and availability:

```json
{"version":1,"id":"caps","ok":true,"driver":"pelcod","axes":["pan","tilt","zoom","focus"],"raw":true,"available":true}
```

C clients use `motors_get_capabilities()` to get the same information. The
function returns the driver name, an axis mask, raw access, and availability.

## Leases

A client acquires an axis before movement:

```json
{"version":1,"id":"lease","op":"acquire","role":"af","axis":"focus","lease_ms":5000}
```

Version 1 defines these roles, from lowest to highest priority:

- `automation`
- `af`
- `manual`
- `safety`

The service rejects an equal or lower priority request while another client
owns the resource. A higher priority request ends the old lease first.

The lease belongs to the client connection. A disconnection or lease timeout
ends its active movement. The `release` operation ends all leases for that
connection:

```json
{"version":1,"id":"release","op":"release"}
```

## Movement

A continuous movement has no duration:

```json
{"version":1,"id":"move","op":"move","axis":"focus","direction":"near"}
```

A timed movement includes `duration_ms`:

```json
{"version":1,"id":"move","op":"move","axis":"focus","direction":"near","duration_ms":70}
```

The driver controls the duration. The service does not use a client timer to
end a timed movement.

The `stop` operation ends one axis and any linked axes that the driver reports:

```json
{"version":1,"id":"stop","op":"stop","axis":"focus"}
```

The selected driver owns the hardware sequence for this operation. A normal
stop does not end the client lease.

## Events

A client sends `subscribe` once to receive lifecycle events:

```json
{"version":1,"id":"subscribe","op":"subscribe"}
```

The service can send an event between request responses. Clients must match
responses by `id` and process events separately.

A movement ends with this event:

```json
{"version":1,"event":"movement_ended","axes":8,"driver_completed_mono_ms":502809}
```

The axis value is a bit mask. Version 1 uses pan `1`, tilt `2`, zoom `4`, focus
`8`, and iris `16`.

`driver_completed_mono_ms` uses `CLOCK_MONOTONIC`. It records when the driver
completed its hardware delivery sequence. It does not prove physical movement.

The service sends this event after a timed movement or an accepted `stop`
request. Subscribers can use it to finish the request that started the movement.

The service reports a lost lease with this event:

```json
{"version":1,"event":"lease_revoked","axes":8,"reason":"preempted","t_mono_ms":502900}
```

The current reasons are `preempted`, `expired`, and `driver_failure`.

A driver can publish hardware telemetry without giving clients transport
access. The XiongMai driver publishes its reported zoom magnification:

```json
{"version":1,"event":"telemetry","name":"zoom_magnification","value":3.2,"driver_observed_mono_ms":502950}
```

`driver_observed_mono_ms` records when the driver parsed the hardware report.
`libmotors` clients read the last report with `motors_zoom_magnification()`.

## Raw access

The `raw` operation sends a driver-specific diagnostic payload:

```json
{"version":1,"id":"raw","op":"raw","payload":{"bytes":"ff010000000001"}}
```

The selected driver validates the payload. The service rejects raw access while
another client has a lease.

Raw access is for hardware development. Portable clients use movement
operations instead.

## Device settings

A driver can expose optional controller settings. Clients first request their
description:

```json
{"version":1,"id":"settings","op":"describe"}
```

The response contains labeled menus and controls. Each control has a stable
name, such as `ir.brightness`. It can also define a range, options, a warning,
or a confirmation requirement.

A client sends a named command after it validates the value against this
description:

```json
{"version":1,"id":"ir","op":"command","name":"ir.brightness","value":5}
```

The driver validates the command again and converts it to the device protocol.
Drivers that have no controller settings can reject `describe`.
