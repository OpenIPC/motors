# motorsd driver protocol version 1

This protocol connects `motorsd` to one persistent driver process. It is a
private service interface. Motor clients use the public socket protocol in
[`PROTOCOL.md`](PROTOCOL.md).

## Process and connection

`motorsd` starts the selected driver with these arguments:

```sh
driver --fd NUMBER --config FILE
```

`NUMBER` is one end of a Unix `SOCK_SEQPACKET` connection. Each packet contains
one JSON object. The maximum packet size is 32768 bytes.

The driver loads and validates `FILE`. It also opens the hardware and completes
its safe startup procedure before it accepts requests.

The driver must not start movement or homing during startup. If the control
connection closes, the driver ends active movement and exits.

## Message rules

Each request contains `version`, `id`, and `op`:

```json
{"version":1,"id":"driver-1","op":"capabilities"}
```

Each response contains the same `id`, an `ok` value, and a state or error:

```json
{"version":1,"id":"driver-1","ok":true,"state":"sent","completed_mono_ms":502809}
```

An `ok` value of `true` means that the driver completed its delivery work. It
does not prove that the hardware moved.

The driver can send a lifecycle event before a response. `motorsd` processes
the event and continues to wait for the matching response.

The driver can also send hardware telemetry:

```json
{"version":1,"event":"telemetry","name":"zoom_magnification","value":3.2,"observed_mono_ms":502950}
```

Version 1 accepts `zoom_magnification` values from `1.0` through `1000.0`.
The timestamp records when the driver parsed the report from the hardware.

## Axis values

Requests use these numeric axis values:

| Axis | Value | Bit |
| --- | ---: | ---: |
| pan | 0 | 1 |
| tilt | 1 | 2 |
| zoom | 2 | 4 |
| focus | 3 | 8 |
| iris | 4 | 16 |

An `axes` field is a bit mask. A driver reports only the axes that its hardware
supports.

## Capabilities

`motorsd` sends this request after it starts the driver:

```json
{"version":1,"id":"driver-1","op":"capabilities"}
```

The driver response describes its name, axes, stop domains, and raw access:

```json
{"version":1,"id":"driver-1","ok":true,"name":"pelcod","axes":15,"stop_domains":[15,15,15,15,15],"raw":true,"completed_mono_ms":502700}
```

`stop_domains` contains one bit mask for each axis. The mask tells `motorsd`
which movements one stop operation can end together.

For example, the Pelco-D driver reports one shared domain for pan, tilt, zoom,
and focus. A driver with independent axes can report one bit for each axis.

## Movement

A continuous movement has a zero duration:

```json
{"version":1,"id":"driver-2","op":"move","axis":3,"direction":"near","duration_ms":0}
```

A timed movement has a positive duration:

```json
{"version":1,"id":"driver-3","op":"move","axis":3,"direction":"near","duration_ms":70}
```

The driver owns movement timing. For a timed movement, the driver ends the
movement and sends a `movement_ended` event:

```json
{"version":1,"event":"movement_ended","axes":8,"completed_mono_ms":502809}
```

The `completed_mono_ms` value uses `CLOCK_MONOTONIC`. It records completion of
the driver delivery sequence. It does not report physical motor feedback.

The direction depends on the axis:

| Axis | Directions |
| --- | --- |
| pan | `left`, `right` |
| tilt | `up`, `down` |
| zoom | `tele`, `wide` |
| focus | `near`, `far` |

## End movement

`motorsd` sends an axis mask:

```json
{"version":1,"id":"driver-4","op":"stop","axes":8}
```

The driver sends the hardware sequence that its device requires. The response
contains `completed_mono_ms` after that sequence ends.

## Raw access

The `raw` request contains a driver-specific payload:

```json
{"version":1,"id":"driver-5","op":"raw","payload":{"bytes":"ff010000000001"}}
```

The driver validates the payload before it accesses the hardware. A driver sets
`raw:false` in its capabilities if it does not support raw access.

## Device settings

A driver sets `settings:true` in its capabilities when it has optional device
controls. `describe` returns their menus, labels, value rules, and warnings.

The public name does not expose transport details. For example,
`ir.brightness` can map to a Pelco preset sequence or another controller
operation. A `command` request contains this name and its optional value. The
driver validates both before it accesses the hardware.

## Shutdown and failure

During a normal shutdown, `motorsd` sends this request:

```json
{"version":1,"id":"driver-6","op":"shutdown"}
```

The driver ends active movement, sends its response, and exits.

If a hardware write fails, the driver sends an error when possible. It then
closes the connection and exits. `motorsd` revokes affected leases before it
tries to restart the driver.
