# motorsd prototype roadmap

This checklist records the work that remains after the architecture prototype.
It does not define the final implementation order.

## Implemented in the prototype

- [x] Public versioned socket protocol
- [x] First-party `libmotors` C API
- [x] `motorsctl` command-line client
- [x] Persistent driver process
- [x] Connection-bound leases and fixed client roles
- [x] Manual preemption of AF ownership
- [x] Continuous and timed movement
- [x] Driver-owned movement timing and hardware delivery rules
- [x] Lifecycle events with monotonic driver timestamps
- [x] Driver failure detection and restart attempts
- [x] Development raw access with lease protection
- [x] Mock-driver architecture tests
- [x] Pelco-D prototype driver and P035 configuration
- [x] Driver-described menus and named device commands
- [x] `pelcodtui` client through the public service API

## Required before normal installation

- [ ] Add an OpenIPC package.
- [ ] Add an init script and service configuration.
- [ ] Define socket ownership and permissions.
- [ ] Define which clients can use the `safety` role and add the typed C wrapper if required.
- [ ] Load driver selection and essential hardware configuration from OpenIPC ENV.
- [ ] Define protocol compatibility rules for future versions.
- [ ] Add driver-described settings to the WebUI.

## Hardware validation

- [ ] Measure AF latency and jitter through `AF -> motorsd -> driver`.
- [ ] Measure timed movement on supported camera hardware.
- [ ] Validate press, hold, release, and timeout behavior from the WebUI.
- [ ] Validate manual preemption during each AF phase.
- [ ] Validate client disconnection during active movement.
- [ ] Validate driver failure and recovery during active movement.
- [ ] Validate service shutdown and camera restart behavior.
- [ ] Record the meaning and accuracy of each driver timestamp.

## Later scope

- [ ] Decide whether one camera needs more than one driver instance.
- [ ] Decide which axes belong in the first common interface.
- [ ] Define portable absolute and relative position operations if hardware tests support them.
- [ ] Add more hardware drivers after the driver protocol is stable.
