#include "pelcodtui.h"
#include "libmotors.h"

#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#define PCT_LOCK_DIR "/tmp/btzoom.lock"
#define PCT_STALE_LOCK_SECONDS 60

static int service_axis(const char *verb, enum motors_client_axis *axis,
                        enum motors_direction *direction) {
  if (!strcmp(verb, "left")) { *axis = MOTORS_PAN; *direction = MOTORS_LEFT; }
  else if (!strcmp(verb, "right")) { *axis = MOTORS_PAN; *direction = MOTORS_RIGHT; }
  else if (!strcmp(verb, "up")) { *axis = MOTORS_TILT; *direction = MOTORS_UP; }
  else if (!strcmp(verb, "down")) { *axis = MOTORS_TILT; *direction = MOTORS_DOWN; }
  else if (!strcmp(verb, "tele")) { *axis = MOTORS_ZOOM; *direction = MOTORS_TELE; }
  else if (!strcmp(verb, "wide")) { *axis = MOTORS_ZOOM; *direction = MOTORS_WIDE; }
  else if (!strcmp(verb, "near")) { *axis = MOTORS_FOCUS; *direction = MOTORS_NEAR; }
  else if (!strcmp(verb, "far")) { *axis = MOTORS_FOCUS; *direction = MOTORS_FAR; }
  else return -1;
  return 0;
}

int pct_named_command(const struct pct_transport *transport, const char *name,
                      const char *value, char *summary, size_t n) {
  if (!transport->use_motorsd) {
    snprintf(summary, n, "named command needs motorsd");
    return -1;
  }
  if (transport->dry_run) {
    snprintf(summary, n, "%s=%s (dry run)", name, value ? value : "action");
    return 0;
  }
  struct motors_client *client = NULL;
  char error[160] = "";
  char wire_name[128];
  size_t i = 0;
  for (; name[i] && i + 1 < sizeof(wire_name); i++)
    wire_name[i] = name[i] == '_' ? '.' : name[i];
  wire_name[i] = '\0';
  if (motors_open(&client, transport->socket_path, error, sizeof(error)) != 0 ||
      motors_command(client, wire_name, value, error, sizeof(error)) != 0) {
    snprintf(summary, n, "motorsd: %s", error[0] ? error : "command failed");
    motors_close(client);
    return -1;
  }
  motors_close(client);
  snprintf(summary, n, "sent %s", name);
  return 0;
}

int pct_preset(const struct pct_transport *transport, const char *operation,
               unsigned preset, char *summary, size_t n) {
  if (transport->use_motorsd) {
    char name[32], value[4];
    snprintf(name, sizeof(name), "preset.%s", operation);
    snprintf(value, sizeof(value), "%u", preset);
    return pct_named_command(transport, name, value, summary, n);
  }
  char command[64];
  snprintf(command, sizeof(command), "preset_%s %u", operation, preset);
  return pct_execute(transport, command, summary, n);
}

void pct_sleep_ms(unsigned ms) {
  struct timespec delay = {ms / 1000, (long)(ms % 1000) * 1000000L};
  while (nanosleep(&delay, &delay) && errno == EINTR)
    ;
}

static void frame(uint8_t address, uint8_t command1, uint8_t command2,
                  uint8_t data1, uint8_t data2, uint8_t out[7]) {
  out[0] = 0xff;
  out[1] = address;
  out[2] = command1;
  out[3] = command2;
  out[4] = data1;
  out[5] = data2;
  out[6] = (uint8_t)(address + command1 + command2 + data1 + data2);
}

void pct_frame_preset(uint8_t address, const char *op, unsigned preset,
                      uint8_t out[7]) {
  uint8_t command2 = !strcmp(op, "set") ? 3 : !strcmp(op, "clear") ? 5 : 7;
  frame(address, 0, command2, 0, (uint8_t)preset, out);
}

void pct_frame_stop(uint8_t address, uint8_t out[7]) {
  frame(address, 0, 0, 0, 0, out);
}

void pct_frame_motion(uint8_t address, const char *verb, unsigned speed,
                      uint8_t out[7]) {
  uint8_t command1 = 0, command2 = 0, data1 = 0, data2 = 0;

  if (!strcmp(verb, "left")) {
    command2 = 0x04;
    data1 = (uint8_t)speed;
  } else if (!strcmp(verb, "right")) {
    command2 = 0x02;
    data1 = (uint8_t)speed;
  } else if (!strcmp(verb, "up")) {
    command2 = 0x08;
    data2 = (uint8_t)speed;
  } else if (!strcmp(verb, "down")) {
    command2 = 0x10;
    data2 = (uint8_t)speed;
  } else if (!strcmp(verb, "tele")) {
    command2 = 0x20;
  } else if (!strcmp(verb, "wide")) {
    command2 = 0x40;
  } else if (!strcmp(verb, "near")) {
    command1 = 0x01;
  } else if (!strcmp(verb, "far")) {
    command2 = 0x80;
  }

  frame(address, command1, command2, data1, data2, out);
}

static speed_t baud_value(unsigned baud) {
  switch (baud) {
  case 1200:
    return B1200;
  case 2400:
    return B2400;
  case 4800:
    return B4800;
  case 9600:
    return B9600;
  case 19200:
    return B19200;
  case 38400:
    return B38400;
  case 57600:
    return B57600;
  case 115200:
    return B115200;
  default:
    return 0;
  }
}

static int lock_port(void) {
  for (int i = 0; i < 40; i++) {
    if (!mkdir(PCT_LOCK_DIR, 0700))
      return 0;
    if (errno != EEXIST)
      return -1;

    /* btzoom shares this lock and requires the directory to remain empty. */
    struct stat status;
    time_t now = time(NULL);
    if (!stat(PCT_LOCK_DIR, &status) && now != (time_t)-1 &&
        now - status.st_mtime > PCT_STALE_LOCK_SECONDS &&
        !rmdir(PCT_LOCK_DIR))
      continue;
    pct_sleep_ms(50);
  }
  errno = EBUSY;
  return -1;
}

static void unlock_port(void) { rmdir(PCT_LOCK_DIR); }

static int open_uart(const struct pct_transport *transport, int *fd,
                     char *summary, size_t n) {
  *fd = -1;
  speed_t speed = baud_value(transport->baud);
  if (!speed) {
    snprintf(summary, n, "unsupported baud");
    return -1;
  }
  if (transport->dry_run)
    return 0;

  *fd = open(transport->device, O_WRONLY | O_NOCTTY);
  if (*fd < 0) {
    snprintf(summary, n, "open %s: %s", transport->device, strerror(errno));
    return -1;
  }

  struct termios settings;
  if (tcgetattr(*fd, &settings)) {
    snprintf(summary, n, "termios: %s", strerror(errno));
    goto fail;
  }
  cfmakeraw(&settings);
  cfsetispeed(&settings, speed);
  cfsetospeed(&settings, speed);
  settings.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
  settings.c_cflag |= CLOCAL | CREAD;
  settings.c_cflag |= CS8;
#ifdef CRTSCTS
  settings.c_cflag &= ~CRTSCTS;
#endif
  if (tcsetattr(*fd, TCSANOW, &settings)) {
    snprintf(summary, n, "termios: %s", strerror(errno));
    goto fail;
  }
  return 0;

fail:
  close(*fd);
  *fd = -1;
  return -1;
}

static int write_frame(int fd, const uint8_t frame_bytes[7]) {
  size_t written = 0;
  while (written < 7) {
    ssize_t result = write(fd, frame_bytes + written, 7 - written);
    if (result < 0 && errno == EINTR)
      continue;
    if (result <= 0)
      return -1;
    written += (size_t)result;
  }
  return tcdrain(fd);
}

int pct_parse_command(const char *text, bool allow_value,
                      struct pct_command *command) {
  char op[32], argument[32], extra;
  int fields = sscanf(text ? text : "", " %31s %31s %c", op, argument, &extra);

  memset(command, 0, sizeof(*command));
  if (fields == 1 && !strcmp(op, "stop")) {
    command->op = PCT_COMMAND_STOP;
    return 0;
  }
  if (fields != 2)
    return -1;

  if (!strcmp(op, "preset_set"))
    command->op = PCT_COMMAND_PRESET_SET;
  else if (!strcmp(op, "preset_call"))
    command->op = PCT_COMMAND_PRESET_CALL;
  else if (!strcmp(op, "preset_clear"))
    command->op = PCT_COMMAND_PRESET_CLEAR;
  else
    return -1;

  if (allow_value && !strcmp(argument, "$value")) {
    command->value_placeholder = true;
    return 0;
  }

  char *end = NULL;
  errno = 0;
  unsigned long value = strtoul(argument, &end, 10);
  if (errno || !end || *end || value < 1 || value > 255)
    return -1;
  command->value = (unsigned)value;
  return 0;
}

static const char *preset_op(enum pct_command_op op) {
  if (op == PCT_COMMAND_PRESET_SET)
    return "set";
  if (op == PCT_COMMAND_PRESET_CLEAR)
    return "clear";
  return "call";
}

int pct_execute(const struct pct_transport *transport, const char *sequence,
                char *summary, size_t n) {
  bool locked = false;
  if (!transport->dry_run && lock_port()) {
    snprintf(summary, n, "PTZ port busy: %s", strerror(errno));
    return -1;
  }
  locked = !transport->dry_run;

  int fd = -1, result = -1, frames = 0;
  if (open_uart(transport, &fd, summary, n))
    goto done;

  char buffer[PCT_TEXT];
  snprintf(buffer, sizeof(buffer), "%s", sequence);
  char *save = NULL;
  bool parsed = false;
  for (char *part = strtok_r(buffer, ";", &save); part;
       part = strtok_r(NULL, ";", &save)) {
    parsed = true;
    struct pct_command command;
    uint8_t frame_bytes[7];
    if (pct_parse_command(part, false, &command)) {
      snprintf(summary, n, "invalid command: %s", part);
      goto done;
    }

    if (command.op == PCT_COMMAND_STOP) {
      for (unsigned i = 0; i < transport->stop_repeat; i++) {
        pct_frame_stop(transport->address, frame_bytes);
        if (!transport->dry_run && write_frame(fd, frame_bytes))
          goto io_error;
        frames++;
        if (i + 1 < transport->stop_repeat)
          pct_sleep_ms(transport->stop_delay_ms);
      }
    } else {
      pct_frame_preset(transport->address, preset_op(command.op), command.value,
                       frame_bytes);
      if (!transport->dry_run && write_frame(fd, frame_bytes))
        goto io_error;
      frames++;
    }
    if (save && *save)
      pct_sleep_ms(transport->sequence_delay_ms);
  }
  if (!parsed) {
    snprintf(summary, n, "invalid empty command sequence");
    goto done;
  }

  snprintf(summary, n, "sent %d frame%s%s", frames, frames == 1 ? "" : "s",
           transport->dry_run ? " (dry run)" : "");
  result = 0;
  goto done;

io_error:
  snprintf(summary, n, "UART write failed: %s", strerror(errno));
done:
  if (fd >= 0)
    close(fd);
  if (locked)
    unlock_port();
  return result;
}

int pct_motion_start(struct pct_motion *motion,
                     const struct pct_transport *transport, const char *verb,
                     unsigned speed, char *summary, size_t n) {
  if (motion->active) {
    snprintf(summary, n, "motion already active");
    return -1;
  }
  if (transport->use_motorsd && !transport->dry_run) {
    enum motors_client_axis axis;
    enum motors_direction direction;
    char error[160] = "";
    struct motors_client *client = NULL;
    if (service_axis(verb, &axis, &direction) != 0 ||
        motors_open(&client, transport->socket_path, error, sizeof(error)) != 0 ||
        motors_acquire(client, MOTORS_MANUAL, axis, 6000, error,
                       sizeof(error)) != 0 ||
        motors_move(client, axis, direction, 0, error, sizeof(error)) != 0) {
      snprintf(summary, n, "motorsd: %s", error[0] ? error : "invalid movement");
      motors_close(client);
      return -1;
    }
    motion->transport = transport;
    motion->service_client = client;
    motion->service_axis = (unsigned)axis;
    motion->active = true;
    snprintf(summary, n, "%s active", verb);
    return 0;
  }
  if (!transport->dry_run && lock_port()) {
    snprintf(summary, n, "PTZ port busy");
    return -1;
  }

  motion->transport = transport;
  if (open_uart(transport, &motion->fd, summary, n))
    goto fail;

  uint8_t frame_bytes[7];
  pct_frame_motion(transport->address, verb, speed, frame_bytes);
  if (!transport->dry_run && write_frame(motion->fd, frame_bytes)) {
    snprintf(summary, n, "UART write failed: %s", strerror(errno));
    goto fail;
  }
  motion->active = true;
  snprintf(summary, n, "%s active%s", verb,
           transport->dry_run ? " (dry run)" : "");
  return 0;

fail:
  if (motion->fd >= 0)
    close(motion->fd);
  motion->fd = -1;
  motion->transport = NULL;
  if (!transport->dry_run)
    unlock_port();
  return -1;
}

int pct_motion_stop(struct pct_motion *motion, char *summary, size_t n) {
  if (!motion->active)
    return 0;

  const struct pct_transport *transport = motion->transport;
  if (transport->use_motorsd && !transport->dry_run) {
    struct motors_client *client = motion->service_client;
    char error[160] = "";
    int result = motors_stop(client,
        (enum motors_client_axis)motion->service_axis, error, sizeof(error));
    if (result == 0)
      (void)motors_release(client, error, sizeof(error));
    motors_close(client);
    motion->service_client = NULL;
    motion->active = false;
    motion->transport = NULL;
    snprintf(summary, n, "%s", result == 0 ? "stopped" :
             (error[0] ? error : "motorsd stop failed"));
    return result;
  }
  uint8_t frame_bytes[7];
  unsigned sent = 0;
  char last_error[128] = "UART STOP failed";
  for (unsigned i = 0; i < transport->stop_repeat; i++) {
    if (motion->fd < 0 &&
        open_uart(transport, &motion->fd, last_error, sizeof(last_error))) {
      if (i + 1 < transport->stop_repeat)
        pct_sleep_ms(transport->stop_delay_ms);
      continue;
    }
    pct_frame_stop(transport->address, frame_bytes);
    if (!transport->dry_run && write_frame(motion->fd, frame_bytes)) {
      snprintf(last_error, sizeof(last_error), "UART STOP failed: %s",
               strerror(errno));
      close(motion->fd);
      motion->fd = -1;
    } else
      sent++;
    if (i + 1 < transport->stop_repeat)
      pct_sleep_ms(transport->stop_delay_ms);
  }

  if (motion->fd >= 0)
    close(motion->fd);
  motion->fd = -1;
  if (!sent) {
    snprintf(summary, n, "%s", last_error);
    return -1;
  }
  motion->active = false;
  motion->transport = NULL;
  if (!transport->dry_run)
    unlock_port();
  if (sent == transport->stop_repeat)
    snprintf(summary, n, "stopped");
  else
    snprintf(summary, n, "stopped (%u/%u STOP frames)", sent,
             transport->stop_repeat);
  return 0;
}

int pct_move(const struct pct_transport *transport, const char *verb,
             unsigned speed, unsigned duration, char *summary, size_t n) {
  if (!strcmp(verb, "stop") && transport->use_motorsd && !transport->dry_run) {
    struct motors_client *client = NULL;
    char error[160] = "";
    if (motors_open(&client, transport->socket_path, error, sizeof(error)) != 0 ||
        motors_stop_all(client, error, sizeof(error)) != 0) {
      snprintf(summary, n, "motorsd: %s", error[0] ? error : "stop failed");
      motors_close(client);
      return -1;
    }
    motors_close(client);
    snprintf(summary, n, "stopped");
    return 0;
  }
  if (!strcmp(verb, "stop"))
    return pct_execute(transport, "stop", summary, n);

  struct pct_motion motion = {.fd = -1};
  if (pct_motion_start(&motion, transport, verb, speed, summary, n))
    return -1;
  pct_sleep_ms(duration);
  if (pct_motion_stop(&motion, summary, n))
    return -1;
  snprintf(summary, n, "%s %ums%s", verb, duration,
           transport->dry_run ? " (dry run)" : "");
  return 0;
}
