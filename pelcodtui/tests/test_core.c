#include "pelcodtui.h"

#include <assert.h>
#include <fcntl.h>
#include <pty.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

static void read_exact(int fd, uint8_t *out, size_t n) {
  size_t off = 0;
  while (off < n) {
    ssize_t r = read(fd, out + off, n - off);
    assert(r > 0);
    off += (size_t)r;
  }
}

static void clear_lock(void) {
  rmdir("/tmp/btzoom.lock");
}

static void set_entry(struct pct_profile *p, const char *section,
                      const char *key, const char *value) {
  for (size_t i = 0; i < p->count; i++) {
    if (!strcmp(p->entries[i].section, section) &&
        !strcmp(p->entries[i].key, key)) {
      snprintf(p->entries[i].value, sizeof(p->entries[i].value), "%s", value);
      return;
    }
  }
  assert(0 && "profile entry not found");
}

static void frame_tests(void) {
  uint8_t f[7];
  const uint8_t set[] = {0xff, 1, 0, 3, 0, 121, 125};
  const uint8_t call[] = {0xff, 1, 0, 7, 0, 120, 128};
  const uint8_t stop[] = {0xff, 1, 0, 0, 0, 0, 1};
  static const struct {
    const char *verb;
    unsigned speed;
    uint8_t frame[7];
  } motions[] = {
      {"left", 30, {0xff, 1, 0, 0x04, 30, 0, 35}},
      {"right", 30, {0xff, 1, 0, 0x02, 30, 0, 33}},
      {"up", 30, {0xff, 1, 0, 0x08, 0, 30, 39}},
      {"down", 30, {0xff, 1, 0, 0x10, 0, 30, 47}},
      {"tele", 0, {0xff, 1, 0, 0x20, 0, 0, 33}},
      {"wide", 0, {0xff, 1, 0, 0x40, 0, 0, 65}},
      {"near", 0, {0xff, 1, 0x01, 0, 0, 0, 2}},
      {"far", 0, {0xff, 1, 0, 0x80, 0, 0, 129}},
  };
  pct_frame_preset(1, "set", 121, f);
  assert(!memcmp(f, set, 7));
  pct_frame_preset(1, "call", 120, f);
  assert(!memcmp(f, call, 7));
  pct_frame_stop(1, f);
  assert(!memcmp(f, stop, 7));
  for (size_t i = 0; i < sizeof(motions) / sizeof(motions[0]); i++) {
    pct_frame_motion(1, motions[i].verb, motions[i].speed, f);
    assert(!memcmp(f, motions[i].frame, 7));
  }
}

static void uart_sequence_test(void) {
  int master, slave;
  char name[128], summary[128];
  assert(!openpty(&master, &slave, name, NULL, NULL));
  close(slave);
  struct pct_transport t = {.baud = 115200, .address = 1,
      .sequence_delay_ms = 1, .stop_repeat = 3, .stop_delay_ms = 1};
  snprintf(t.device, sizeof(t.device), "%s", name);
  clear_lock();
  assert(!pct_execute(&t, "preset_set 115; preset_set 7", summary,
                      sizeof(summary)));
  uint8_t got[14], first[7], second[7];
  pct_frame_preset(1, "set", 115, first);
  pct_frame_preset(1, "set", 7, second);
  read_exact(master, got, sizeof(got));
  assert(!memcmp(got, first, 7));
  assert(!memcmp(got + 7, second, 7));
  close(master);
}

static void command_rejection_test(void) {
  struct pct_transport t = {.baud = 115200, .address = 1,
                            .stop_repeat = 3, .dry_run = true};
  char summary[128];
  clear_lock();
  assert(pct_execute(&t, "preset_call 10 garbage", summary, sizeof(summary)));
  assert(pct_execute(&t, "preset_call 0", summary, sizeof(summary)));
  assert(pct_execute(&t, ";;;", summary, sizeof(summary)));
}

static void stale_lock_test(void) {
  int master, slave;
  char name[128], summary[128];
  assert(!openpty(&master, &slave, name, NULL, NULL));
  close(slave);
  struct pct_transport t = {.baud = 115200, .address = 1};
  snprintf(t.device, sizeof(t.device), "%s", name);
  clear_lock();
  assert(!mkdir("/tmp/btzoom.lock", 0700));
  struct timespec stale[2] = {{.tv_sec = time(NULL) - 61},
                              {.tv_sec = time(NULL) - 61}};
  assert(!utimensat(AT_FDCWD, "/tmp/btzoom.lock", stale, 0));
  assert(!pct_execute(&t, "preset_call 1", summary, sizeof(summary)));
  uint8_t got[7], expected[] = {0xff, 1, 0, 7, 0, 1, 9};
  read_exact(master, got, sizeof(got));
  assert(!memcmp(got, expected, sizeof(got)));
  close(master);
}

static void dry_run_lock_test(void) {
  struct pct_transport t = {.baud = 115200, .address = 1,
                            .stop_repeat = 3, .dry_run = true};
  char summary[128];
  clear_lock();
  assert(!mkdir("/tmp/btzoom.lock", 0700));
  assert(!pct_execute(&t, "preset_call 1", summary, sizeof(summary)));
  struct pct_motion m = {.fd = -1};
  assert(!pct_motion_start(&m, &t, "tele", 0, summary, sizeof(summary)));
  assert(!pct_motion_stop(&m, summary, sizeof(summary)));
  assert(!rmdir("/tmp/btzoom.lock"));
}

static void motion_test(void) {
  int master, slave;
  char name[128], summary[128];
  assert(!openpty(&master, &slave, name, NULL, NULL));
  close(slave);
  struct pct_transport t = {.baud = 115200, .address = 1,
      .stop_repeat = 3, .stop_delay_ms = 1};
  snprintf(t.device, sizeof(t.device), "%s", name);
  clear_lock();
  struct pct_motion m = {.fd = -1};
  assert(!pct_motion_start(&m, &t, "tele", 0, summary, sizeof(summary)));
  assert(access("/tmp/btzoom.lock/pelcodtui.pid", F_OK) < 0);
  assert(m.fd >= 0);
  struct termios settings;
  assert(!tcgetattr(m.fd, &settings));
  assert(!(settings.c_cflag & (PARENB | CSTOPB)));
  assert((settings.c_cflag & CSIZE) == CS8);
#ifdef CRTSCTS
  assert(!(settings.c_cflag & CRTSCTS));
#endif
  assert(!pct_motion_stop(&m, summary, sizeof(summary)));
  assert(m.fd == -1);
  uint8_t got[28], start[7], stop[7];
  pct_frame_motion(1, "tele", 0, start);
  pct_frame_stop(1, stop);
  read_exact(master, got, sizeof(got));
  assert(!memcmp(got, start, 7));
  for (int i = 1; i < 4; i++)
    assert(!memcmp(got + i * 7, stop, 7));
  close(master);
}

static void motion_switch_test(void) {
  int master, slave;
  char name[128], summary[128];
  assert(!openpty(&master, &slave, name, NULL, NULL));
  close(slave);
  struct pct_transport t = {.baud = 115200, .address = 1,
                            .stop_repeat = 3, .stop_delay_ms = 1};
  snprintf(t.device, sizeof(t.device), "%s", name);
  clear_lock();
  struct pct_motion m = {.fd = -1};
  assert(!pct_motion_start(&m, &t, "left", 30, summary, sizeof(summary)));
  assert(!pct_motion_stop(&m, summary, sizeof(summary)));
  assert(!pct_motion_start(&m, &t, "tele", 0, summary, sizeof(summary)));
  assert(!pct_motion_stop(&m, summary, sizeof(summary)));

  uint8_t got[56], left[7], tele[7], stop[7];
  pct_frame_motion(1, "left", 30, left);
  pct_frame_motion(1, "tele", 0, tele);
  pct_frame_stop(1, stop);
  read_exact(master, got, sizeof(got));
  assert(!memcmp(got, left, 7));
  for (int i = 1; i < 4; i++)
    assert(!memcmp(got + i * 7, stop, 7));
  assert(!memcmp(got + 28, tele, 7));
  for (int i = 5; i < 8; i++)
    assert(!memcmp(got + i * 7, stop, 7));
  close(master);
}

static void recovered_stop_test(void) {
  int master, slave;
  char name[128], summary[128];
  assert(!openpty(&master, &slave, name, NULL, NULL));
  close(slave);
  struct pct_transport t = {.baud = 115200, .address = 1,
                            .stop_repeat = 3, .stop_delay_ms = 1};
  snprintf(t.device, sizeof(t.device), "%s", name);
  clear_lock();
  struct pct_motion m = {.fd = -1};
  assert(!pct_motion_start(&m, &t, "tele", 0, summary, sizeof(summary)));
  assert(m.fd >= 0);
  close(m.fd);
  assert(!pct_motion_stop(&m, summary, sizeof(summary)));
  assert(!strcmp(summary, "stopped (2/3 STOP frames)"));
  assert(!m.active);
  assert(m.fd == -1);

  uint8_t got[21];
  const uint8_t tele[] = {0xff, 1, 0, 0x20, 0, 0, 33};
  const uint8_t stop[] = {0xff, 1, 0, 0, 0, 0, 1};
  read_exact(master, got, sizeof(got));
  assert(!memcmp(got, tele, 7));
  assert(!memcmp(got + 7, stop, 7));
  assert(!memcmp(got + 14, stop, 7));
  close(master);
}

static void failed_stop_test(void) {
  int master, slave;
  char name[128], summary[128];
  assert(!openpty(&master, &slave, name, NULL, NULL));
  close(slave);
  struct pct_transport t = {.baud = 115200, .address = 1,
                            .stop_repeat = 3};
  snprintf(t.device, sizeof(t.device), "%s", name);
  clear_lock();
  struct pct_motion m = {.fd = -1};
  assert(!pct_motion_start(&m, &t, "tele", 0, summary, sizeof(summary)));
  assert(m.fd >= 0);
  close(m.fd);
  m.fd = -1;
  snprintf(t.device, sizeof(t.device), "/missing-pelcodtui-uart");
  assert(pct_motion_stop(&m, summary, sizeof(summary)));
  assert(m.active);
  assert(m.fd == -1);

  int replacement_master, replacement_slave;
  assert(!openpty(&replacement_master, &replacement_slave, name, NULL, NULL));
  close(replacement_slave);
  snprintf(t.device, sizeof(t.device), "%s", name);
  assert(!pct_motion_stop(&m, summary, sizeof(summary)));
  assert(!m.active);
  assert(m.fd == -1);
  close(replacement_master);
  clear_lock();
  close(master);
}

static void execute_dry(const char *command) {
  struct pct_transport t = {.baud = 115200, .address = 1,
                            .stop_repeat = 3, .dry_run = true};
  char summary[128];
  clear_lock();
  assert(!pct_execute(&t, command, summary, sizeof(summary)));
}

static void all_profile_commands_test(const struct pct_profile *p) {
  char command[PCT_TEXT], err[128];
  for (size_t i = 0; i < p->section_count; i++) {
    const char *section = p->sections[i];
    if (!strncmp(section, "action.", 7)) {
      execute_dry(pct_get(p, section, "command"));
      continue;
    }
    if (strncmp(section, "setting.", 8))
      continue;

    const char *id = section + 8;
    const char *type = pct_get(p, section, "type");
    if (!strcmp(type, "number")) {
      assert(!pct_expand_setting(p, id, pct_get(p, section, "default"), command,
                                 sizeof(command), err, sizeof(err)));
      execute_dry(command);
      continue;
    }

    char options[PCT_TEXT];
    snprintf(options, sizeof(options), "%s", pct_get(p, section, "options"));
    char *save = NULL;
    for (char *option = strtok_r(options, ",", &save); option;
         option = strtok_r(NULL, ",", &save)) {
      while (*option == ' ')
        option++;
      assert(!pct_expand_setting(p, id, option, command, sizeof(command), err,
                                 sizeof(err)));
      execute_dry(command);
    }
  }
}

static void state_test(const char *id) {
  const char *path = "/tmp/pelcodtui-test-state.conf";
  char err[128];
  unlink(path);
  assert(!pct_state_write(path, id, "cruise_speed", "7",
                          "preset_set 115; preset_set 7", err, sizeof(err)));
  assert(!pct_state_write(path, id, "idle_delay", "5",
                          "preset_set 132; preset_set 5", err, sizeof(err)));
  assert(!pct_state_write(path, id, "cruise_speed", "8",
                          "preset_set 115; preset_set 8", err, sizeof(err)));
  struct pct_state_record updated, preserved;
  assert(!pct_state_read(path, id, "cruise_speed", &updated));
  assert(!strcmp(updated.value, "8"));
  assert(!strcmp(updated.command, "preset_set 115; preset_set 8"));
  assert(!pct_state_read(path, id, "idle_delay", &preserved));
  assert(!strcmp(preserved.value, "5"));
  assert(!pct_state_write(path, "app", "selection", "/missing-profile.conf",
                          "select", err, sizeof(err)));
  assert(!pct_state_remove(path, "app", "selection", err, sizeof(err)));
  assert(pct_state_read(path, "app", "selection", &updated));
  assert(!pct_state_read(path, id, "idle_delay", &preserved));
  unlink(path);
}

static void validation_test(const struct pct_profile *source) {
  struct pct_profile p;
  char err[256];
  p = *source;
  set_entry(&p, "uart", "address", "256");
  assert(pct_profile_validate(&p, err, sizeof(err)));
  p = *source;
  set_entry(&p, "uart", "baud", "7200");
  assert(pct_profile_validate(&p, err, sizeof(err)));
  p = *source;
  set_entry(&p, "setting.cruise_speed", "command",
            "preset_set 115 garbage");
  assert(pct_profile_validate(&p, err, sizeof(err)));
  p = *source;
  set_entry(&p, "setting.cruise_speed", "command", ";;;");
  assert(pct_profile_validate(&p, err, sizeof(err)));
  p = *source;
  set_entry(&p, "menu.cruise", "items", "missing_item");
  assert(pct_profile_validate(&p, err, sizeof(err)));
}

static void repeated_value_test(const struct pct_profile *source) {
  struct pct_profile p = *source;
  char command[PCT_TEXT], err[128];
  set_entry(&p, "setting.cruise_speed", "command",
            "preset_set $value; preset_call $value");
  assert(!pct_profile_validate(&p, err, sizeof(err)));
  assert(!pct_expand_setting(&p, "cruise_speed", "7", command,
                             sizeof(command), err, sizeof(err)));
  assert(!strcmp(command, "preset_set 7; preset_call 7"));
}

int main(int argc, char **argv) {
  assert(argc == 2);
  char err[256], cmd[512];
  struct pct_profile p;
  assert(!pct_profile_load(&p, argv[1], err, sizeof(err)));
  const char *id = pct_get(&p, "profile", "id");
  assert(id);
  assert(!pct_expand_setting(&p, "cruise_speed", "7", cmd, sizeof(cmd), err,
                             sizeof(err)));
  assert(!strcmp(cmd, "preset_set 115; preset_set 7"));
  assert(pct_expand_setting(&p, "cruise_speed", "11", cmd, sizeof(cmd), err,
                            sizeof(err)));
  if (!strcmp(id, "p35-hieasy")) {
    assert(!pct_expand_setting(&p, "cruise_dwell", "10s", cmd, sizeof(cmd),
                               err, sizeof(err)));
    assert(!strcmp(cmd, "preset_set 54"));
  } else if (!strcmp(id, "h07-hieasy")) {
    assert(!pct_expand_setting(&p, "af_trigger", "zoom", cmd, sizeof(cmd), err,
                               sizeof(err)));
    assert(!strcmp(cmd, "preset_set 250; preset_call 1"));
    assert(!pct_expand_setting(&p, "ir_brightness", "5", cmd, sizeof(cmd), err,
                               sizeof(err)));
    assert(!strcmp(cmd, "preset_set 122; preset_set 5"));
    assert(pct_expand_setting(&p, "ir_brightness", "0", cmd, sizeof(cmd), err,
                              sizeof(err)));
    assert(pct_expand_setting(&p, "ir_brightness", "11", cmd, sizeof(cmd), err,
                              sizeof(err)));
  } else {
    assert(!strcmp(id, "p6slite"));
  }
  validation_test(&p);
  repeated_value_test(&p);
  all_profile_commands_test(&p);
  frame_tests();
  uart_sequence_test();
  command_rejection_test();
  stale_lock_test();
  dry_run_lock_test();
  motion_test();
  motion_switch_test();
  recovered_stop_test();
  failed_stop_test();
  state_test(id);
  puts("all tests passed");
  return 0;
}
