#ifndef PELCODTUI_H
#define PELCODTUI_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define PCT_MAX_ENTRIES 512
#define PCT_MAX_SECTIONS 96
#define PCT_TEXT 512

struct pct_entry { char section[96], key[96], value[PCT_TEXT]; };
struct pct_profile {
    struct pct_entry entries[PCT_MAX_ENTRIES];
    char sections[PCT_MAX_SECTIONS][96];
    size_t count, section_count;
    char path[256];
};

struct pct_transport {
    char device[128];
    unsigned baud, address, sequence_delay_ms;
    unsigned stop_repeat, stop_delay_ms;
    bool dry_run;
};

struct pct_motion {
    const struct pct_transport *transport;
    int fd;
    bool active;
};

struct pct_state_record {
    char value[128], command[PCT_TEXT], timestamp[40];
};

enum pct_command_op {
    PCT_COMMAND_STOP,
    PCT_COMMAND_PRESET_SET,
    PCT_COMMAND_PRESET_CALL,
    PCT_COMMAND_PRESET_CLEAR,
};

struct pct_command {
    enum pct_command_op op;
    unsigned value;
    bool value_placeholder;
};

int pct_profile_load(struct pct_profile *p, const char *path, char *err, size_t n);
int pct_profile_validate(const struct pct_profile *p, char *err, size_t n);
const char *pct_get(const struct pct_profile *p, const char *section, const char *key);
bool pct_csv_has(const char *csv, const char *value);
int pct_parse_command(const char *text, bool allow_value,
                      struct pct_command *command);
int pct_expand_setting(const struct pct_profile *p, const char *id, const char *value,
                       char *out, size_t n, char *err, size_t en);

void pct_frame_preset(uint8_t addr, const char *op, unsigned preset, uint8_t out[7]);
void pct_frame_stop(uint8_t addr, uint8_t out[7]);
void pct_frame_motion(uint8_t addr, const char *verb, unsigned speed, uint8_t out[7]);
int pct_execute(const struct pct_transport *t, const char *sequence,
                char *summary, size_t n);
int pct_move(const struct pct_transport *t, const char *verb, unsigned speed,
             unsigned duration_ms, char *summary, size_t n);
int pct_motion_start(struct pct_motion *motion, const struct pct_transport *t,
                     const char *verb, unsigned speed, char *summary, size_t n);
int pct_motion_stop(struct pct_motion *motion, char *summary, size_t n);

int pct_state_read(const char *path, const char *profile, const char *setting,
                   struct pct_state_record *out);
int pct_state_write(const char *path, const char *profile, const char *setting,
                    const char *value, const char *command, char *err, size_t n);
int pct_state_remove(const char *path, const char *profile, const char *setting,
                     char *err, size_t n);

void pct_sleep_ms(unsigned ms);

#endif
