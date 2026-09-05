#ifndef PCT_TUI_INTERNAL_H
#define PCT_TUI_INTERNAL_H

#include "pelcodtui.h"
#include <signal.h>

struct pct_tui_settings { unsigned pan_pulse, zoom_pulse, focus_pulse; };
struct pct_ui_context {
  const char *profiles_dir, *state_path;
  char message[256];
  volatile sig_atomic_t *shutdown_signal;
};

static inline bool pct_ui_shutdown(const struct pct_ui_context *ui) {
  return *ui->shutdown_signal != 0;
}

const char *pct_ui_nz(const char *value, const char *fallback);
void pct_ui_draw_areas(int split);
void pct_ui_load_settings(struct pct_ui_context *, struct pct_tui_settings *);
int pct_ui_picker(struct pct_ui_context *, char *, size_t);
int pct_ui_preset_menu(struct pct_ui_context *, const struct pct_transport *,
                       bool, const struct pct_profile *);
void pct_ui_camera_settings_menu(struct pct_ui_context *,
                                 const struct pct_profile *,
                                 const struct pct_transport *);
void pct_ui_settings_menu(struct pct_ui_context *, struct pct_tui_settings *);
int pct_ui_run(struct pct_ui_context *, struct pct_profile *,
               struct pct_transport *);

#endif
