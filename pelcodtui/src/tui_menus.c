#include "tui_internal.h"
#include <ctype.h>
#include <curses.h>
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#define MAX_ITEMS 96
struct item {
  char id[96], section[128], menu[64];
  bool action, tui, preset;
};
static int confirm_dialog(struct pct_ui_context *ui, const char *text);
const char *pct_ui_nz(const char *s, const char *d) { return s && *s ? s : d; }
static void split_items(struct item *out, int *count, const char *menu,
                        const char *csv, const struct pct_profile *p) {
  char b[PCT_TEXT];
  snprintf(b, sizeof(b), "%s", csv ?: "");
  char *sv = NULL;
  for (char *x = strtok_r(b, ",", &sv); x && *count < MAX_ITEMS;
       x = strtok_r(NULL, ",", &sv)) {
    while (*x == ' ')
      x++;
    struct item *i = &out[(*count)++];
    memset(i, 0, sizeof(*i));
    snprintf(i->id, sizeof(i->id), "%s", x);
    snprintf(i->menu, sizeof(i->menu), "%s", menu);
    if (!strcmp(x, "__preset_control")) {
      snprintf(i->id, sizeof(i->id), "preset");
      i->preset = true;
      continue;
    }
    if (!strcmp(x, "__tui_settings")) {
      snprintf(i->id, sizeof(i->id), "tui_settings");
      i->tui = true;
      i->action = true;
      continue;
    }
    snprintf(i->section, sizeof(i->section), "setting.%s", x);
    i->tui = false;
    i->preset = false;
    i->action = pct_get(p, i->section, "label") == NULL;
    if (i->action)
      snprintf(i->section, sizeof(i->section), "action.%s", x);
  }
}
static int collect(const struct pct_profile *p, struct item *out) {
  int n = 0;
  for (size_t i = 0; i < p->section_count; i++)
    if (!strncmp(p->sections[i], "menu.", 5)) {
      const char *label =
          pct_ui_nz(pct_get(p, p->sections[i], "label"), p->sections[i] + 5);
      split_items(out, &n, label, pct_get(p, p->sections[i], "items"), p);
    }
  for (size_t s = 0; s < p->section_count && n < MAX_ITEMS; s++) {
    const char *section = p->sections[s];
    size_t prefix = !strncmp(section, "setting.", 8)  ? 8
                    : !strncmp(section, "action.", 7) ? 7
                                                      : 0;
    if (!prefix)
      continue;
    const char *id = section + prefix;
    bool listed = false;
    for (int i = 0; i < n; i++)
      if (!strcmp(out[i].id, id))
        listed = true;
    if (listed)
      continue;
    struct item *item = &out[n++];
    memset(item, 0, sizeof(*item));
    snprintf(item->id, sizeof(item->id), "%s", id);
    snprintf(item->section, sizeof(item->section), "%s", section);
    snprintf(item->menu, sizeof(item->menu), "Other / misc");
    item->action = prefix == 7;
  }
  return n;
}

static unsigned saved_uint(struct pct_ui_context *ui, const char *key,
                           unsigned fallback) {
  struct pct_state_record r;
  if (pct_state_read(ui->state_path, "app", key, &r))
    return fallback;
  char *end = NULL;
  unsigned long v = strtoul(r.value, &end, 10);
  return end && !*end && v > 0 && v <= 5000 ? (unsigned)v : fallback;
}
void pct_ui_load_settings(struct pct_ui_context *ui, struct pct_tui_settings *s) {
  s->pan_pulse = saved_uint(ui, "pan_pulse", 250);
  s->zoom_pulse = saved_uint(ui, "zoom_pulse", 300);
  s->focus_pulse = saved_uint(ui, "focus_pulse", 120);
}
static const char *tui_label(const char *id) {
  if (!strcmp(id, "tui_settings"))
    return "TUI settings";
  if (!strcmp(id, "pan_pulse"))
    return "Pan/tilt key pulse";
  if (!strcmp(id, "zoom_pulse"))
    return "Zoom key pulse";
  if (!strcmp(id, "focus_pulse"))
    return "Focus key pulse";
  return "Reset TUI settings";
}
static const char *tui_description(const char *id) {
  if (!strcmp(id, "tui_settings"))
    return "Configure keyboard movement pulse durations.";
  if (!strcmp(id, "pan_pulse"))
    return "Duration added by each W/A/S/D key press.";
  if (!strcmp(id, "zoom_pulse"))
    return "Duration added by each Z/X key press.";
  if (!strcmp(id, "focus_pulse"))
    return "Duration added by each N/F key press.";
  return "Restore all TUI pulse durations to their defaults.";
}
static unsigned *tui_value(struct pct_tui_settings *s, const char *id) {
  if (!strcmp(id, "pan_pulse"))
    return &s->pan_pulse;
  if (!strcmp(id, "zoom_pulse"))
    return &s->zoom_pulse;
  if (!strcmp(id, "focus_pulse"))
    return &s->focus_pulse;
  return NULL;
}
static unsigned tui_default(const char *id) {
  if (!strcmp(id, "pan_pulse"))
    return 250;
  if (!strcmp(id, "zoom_pulse"))
    return 300;
  return 120;
}
static void save_tui_value(struct pct_ui_context *ui, const char *id,
                           unsigned value) {
  char text[32], err[128];
  snprintf(text, sizeof(text), "%u", value);
  mkdir("/etc/pelcodtui", 0755);
  if (pct_state_write(ui->state_path, "app", id, text, "tui setting", err,
                      sizeof(err)))
    snprintf(ui->message, sizeof(ui->message), "Save failed: %s", err);
  else
    snprintf(ui->message, sizeof(ui->message), "Saved %s=%ums", id, value);
}
static void configure_tui(struct pct_ui_context *ui,
                          struct pct_tui_settings *s, const struct item *i) {
  if (i->action) {
    if (!confirm_dialog(ui, "Reset all TUI pulse durations?"))
      return;
    s->pan_pulse = 250;
    s->zoom_pulse = 300;
    s->focus_pulse = 120;
    save_tui_value(ui, "pan_pulse", s->pan_pulse);
    save_tui_value(ui, "zoom_pulse", s->zoom_pulse);
    save_tui_value(ui, "focus_pulse", s->focus_pulse);
    snprintf(ui->message, sizeof(ui->message), "TUI settings reset");
    return;
  }
  unsigned *v = tui_value(s, i->id);
  echo();
  curs_set(1);
  mvprintw(LINES - 2, 2, "Pulse duration 20..2000 ms: ");
  clrtoeol();
  char b[16] = "";
  int input_result = getnstr(b, 15);
  noecho();
  curs_set(0);
  if (input_result == ERR)
    return;
  char *end = NULL;
  unsigned long n = strtoul(b, &end, 10);
  if (!end || *end || n < 20 || n > 2000) {
    snprintf(ui->message, sizeof(ui->message), "Pulse must be 20..2000 ms");
    return;
  }
  *v = (unsigned)n;
  save_tui_value(ui, i->id, *v);
}

int pct_ui_picker(struct pct_ui_context *ui, char *out, size_t n) {
  const char *profiles_dir = ui->profiles_dir;
  DIR *d = opendir(profiles_dir);
  if (!d) {
    profiles_dir = "./cameras";
    d = opendir(profiles_dir);
  }
  char paths[32][256], names[32][96];
  int count = 0, sel = 0;
  struct dirent *e;
  if (d) {
    while ((e = readdir(d)) && count < 31) {
      size_t l = strlen(e->d_name);
      if (l > 5 && !strcmp(e->d_name + l - 5, ".conf")) {
        snprintf(paths[count], 256, "%s/%s", profiles_dir, e->d_name);
        struct pct_profile p;
        char er[128];
        if (!pct_profile_load(&p, paths[count], er, sizeof(er)))
          snprintf(names[count++], 96, "%s",
                   pct_ui_nz(pct_get(&p, "profile", "name"), e->d_name));
      }
    }
    closedir(d);
  }
  snprintf(names[count], 96, "My camera is not in the list");
  paths[count][0] = 0;
  count++;
  for (;;) {
    erase();
    mvprintw(2, 4, "No known camera state exists");
    mvprintw(4, 4, "Pelco-D cannot read settings back from the camera.");
    mvprintw(6, 4, "Select a camera profile:");
    for (int i = 0; i < count; i++) {
      if (i == sel)
        attron(A_REVERSE);
      mvprintw(8 + i, 6, "%s", names[i]);
      if (i == sel)
        attroff(A_REVERSE);
    }
    refresh();
    int c = getch();
    if (pct_ui_shutdown(ui))
      return -1;
    if (c == KEY_UP && sel)
      sel--;
    else if (c == KEY_DOWN && sel < count - 1)
      sel++;
    else if (c == '\n') {
      snprintf(out, n, "%s", paths[sel]);
      return paths[sel][0] ? 0 : 1;
    } else if (c == 'q' || c == 27)
      return -1;
  }
}
static int print_wrapped(WINDOW *w, int y, int x, int width, int max_lines,
                         const char *text) {
  int lines = 0;
  while (*text && lines < max_lines) {
    while (*text == ' ')
      text++;
    size_t available = strcspn(text, "\n"), take = available;
    if (take > (size_t)width) {
      take = (size_t)width;
      while (take && !isspace((unsigned char)text[take]))
        take--;
      if (!take)
        take = (size_t)width;
    }
    size_t visible = take;
    while (visible && isspace((unsigned char)text[visible - 1]))
      visible--;
    mvwaddnstr(w, y + lines++, x, text, (int)visible);
    text += take;
    while (*text == ' ')
      text++;
    if (*text == '\n')
      text++;
  }
  return lines;
}

static int confirm_dialog(struct pct_ui_context *ui, const char *text) {
  int rows, cols;
  getmaxyx(stdscr, rows, cols);
  int height = 9, width = cols > 70 ? 70 : cols - 4;
  WINDOW *w = newwin(height, width, (rows - height) / 2,
                     (cols - width) / 2);
  box(w, 0, 0);
  print_wrapped(w, 1, 2, width - 4, height - 4, text);
  mvwprintw(w, height - 2, 2,
            "Press y to continue, any other key to cancel");
  wrefresh(w);
  int c = wgetch(w);
  delwin(w);
  return !pct_ui_shutdown(ui) && (c == 'y' || c == 'Y');
}
static int choose_value(struct pct_ui_context *ui, const struct pct_profile *p,
                        const struct item *i,
                        char *out, size_t n) {
  const char *type = pct_get(p, i->section, "type");
  if (!strcmp(type, "number")) {
    echo();
    curs_set(1);
    mvprintw(LINES - 2, 2, "Value (%s..%s %s): ", pct_get(p, i->section, "min"),
             pct_get(p, i->section, "max"),
             pct_ui_nz(pct_get(p, i->section, "unit"), ""));
    clrtoeol();
    char b[32] = "";
    int input_result = getnstr(b, 31);
    noecho();
    curs_set(0);
    if (input_result == ERR)
      return -1;
    snprintf(out, n, "%s", b);
    return 0;
  }
  const char *opts = pct_get(p, i->section, "options");
  char b[PCT_TEXT], vals[16][64];
  int c = 0, sel = 0;
  snprintf(b, sizeof(b), "%s", opts);
  char *sv = NULL;
  for (char *x = strtok_r(b, ",", &sv); x && c < 16;
       x = strtok_r(NULL, ",", &sv)) {
    while (*x == ' ')
      x++;
    snprintf(vals[c++], 64, "%s", x);
  }
  for (;;) {
    mvprintw(LINES - 2, 2, "Choose: ");
    clrtoeol();
    for (int j = 0; j <= c; j++) {
      if (j == sel)
        attron(A_REVERSE);
      const char *label = "Cancel";
      char key[160];
      if (j < c) {
        snprintf(key, sizeof(key), "option.%.63s.label", vals[j]);
        label = pct_ui_nz(pct_get(p, i->section, key), vals[j]);
      }
      printw(" %s ", label);
      if (j == sel)
        attroff(A_REVERSE);
    }
    refresh();
    int k = getch();
    if (pct_ui_shutdown(ui))
      return -1;
    if ((k == KEY_LEFT || k == KEY_UP) && sel)
      sel--;
    else if ((k == KEY_RIGHT || k == KEY_DOWN) && sel < c)
      sel++;
    else if (k == '\n') {
      if (sel == c)
        return -1;
      snprintf(out, n, "%s", vals[sel]);
      return 0;
    } else if (k == 27)
      return -1;
  }
}
static void apply_item(struct pct_ui_context *ui, const struct pct_profile *p,
                       const struct item *i,
                       const struct pct_transport *t) {
  char value[128] = "action", cmd[PCT_TEXT], err[160];
  const char *confirm = pct_get(p, i->section, "confirm");
  if (confirm && !strcmp(confirm, "true") &&
      !confirm_dialog(ui, pct_ui_nz(pct_get(p, i->section, "warning"),
                         "This action can change camera state."))) {
    snprintf(ui->message, sizeof(ui->message), "Cancelled");
    return;
  }
  if (i->action)
    snprintf(cmd, sizeof(cmd), "%s", pct_get(p, i->section, "command"));
  else {
    if (choose_value(ui, p, i, value, sizeof(value)))
      return;
    if (pct_expand_setting(p, i->id, value, cmd, sizeof(cmd), err,
                           sizeof(err))) {
      snprintf(ui->message, sizeof(ui->message), "%s", err);
      return;
    }
  }
  if (pct_execute(t, cmd, ui->message, sizeof(ui->message)))
    return;
  if (t->dry_run)
    return;
  mkdir("/etc/pelcodtui", 0755);
  if (pct_state_write(ui->state_path, pct_get(p, "profile", "id"), i->id, value,
                      cmd, err, sizeof(err)))
    snprintf(ui->message, sizeof(ui->message), "Sent, but %s", err);
}

static void draw_setting_details(struct pct_ui_context *ui,
                                 const struct pct_profile *p,
                                 const struct item *item, int x) {
  if (item->preset) {
    attron(COLOR_PAIR(1));
    mvprintw(3, x, "Category: %s", item->menu);
    attroff(COLOR_PAIR(1));
    mvprintw(4, x, "Preset control");
    mvprintw(6, x, "Set or call any Pelco-D preset from 1 through 255.");
    mvprintw(9, x, "Press Enter to open preset control");
    return;
  }
  const char *label = pct_ui_nz(pct_get(p, item->section, "label"), item->id);
  const char *desc = pct_ui_nz(pct_get(p, item->section, "description"), "");
  attron(COLOR_PAIR(1));
  mvprintw(3, x, "Category: %s", item->menu);
  attroff(COLOR_PAIR(1));
  mvprintw(4, x, "%s", label);
  int max_desc_lines = LINES - 16;
  if (max_desc_lines < 1)
    max_desc_lines = 1;
  int desc_lines = print_wrapped(stdscr, 6, x, COLS - x - 2,
                                 max_desc_lines, desc);
  int state_y = 6 + desc_lines + 1;
  if (state_y < 9)
    state_y = 9;

  struct pct_state_record record;
  if (!pct_state_read(ui->state_path, pct_get(p, "profile", "id"), item->id,
                      &record)) {
    mvprintw(state_y, x, "Last sent: %s", record.value);
    mvprintw(state_y + 1, x, "At: %s", record.timestamp);
    mvprintw(state_y + 2, x, "Not confirmed by camera");
  } else {
    mvprintw(state_y, x, "Last sent: unknown");
  }
}

static void camera_category_menu(struct pct_ui_context *ui,
                                 const struct pct_profile *p,
                                 const struct pct_transport *t,
                                 struct item *items, int count,
                                 const char *category) {
  int indexes[MAX_ITEMS], item_count = 0, sel = 0;
  for (int i = 0; i < count; i++)
    if (!items[i].tui && !strcmp(items[i].menu, category))
      indexes[item_count++] = i;

  timeout(-1);
  for (;;) {
    erase();
    int split = COLS / 2;
    if (split < 38)
      split = 38;
    pct_ui_draw_areas(split);
    attron(COLOR_PAIR(1) | A_BOLD);
    mvprintw(0, 2, "pelcodtui");
    attroff(COLOR_PAIR(1) | A_BOLD);
    printw("  Camera settings  >  %s", category);

    for (int i = 0; i < item_count && i < LINES - 7; i++) {
      const struct item *item = &items[indexes[i]];
      const char *label = item->preset
                              ? "Preset control"
                              : pct_ui_nz(pct_get(p, item->section, "label"), item->id);
      if (i == sel)
        attron(A_REVERSE);
      mvprintw(3 + i, 2, "%-*.*s", split - 4, split - 4, label);
      if (i == sel)
        attroff(A_REVERSE);
    }
    if (item_count)
      draw_setting_details(ui, p, &items[indexes[sel]], split + 1);
    attron(COLOR_PAIR(2));
    mvprintw(LINES - 4, 2, "%.*s", COLS - 4, ui->message);
    attroff(COLOR_PAIR(2));
    mvprintw(LINES - 2, 2, "Enter select  Q back");
    refresh();

    int c = getch();
    if (pct_ui_shutdown(ui) || c == 'q' || c == 'Q' || c == 27)
      break;
    if (c == KEY_UP && sel)
      sel--;
    else if (c == KEY_DOWN && sel < item_count - 1)
      sel++;
    else if (c == '\n' && item_count) {
      struct item *item = &items[indexes[sel]];
      if (item->preset) {
        pct_ui_preset_menu(ui, t, false, p);
        timeout(-1);
      } else {
        apply_item(ui, p, item, t);
      }
    }
  }
}

void pct_ui_camera_settings_menu(struct pct_ui_context *ui,
                                 const struct pct_profile *p,
                                 const struct pct_transport *t) {
  struct item items[MAX_ITEMS];
  int count = collect(p, items);
  char categories[32][64];
  int category_count = 0, sel = 0;
  for (int i = 0; i < count; i++) {
    if (items[i].tui)
      continue;
    bool found = false;
    for (int j = 0; j < category_count; j++)
      if (!strcmp(categories[j], items[i].menu))
        found = true;
    if (!found && category_count < 32)
      snprintf(categories[category_count++], sizeof(categories[0]), "%s",
               items[i].menu);
  }

  timeout(-1);
  for (;;) {
    erase();
    int split = COLS / 2;
    if (split < 38)
      split = 38;
    pct_ui_draw_areas(split);
    attron(COLOR_PAIR(1) | A_BOLD);
    mvprintw(0, 2, "pelcodtui");
    attroff(COLOR_PAIR(1) | A_BOLD);
    printw("  Camera settings");
    for (int i = 0; i < category_count; i++) {
      if (i == sel)
        attron(A_REVERSE);
      mvprintw(3 + i, 2, "%-*.*s", split - 4, split - 4, categories[i]);
      if (i == sel)
        attroff(A_REVERSE);
    }
    if (category_count) {
      attron(COLOR_PAIR(1));
      mvprintw(3, split + 1, "Category");
      attroff(COLOR_PAIR(1));
      mvprintw(5, split + 1, "%s", categories[sel]);
      int entries = 0;
      for (int i = 0; i < count; i++)
        if (!items[i].tui && !strcmp(items[i].menu, categories[sel]))
          entries++;
      mvprintw(7, split + 1, "%d setting%s", entries,
               entries == 1 ? "" : "s");
    }
    attron(COLOR_PAIR(2));
    mvprintw(LINES - 4, 2, "%.*s", COLS - 4, ui->message);
    attroff(COLOR_PAIR(2));
    mvprintw(LINES - 2, 2, "Enter open  Q back");
    refresh();

    int c = getch();
    if (pct_ui_shutdown(ui) || c == 'q' || c == 'Q' || c == 27)
      break;
    if (c == KEY_UP && sel)
      sel--;
    else if (c == KEY_DOWN && sel < category_count - 1)
      sel++;
    else if (c == '\n' && category_count) {
      camera_category_menu(ui, p, t, items, count, categories[sel]);
      timeout(-1);
    }
  }
  timeout(20);
}

int pct_ui_preset_menu(struct pct_ui_context *ui,
                       const struct pct_transport *t, bool camera_key,
                       const struct pct_profile *p) {
  char number[4] = "", action[5] = "Call";
  size_t len = 0;
  timeout(-1);
  for (;;) {
    erase();
    attron(COLOR_PAIR(1) | A_BOLD);
    mvprintw(1, 2, "pelcodtui");
    attroff(COLOR_PAIR(1) | A_BOLD);
    printw("  Preset control");
    mvprintw(4, 4, "Type a number: range 1..255,");
    mvprintw(5, 4, "press S for Set or C for Call, then Enter.");
    mvprintw(8, 4, "Preset number: [%-3s]  Current action: %s", number, action);
    attron(COLOR_PAIR(2));
    mvprintw(11, 4, "%s", ui->message);
    attroff(COLOR_PAIR(2));
    mvprintw(LINES - 2, 2, "%s", camera_key ? "Esc camera  Q quit" : "Q back");
    refresh();
    int c = getch();
    if (pct_ui_shutdown(ui))
      return 0;
    if (c == 'q' || c == 'Q')
      return 0;
    if (c == 27)
      return camera_key ? 2 : 0;
    if (c >= '0' && c <= '9' && len < 3) {
      number[len++] = (char)c;
      number[len] = 0;
    } else if ((c == KEY_BACKSPACE || c == 127 || c == 8) && len) {
      number[--len] = 0;
    } else if (c == 's' || c == 'S') {
      snprintf(action, sizeof(action), "Set");
    } else if (c == 'c' || c == 'C') {
      snprintf(action, sizeof(action), "Call");
    } else if (c == '\n') {
      int preset = atoi(number);
      if (preset < 1 || preset > 255) {
        snprintf(ui->message, sizeof(ui->message), "Preset must be 1..255");
        continue;
      }
      char cmd[64];
      const char *dangerous = p ? pct_get(p, "profile",
          !strcmp(action, "Set") ? "dangerous_set" : "dangerous_call") : NULL;
      char preset_text[4];
      snprintf(preset_text, sizeof(preset_text), "%d", preset);
      if (pct_csv_has(dangerous, preset_text) &&
          !confirm_dialog(ui, "This preset command can reset or delete controller state."))
        continue;
      snprintf(cmd, sizeof(cmd), "preset_%s %d",
               !strcmp(action, "Set") ? "set" : "call", preset);
      pct_execute(t, cmd, ui->message, sizeof(ui->message));
    }
  }
}

void pct_ui_settings_menu(struct pct_ui_context *ui,
                          struct pct_tui_settings *settings) {
  const char *ids[] = {"pan_pulse", "zoom_pulse", "focus_pulse", "reset"};
  int sel = 0;
  timeout(-1);
  for (;;) {
    erase();
    int split = COLS / 2;
    if (split < 38)
      split = 38;
    pct_ui_draw_areas(split);
    attron(COLOR_PAIR(1) | A_BOLD);
    mvprintw(0, 2, "pelcodtui");
    attroff(COLOR_PAIR(1) | A_BOLD);
    printw("  TUI settings");

    for (int i = 0; i < 4; i++) {
      if (i == sel)
        attron(A_REVERSE);
      mvprintw(4 + i, 3, "%-*.*s", split - 5, split - 5, tui_label(ids[i]));
      if (i == sel)
        attroff(A_REVERSE);
    }
    int x = split + 1;
    mvprintw(3, x, "TUI settings");
    mvprintw(5, x, "%.*s", COLS - x - 2, tui_description(ids[sel]));
    unsigned *value = tui_value(settings, ids[sel]);
    if (value) {
      mvprintw(8, x, "Current: %u ms", *value);
      mvprintw(9, x, "Default: %u ms", tui_default(ids[sel]));
    } else {
      mvprintw(8, x, "Pan 250 ms, zoom 300 ms, focus 120 ms");
    }
    attron(COLOR_PAIR(2));
    mvprintw(LINES - 4, 2, "%.*s", COLS - 4, ui->message);
    attroff(COLOR_PAIR(2));
    mvprintw(LINES - 2, 2, "Enter change  Q back");
    refresh();
    int c = getch();
    if (pct_ui_shutdown(ui))
      break;
    if (c == 'q' || c == 'Q' || c == 27)
      break;
    if (c == KEY_UP && sel)
      sel--;
    else if (c == KEY_DOWN && sel < 3)
      sel++;
    else if (c == '\n') {
      struct item item = {.tui = true, .action = sel == 3};
      snprintf(item.id, sizeof(item.id), "%s", ids[sel]);
      configure_tui(ui, settings, &item);
    }
  }
  timeout(20);
}
