#include "tui_internal.h"
#include <curses.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

static int64_t monotonic_ms(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}
static const char *motion_key(int c, const struct pct_tui_settings *s,
                              unsigned *speed, unsigned *pulse) {
  *speed = 30;
  *pulse = s->pan_pulse;
  if (c == 'a' || c == 'A')
    return "left";
  if (c == 'd' || c == 'D')
    return "right";
  if (c == 'w' || c == 'W')
    return "up";
  if (c == 's' || c == 'S')
    return "down";
  *speed = 0;
  if (c == 'z' || c == 'Z') {
    *pulse = s->zoom_pulse;
    return "wide";
  }
  if (c == 'x' || c == 'X') {
    *pulse = s->zoom_pulse;
    return "tele";
  }
  if (c == 'n' || c == 'N') {
    *pulse = s->focus_pulse;
    return "near";
  }
  if (c == 'f' || c == 'F') {
    *pulse = s->focus_pulse;
    return "far";
  }
  return NULL;
}
static int stop_motion(struct pct_ui_context *ui, struct pct_motion *m) {
  return m->active ? pct_motion_stop(m, ui->message, sizeof(ui->message)) : 0;
}
void pct_ui_draw_areas(int split) {
  int bottom = LINES - 6;
  mvhline(2, 1, ACS_HLINE, COLS - 2);
  mvhline(bottom, 1, ACS_HLINE, COLS - 2);
  mvhline(LINES - 3, 1, ACS_HLINE, COLS - 2);
  mvhline(LINES - 1, 1, ACS_HLINE, COLS - 2);
  mvvline(3, 1, ACS_VLINE, LINES - 4);
  mvvline(3, COLS - 2, ACS_VLINE, LINES - 4);
  mvvline(3, split - 1, ACS_VLINE, bottom - 3);
  mvaddch(2, 1, ACS_ULCORNER);
  mvaddch(2, COLS - 2, ACS_URCORNER);
  mvaddch(bottom, 1, ACS_LTEE);
  mvaddch(bottom, COLS - 2, ACS_RTEE);
  mvaddch(LINES - 3, 1, ACS_LTEE);
  mvaddch(LINES - 3, COLS - 2, ACS_RTEE);
  mvaddch(LINES - 1, 1, ACS_LLCORNER);
  mvaddch(LINES - 1, COLS - 2, ACS_LRCORNER);
  mvaddch(2, split - 1, ACS_TTEE);
  mvaddch(bottom, split - 1, ACS_BTEE);
  mvprintw(2, 3, " Settings ");
  mvprintw(2, split + 1, " Details ");
  mvprintw(bottom, 3, " Status ");
  mvprintw(LINES - 3, 3, " Controls ");
}

int pct_ui_run(struct pct_ui_context *ui, struct pct_profile *p,
               struct pct_transport *t) {
  int sel = 0;
  const char *main_labels[] = {"Preset control", "Camera settings",
                               "TUI settings"};
  const char *main_descriptions[] = {
      "Set or call any Pelco-D preset from 1 through 255.",
      "Configure camera controller features grouped by function.",
      "Configure keyboard movement pulse durations."};
  struct pct_motion motion = {.fd = -1};
  struct pct_tui_settings settings;
  pct_ui_load_settings(ui, &settings);
  char active[16] = "";
  int64_t deadline = 0, started = 0, stop_retry_at = 0;
  keypad(stdscr, TRUE);
  timeout(20);
  curs_set(0);
  if (has_colors()) {
    start_color();
    use_default_colors();
    init_pair(1, COLOR_CYAN, -1);
    init_pair(2, COLOR_YELLOW, -1);
  }
  for (;;) {
    if (pct_ui_shutdown(ui)) {
      for (int tries = 0; motion.active && tries < 3; tries++) {
        stop_motion(ui, &motion);
        pct_sleep_ms(20);
      }
      return 128 + *ui->shutdown_signal;
    }
    int64_t now = monotonic_ms();
    if (motion.active && now >= deadline && now >= stop_retry_at) {
      if (stop_motion(ui, &motion))
        stop_retry_at = now + 250;
      else
        active[0] = 0;
    }
    if (motion.active && now < deadline)
      snprintf(ui->message, sizeof(ui->message), "%s active (%lld/%lld ms)", active,
               (long long)(now - started), (long long)(deadline - started));
    erase();
    attron(COLOR_PAIR(1) | A_BOLD);
    mvprintw(0, 2, "pelcodtui");
    attroff(COLOR_PAIR(1) | A_BOLD);
    printw("  %s  %s @ %u addr %u%s",
           pct_ui_nz(pct_get(p, "profile", "name"), "Unnamed profile"),
           t->device, t->baud, t->address, t->dry_run ? "  DRY RUN" : "");
    mvprintw(
        1, 2,
        "Camera state is not readable; values below are last commands sent.");
    int listw = COLS / 2;
    if (listw < 38)
      listw = 38;
    pct_ui_draw_areas(listw);
    for (int i = 0; i < 3; i++) {
      if (i == sel)
        attron(A_REVERSE);
      mvprintw(3 + i, 2, "%-*.*s", listw - 4, listw - 4, main_labels[i]);
      if (i == sel)
        attroff(A_REVERSE);
    }
    int details_x = listw + 1;
    attron(COLOR_PAIR(1));
    mvprintw(3, details_x, "Main menu");
    attroff(COLOR_PAIR(1));
    mvprintw(4, details_x, "%s", main_labels[sel]);
    mvprintw(6, details_x, "%.*s", COLS - details_x - 2,
             main_descriptions[sel]);
    if (sel == 2) {
      mvprintw(9, details_x, "Pan/tilt: %u ms", settings.pan_pulse);
      mvprintw(10, details_x, "Zoom: %u ms", settings.zoom_pulse);
      mvprintw(11, details_x, "Focus: %u ms", settings.focus_pulse);
    }
    attron(COLOR_PAIR(2));
    mvprintw(LINES - 4, 2, "%.*s", COLS - 4, ui->message);
    attroff(COLOR_PAIR(2));
    mvprintw(LINES - 2, 2,
             "Enter select  C camera  WASD move  Z/X zoom  N/F "
             "focus  Space STOP  Q quit");
    refresh();
    int c = getch();
    if (c == ERR)
      continue;
    unsigned speed, pulse;
    const char *v = motion_key(c, &settings, &speed, &pulse);
    if (v) {
      now = monotonic_ms();
      if (motion.active && !strcmp(active, v)) {
        deadline += pulse;
        int64_t cap = started + 5000;
        if (deadline > cap)
          deadline = cap;
      } else {
        stop_motion(ui, &motion);
        if (!motion.active && !pct_motion_start(&motion, t, v, speed,
                                                ui->message, sizeof(ui->message))) {
          snprintf(active, sizeof(active), "%s", v);
          started = now;
          deadline = now + pulse;
          stop_retry_at = deadline;
        }
      }
      continue;
    }
    stop_motion(ui, &motion);
    if (motion.active)
      continue;
    active[0] = 0;
    if (c == 'q' || c == 'Q')
      break;
    if (c == 'c' || c == 'C')
      return 2;
    if (c == KEY_UP && sel)
      sel--;
    else if (c == KEY_DOWN && sel < 2)
      sel++;
    else if (c == '\n') {
      if (sel == 0) {
        pct_ui_preset_menu(ui, t, false, p);
        timeout(20);
      } else if (sel == 1) {
        pct_ui_camera_settings_menu(ui, p, t);
      } else {
        pct_ui_settings_menu(ui, &settings);
      }
    } else if (c == ' ')
      pct_move(t, "stop", 0, 0, ui->message, sizeof(ui->message));
  }
  stop_motion(ui, &motion);
  return 0;
}
