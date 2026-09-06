#include "tui_internal.h"
#include "libmotors.h"
#include <curses.h>
#include <locale.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static volatile sig_atomic_t shutdown_signal;
static void request_shutdown(int signal) { shutdown_signal = signal; }

static int generic_tui(struct pct_ui_context *ui, bool dry_run,
                       bool use_motorsd, const char *socket_path) {
  struct pct_transport t = {.baud = 115200, .address = 1,
      .sequence_delay_ms = 150, .stop_repeat = 3, .stop_delay_ms = 10,
      .dry_run = dry_run, .use_motorsd = use_motorsd};
  snprintf(t.device, sizeof(t.device), "/dev/ttyAMA0");
  snprintf(t.socket_path, sizeof(t.socket_path), "%s", socket_path);
  return pct_ui_preset_menu(ui, &t, true, NULL);
}

static void configure_transport(struct pct_transport *t,
                                const struct pct_profile *p, bool dry_run) {
  *t = (struct pct_transport){.baud = 115200, .address = 1,
      .sequence_delay_ms = 150, .stop_repeat = 3, .stop_delay_ms = 10,
      .dry_run = dry_run};
  snprintf(t->device, sizeof(t->device), "%s",
           pct_ui_nz(pct_get(p, "uart", "device"), "/dev/ttyAMA0"));
  if (pct_get(p, "uart", "baud"))
    t->baud = atoi(pct_get(p, "uart", "baud"));
  if (pct_get(p, "uart", "address"))
    t->address = atoi(pct_get(p, "uart", "address"));
  if (pct_get(p, "driver", "sequence_delay_ms"))
    t->sequence_delay_ms = atoi(pct_get(p, "driver", "sequence_delay_ms"));
  if (pct_get(p, "driver", "stop_repeat"))
    t->stop_repeat = atoi(pct_get(p, "driver", "stop_repeat"));
  if (pct_get(p, "driver", "stop_delay_ms"))
    t->stop_delay_ms = atoi(pct_get(p, "driver", "stop_delay_ms"));
}

int main(int argc, char **argv) {
  struct sigaction sa = {.sa_handler = request_shutdown};
  sigemptyset(&sa.sa_mask);
  sigaction(SIGINT, &sa, NULL);
  sigaction(SIGTERM, &sa, NULL);
  sigaction(SIGHUP, &sa, NULL);
  setlocale(LC_ALL, "");
  struct pct_ui_context ui = {.profiles_dir = "/etc/pelcodtui/cameras",
      .state_path = "/etc/pelcodtui/state.conf", .message = "Ready",
      .shutdown_signal = &shutdown_signal};
  const char *profile = NULL;
  const char *socket_path = "/run/motorsd.sock";
  bool dry = false, direct = false, profile_from_state = false;
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "--profile") && i + 1 < argc) profile = argv[++i];
    else if (!strcmp(argv[i], "--profiles-dir") && i + 1 < argc) ui.profiles_dir = argv[++i];
    else if (!strcmp(argv[i], "--state") && i + 1 < argc) ui.state_path = argv[++i];
    else if (!strcmp(argv[i], "--socket") && i + 1 < argc) socket_path = argv[++i];
    else if (!strcmp(argv[i], "--direct")) direct = true;
    else if (!strcmp(argv[i], "--dry-run")) dry = true;
    else if (!strcmp(argv[i], "--validate") && i + 1 < argc) {
      struct pct_profile p; char error[160];
      int result = pct_profile_load(&p, argv[++i], error, sizeof(error));
      puts(result ? error : "profile valid");
      return result != 0;
    } else {
      fprintf(stderr, "Usage: pelcodtui [--profile FILE] [--profiles-dir DIR] "
                      "[--state FILE] [--socket PATH] [--direct] "
                      "[--dry-run] [--validate FILE]\n");
      return 2;
    }
  }

  if (!direct) {
    static struct pct_profile service_profile;
    static char description[32769];
    struct motors_client *client = NULL;
    char service_error[200] = "";
    int described = motors_open(&client, socket_path, service_error,
                                sizeof(service_error));
    if (!described)
      described = motors_describe(client, description, sizeof(description),
                                  service_error, sizeof(service_error));
    motors_close(client);
    if (!described)
      described = pct_profile_from_description(&service_profile, description,
                                                service_error,
                                                sizeof(service_error));

    initscr(); cbreak(); noecho(); keypad(stdscr, TRUE);
    if (!described) {
      struct pct_transport transport;
      configure_transport(&transport, &service_profile, dry);
      transport.use_motorsd = true;
      snprintf(transport.socket_path, sizeof(transport.socket_path), "%s",
               socket_path);
      int result = pct_ui_run(&ui, &service_profile, &transport);
      endwin();
      return result;
    }
    snprintf(ui.message, sizeof(ui.message), "Settings unavailable: %s",
             service_error[0] ? service_error : "driver has no settings");
    int result = generic_tui(&ui, dry, true, socket_path);
    endwin();
    return result;
  }

  initscr(); cbreak(); noecho(); keypad(stdscr, TRUE);
  char picked[256], error[200];
  struct pct_state_record saved;
  if (!profile && !pct_state_read(ui.state_path, "app", "selection", &saved) &&
      strcmp(saved.value, "generic")) {
    profile = saved.value;
    profile_from_state = true;
  }
  for (;;) {
    bool generic = false;
    if (!profile) {
      int result = pct_ui_picker(&ui, picked, sizeof(picked));
      if (result < 0) { endwin(); return 0; }
      generic = result > 0;
      if (!generic) profile = picked;
      profile_from_state = false;
      char state_error[128];
      pct_state_write(ui.state_path, "app", "selection",
                      generic ? "generic" : profile, "select", state_error,
                      sizeof(state_error));
    }
    if (generic || !strcmp(profile, "generic")) {
      int result = generic_tui(&ui, dry, !direct, socket_path);
      if (result == 2) { profile = NULL; continue; }
      endwin(); return result;
    }
    struct pct_profile p;
    if (pct_profile_load(&p, profile, error, sizeof(error))) {
      if (profile_from_state) {
        char state_error[128];
        pct_state_remove(ui.state_path, "app", "selection", state_error,
                         sizeof(state_error));
        profile = NULL;
        profile_from_state = false;
        continue;
      }
      endwin(); fprintf(stderr, "%s\n", error); return 1;
    }
    struct pct_transport t;
    configure_transport(&t, &p, dry);
    t.use_motorsd = !direct;
    snprintf(t.socket_path, sizeof(t.socket_path), "%s", socket_path);
    int result = pct_ui_run(&ui, &p, &t);
    if (result == 2) { profile = NULL; continue; }
    endwin(); return result;
  }
}
