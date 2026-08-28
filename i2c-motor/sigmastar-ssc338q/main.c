#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <stdint.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/file.h>

#define DEFAULT_I2C_BUS "/dev/i2c-2"
#define DEFAULT_I2C_ADDR 0x10
#define DEFAULT_PPS 800
#define DEFAULT_STEPS 500

#define LOCK_FILE "/tmp/motor.lock"

// Exact verified mechanical limits for CW-TY207135D14 (8MP 2.7-13.5mm)
#define LENS_ZOOM_MAX_STEPS   6000
#define LENS_FOCUS_MAX_STEPS  4800

// True optical usable baseline (Zoom < 2000 is beyond focal plane for this sensor mount)
#define LENS_ZOOM_HOME_POS    2000 // Usable wide-angle baseline (6.3 mm)
#define LENS_FOCUS_HOME_POS   4690 // Sharp focus at Zoom 2000

#define POS_FILE "/tmp/lens_pos"

static int g_lock_fd = -1;

void acquire_lock(void) {
    g_lock_fd = open(LOCK_FILE, O_CREAT | O_RDWR, 0666);
    if (g_lock_fd >= 0) {
        flock(g_lock_fd, LOCK_EX);
    }
}

void release_lock(void) {
    if (g_lock_fd >= 0) {
        flock(g_lock_fd, LOCK_UN);
        close(g_lock_fd);
        g_lock_fd = -1;
    }
}

typedef struct {
    int zoom_pos;
    int focus_pos;
    int is_calibrated;
} LensState;

// Forward Declarations
int set_zoom_parfocal(const char *bus, unsigned char addr, int target_zoom, int pps);
int do_home(const char *bus, unsigned char addr, int pps);
int move_zoom_tracked(const char *bus, unsigned char addr, int steps, int dir, int pps);
int move_focus_tracked(const char *bus, unsigned char addr, int steps, int dir, int pps);
double zoom_to_focal_mm(int zoom_pos);
int focal_mm_to_zoom(double focal_mm);
void load_state(LensState *st);
void save_state(const LensState *st);

void load_state(LensState *st) {
    st->zoom_pos = LENS_ZOOM_HOME_POS;
    st->focus_pos = LENS_FOCUS_HOME_POS;
    st->is_calibrated = 0;

    FILE *f = fopen(POS_FILE, "r");
    if (f) {
        fscanf(f, "%d %d %d", &st->zoom_pos, &st->focus_pos, &st->is_calibrated);
        fclose(f);
    }
}

void save_state(const LensState *st) {
    FILE *f = fopen(POS_FILE, "w");
    if (f) {
        fprintf(f, "%d %d %d\n", st->zoom_pos, st->focus_pos, st->is_calibrated);
        fclose(f);
    }
}

void riu_init_hardware(void) {
    static int initialized = 0;
    if (initialized) return;
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd >= 0) {
        // Bank 0x111B Offset 0x06 = 0x0000 (I2C/Motor Power Gate)
        void *map = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x1F223000);
        if (map != MAP_FAILED) {
            volatile uint16_t *reg = (volatile uint16_t *)((uint8_t *)map + 0x618); // Bank 0x111B off 0x06
            *reg = 0x0000;
            munmap(map, 0x1000);
        }
        close(fd);
    }
    initialized = 1;
}

int i2c_write(int fd, unsigned char addr, unsigned char reg, unsigned char val) {
    riu_init_hardware();
    unsigned char buf[2] = {reg, val};
    struct i2c_msg msg = {
        .addr = addr,
        .flags = 0,
        .len = 2,
        .buf = buf
    };
    struct i2c_rdwr_ioctl_data rdwr = {
        .msgs = &msg,
        .nmsgs = 1
    };
    return ioctl(fd, I2C_RDWR, &rdwr);
}

void set_channel_speeds(int fd, unsigned char addr, int pps_focus, int pps_zoom) {
    if (pps_focus < 32) pps_focus = 32;
    if (pps_focus > 16383) pps_focus = 16383;
    if (pps_zoom < 32) pps_zoom = 32;
    if (pps_zoom > 16383) pps_zoom = 16383;

    unsigned int div_f = 24000000 / pps_focus;
    unsigned int div_z = 24000000 / pps_zoom;

    // Channel 1: Focus Speed (Regs 0x01, 0x02)
    i2c_write(fd, addr, 0x01, (div_f >> 7) & 0xFF);
    i2c_write(fd, addr, 0x02, 0x80 | (div_f >> 15));

    // Channel 2: Zoom Speed (Regs 0x05, 0x06)
    i2c_write(fd, addr, 0x05, (div_z >> 7) & 0xFF);
    i2c_write(fd, addr, 0x06, 0x80 | (div_z >> 15));
}

// Single-packet hardware move for Channel 2 (ZOOM: Regs 0x07, 0x08, Trigger 0x4F)
int raw_step_zoom_chunk(const char *bus, unsigned char addr, int chunk_steps, int dir, int pps) {
    if (chunk_steps <= 0) return 0;
    if (chunk_steps > 3500) chunk_steps = 3500;

    int fd = open(bus, O_RDWR);
    if (fd < 0) return -1;

    // Wake chip & enable excitation
    i2c_write(fd, addr, 0x00, 0x01);
    i2c_write(fd, addr, 0x0A, 0x08);

    set_channel_speeds(fd, addr, pps, pps);

    unsigned char z_lo = chunk_steps & 0xFF;
    unsigned char z_hi = (dir ? 0xC0 : 0x80) | ((chunk_steps >> 8) & 0x0F);

    i2c_write(fd, addr, 0x03, 0x00);
    i2c_write(fd, addr, 0x04, 0x00);
    i2c_write(fd, addr, 0x07, z_lo);
    i2c_write(fd, addr, 0x08, z_hi);
    i2c_write(fd, addr, 0x09, 0x4F); // Trigger Channel 2 (Zoom)

    close(fd);

    int sleep_ms = (chunk_steps * 1000) / pps + 40;
    usleep(sleep_ms * 1000);

    // Power off coils to prevent motor heating (0 holding current)
    fd = open(bus, O_RDWR);
    if (fd >= 0) {
        i2c_write(fd, addr, 0x0A, 0x00);
        i2c_write(fd, addr, 0x00, 0x00);
        close(fd);
    }
    return 0;
}

// Single-packet hardware move for Channel 1 (FOCUS: Regs 0x03, 0x04, Trigger 0x8F)
int raw_step_focus_chunk(const char *bus, unsigned char addr, int chunk_steps, int dir, int pps) {
    if (chunk_steps <= 0) return 0;
    if (chunk_steps > 3500) chunk_steps = 3500;

    int fd = open(bus, O_RDWR);
    if (fd < 0) return -1;

    // Wake chip & enable excitation
    i2c_write(fd, addr, 0x00, 0x01);
    i2c_write(fd, addr, 0x0A, 0x08);

    set_channel_speeds(fd, addr, pps, pps);

    int hw_dir = dir ? 0 : 1; // Invert hardware bitmask for Focus axis
    unsigned char f_lo = chunk_steps & 0xFF;
    unsigned char f_hi = (hw_dir ? 0xC0 : 0x80) | ((chunk_steps >> 8) & 0x0F);

    i2c_write(fd, addr, 0x07, 0x00);
    i2c_write(fd, addr, 0x08, 0x00);
    i2c_write(fd, addr, 0x03, f_lo);
    i2c_write(fd, addr, 0x04, f_hi);
    i2c_write(fd, addr, 0x09, 0x8F); // Trigger Channel 1 (Focus)

    close(fd);

    int sleep_ms = (chunk_steps * 1000) / pps + 40;
    usleep(sleep_ms * 1000);

    // Power off coils to prevent motor heating (0 holding current)
    fd = open(bus, O_RDWR);
    if (fd >= 0) {
        i2c_write(fd, addr, 0x0A, 0x00);
        i2c_write(fd, addr, 0x00, 0x00);
        close(fd);
    }
    return 0;
}

// Synchronized Simultaneous Dual-Axis Hardware Movement (Trigger 0xCF)
int raw_step_dual_sync(const char *bus, unsigned char addr, int z_steps, int z_dir, int f_steps, int f_dir, int base_pps) {
    if (z_steps <= 0 && f_steps <= 0) return 0;
    if (z_steps > 3500) z_steps = 3500;
    if (f_steps > 3500) f_steps = 3500;

    int fd = open(bus, O_RDWR);
    if (fd < 0) return -1;

    // Wake chip & enable excitation
    i2c_write(fd, addr, 0x00, 0x01);
    i2c_write(fd, addr, 0x0A, 0x08);

    // Calculate proportional speeds so both motors start and finish together
    int max_steps = (z_steps > f_steps) ? z_steps : f_steps;
    int pps_z = base_pps;
    int pps_f = base_pps;
    if (max_steps > 0) {
        if (z_steps > 0) pps_z = (base_pps * z_steps) / max_steps;
        if (f_steps > 0) pps_f = (base_pps * f_steps) / max_steps;
    }
    set_channel_speeds(fd, addr, pps_f, pps_z);

    int hw_f_dir = f_dir ? 0 : 1; // Invert hardware bitmask for Focus axis
    unsigned char f_lo = f_steps & 0xFF;
    unsigned char f_hi = (hw_f_dir ? 0xC0 : 0x80) | ((f_steps >> 8) & 0x0F);

    unsigned char z_lo = z_steps & 0xFF;
    unsigned char z_hi = (z_dir ? 0xC0 : 0x80) | ((z_steps >> 8) & 0x0F);

    i2c_write(fd, addr, 0x03, f_lo);
    i2c_write(fd, addr, 0x04, f_hi);
    i2c_write(fd, addr, 0x07, z_lo);
    i2c_write(fd, addr, 0x08, z_hi);

    unsigned char trigger = 0xCF;
    if (z_steps == 0) trigger = 0x8F;
    else if (f_steps == 0) trigger = 0x4F;
    i2c_write(fd, addr, 0x09, trigger);

    close(fd);

    int max_time_ms = 0;
    if (z_steps > 0 && pps_z > 0) {
        int t = (z_steps * 1000) / pps_z;
        if (t > max_time_ms) max_time_ms = t;
    }
    if (f_steps > 0 && pps_f > 0) {
        int t = (f_steps * 1000) / pps_f;
        if (t > max_time_ms) max_time_ms = t;
    }
    usleep((max_time_ms + 40) * 1000);

    // Power off coils to prevent motor heating (0 holding current)
    fd = open(bus, O_RDWR);
    if (fd >= 0) {
        i2c_write(fd, addr, 0x0A, 0x00);
        i2c_write(fd, addr, 0x00, 0x00);
        close(fd);
    }
    return 0;
}

// Move Zoom with automatic multi-chunk handling
int move_zoom(const char *bus, unsigned char addr, int total_steps, int dir, int pps) {
    printf(">>> ZOOM: Moving %d steps %s @ %d PPS...",
           total_steps, dir ? "IN (Tele)" : "OUT (Wide)", pps);
    fflush(stdout);

    int remaining = total_steps;
    while (remaining > 0) {
        int chunk = (remaining > 3000) ? 3000 : remaining;
        raw_step_zoom_chunk(bus, addr, chunk, dir, pps);
        remaining -= chunk;
    }
    printf(" Done.\n");
    return 0;
}

// Move Focus with automatic multi-chunk handling
int move_focus(const char *bus, unsigned char addr, int total_steps, int dir, int pps) {
    printf(">>> FOCUS: Moving %d steps %s @ %d PPS...",
           total_steps, dir ? "NEAR" : "FAR", pps);
    fflush(stdout);

    int remaining = total_steps;
    while (remaining > 0) {
        int chunk = (remaining > 3000) ? 3000 : remaining;
        raw_step_focus_chunk(bus, addr, chunk, dir, pps);
        remaining -= chunk;
    }
    printf(" Done.\n");
    return 0;
}

// Parabolic Optical Focus Tracking Curve: Focus(z) = a*z^2 + b*z + c
int get_calibrated_focus(int zoom_pos) {
    if (zoom_pos < 2000) return 4690;
    if (zoom_pos > 6000) zoom_pos = 6000;

    double z = (double)zoom_pos;
    double a = -0.000190833333;
    double b = 0.704166666667;
    double c = 4045.0;

    double foc = a * z * z + b * z + c;
    if (foc < 0) foc = 0;
    if (foc > LENS_FOCUS_MAX_STEPS) foc = LENS_FOCUS_MAX_STEPS;
    return (int)round(foc);
}

double zoom_to_focal_mm(int zoom_pos) {
    return 2.7 + ((double)zoom_pos / (double)LENS_ZOOM_MAX_STEPS) * 10.8;
}

int focal_mm_to_zoom(double focal_mm) {
    if (focal_mm < 2.7) focal_mm = 2.7;
    if (focal_mm > 13.5) focal_mm = 13.5;
    return (int)(((focal_mm - 2.7) / 10.8) * (double)LENS_ZOOM_MAX_STEPS);
}

int do_home(const char *bus, unsigned char addr, int pps) {
    printf("=================================================================\n");
    printf("  Executing Optical Homing Calibration (CW-TY207135D14 5x)\n");
    printf("  Mechanical Limits: Zoom = 0..%d steps, Focus = 0..%d steps\n", 
           LENS_ZOOM_MAX_STEPS, LENS_FOCUS_MAX_STEPS);
    printf("  Calibrated Usable Home: Zoom = %d (6.3mm), Focus = %d (Sharp)\n",
           LENS_ZOOM_HOME_POS, LENS_FOCUS_HOME_POS);
    printf("=================================================================\n");

    printf("[1/4] Homing Zoom Axis to 0 (Full Wide mechanical hard-stop)...");
    fflush(stdout);
    move_zoom(bus, addr, LENS_ZOOM_MAX_STEPS + 500, 0, pps);

    printf("[2/4] Homing Focus Axis to 0 (Infinity mechanical hard-stop)...");
    fflush(stdout);
    move_focus(bus, addr, LENS_FOCUS_MAX_STEPS + 500, 0, pps);

    printf("[3/4] Positioning Zoom to calibrated Wide optical angle (%d steps, %.1f mm)...", 
           LENS_ZOOM_HOME_POS, zoom_to_focal_mm(LENS_ZOOM_HOME_POS));
    fflush(stdout);
    move_zoom(bus, addr, LENS_ZOOM_HOME_POS, 1, pps);

    printf("[4/4] Setting Focus to calibrated sharp focal plane (%d steps NEAR)...", LENS_FOCUS_HOME_POS);
    fflush(stdout);
    move_focus(bus, addr, LENS_FOCUS_HOME_POS, 1, pps);

    LensState st = {
        .zoom_pos = LENS_ZOOM_HOME_POS,
        .focus_pos = LENS_FOCUS_HOME_POS,
        .is_calibrated = 1
    };
    save_state(&st);

    printf("\n>>> Homing Complete! Lens Calibrated at Zoom: %d (%.1fmm), Focus: %d <<<\n",
           st.zoom_pos, zoom_to_focal_mm(st.zoom_pos), st.focus_pos);
    return 0;
}

int move_zoom_tracked(const char *bus, unsigned char addr, int steps, int dir, int pps) {
    LensState st;
    load_state(&st);

    int target = dir ? (st.zoom_pos + steps) : (st.zoom_pos - steps);
    if (target > LENS_ZOOM_MAX_STEPS) target = LENS_ZOOM_MAX_STEPS;
    if (target < 0) target = 0;

    int actual_steps = abs(target - st.zoom_pos);
    if (actual_steps == 0) {
        printf("Zoom already at %s limit (%d / %d steps)\n", 
               dir ? "TELE" : "WIDE", st.zoom_pos, LENS_ZOOM_MAX_STEPS);
        return 0;
    }

    move_zoom(bus, addr, actual_steps, dir, pps);

    st.zoom_pos = target;
    save_state(&st);
    printf(">>> Zoom Position: %d / %d steps (%.1f mm, %.1fx) <<<\n",
           st.zoom_pos, LENS_ZOOM_MAX_STEPS, zoom_to_focal_mm(st.zoom_pos),
           zoom_to_focal_mm(st.zoom_pos) / 2.7);
    return 0;
}

int move_focus_tracked(const char *bus, unsigned char addr, int steps, int dir, int pps) {
    LensState st;
    load_state(&st);

    int target = dir ? (st.focus_pos + steps) : (st.focus_pos - steps);
    if (target > LENS_FOCUS_MAX_STEPS) target = LENS_FOCUS_MAX_STEPS;
    if (target < 0) target = 0;

    int actual_steps = abs(target - st.focus_pos);
    if (actual_steps == 0) {
        printf("Focus already at %s limit (%d / %d steps)\n", 
               dir ? "NEAR" : "FAR", st.focus_pos, LENS_FOCUS_MAX_STEPS);
        return 0;
    }

    move_focus(bus, addr, actual_steps, dir, pps);

    st.focus_pos = target;
    save_state(&st);
    printf(">>> Focus Position: %d / %d steps (%d%%) <<<\n",
           st.focus_pos, LENS_FOCUS_MAX_STEPS, (st.focus_pos * 100) / LENS_FOCUS_MAX_STEPS);
    return 0;
}

int set_zoom_absolute(const char *bus, unsigned char addr, int target_pos, int pps) {
    LensState st;
    load_state(&st);

    if (target_pos > LENS_ZOOM_MAX_STEPS) target_pos = LENS_ZOOM_MAX_STEPS;
    if (target_pos < 0) target_pos = 0;

    int delta = target_pos - st.zoom_pos;
    if (delta == 0) {
        printf("Zoom already at target %d (%.1f mm)\n", target_pos, zoom_to_focal_mm(target_pos));
        return 0;
    }

    int dir = (delta > 0) ? 1 : 0;
    int steps = abs(delta);
    return move_zoom_tracked(bus, addr, steps, dir, pps);
}

int set_focus_absolute(const char *bus, unsigned char addr, int target_pos, int pps) {
    LensState st;
    load_state(&st);

    if (target_pos > LENS_FOCUS_MAX_STEPS) target_pos = LENS_FOCUS_MAX_STEPS;
    if (target_pos < 0) target_pos = 0;

    int delta = target_pos - st.focus_pos;
    if (delta == 0) {
        printf("Focus already at target %d / %d\n", target_pos, LENS_FOCUS_MAX_STEPS);
        return 0;
    }

    int dir = (delta > 0) ? 1 : 0;
    int steps = abs(delta);
    return move_focus_tracked(bus, addr, steps, dir, pps);
}

// Smooth Continuous Parfocal Optical Tracking Movement
// Moves Zoom & Focus SIMULTANEOUSLY in interpolated micro-step segments
int set_focal_length_smooth(const char *bus, unsigned char addr, double focal_mm, int pps) {
    LensState st;
    load_state(&st);

    int target_zoom = focal_mm_to_zoom(focal_mm);
    int target_focus = get_calibrated_focus(target_zoom);

    int start_zoom = st.zoom_pos;
    int start_focus = st.focus_pos;

    int total_z_delta = target_zoom - start_zoom;
    if (abs(total_z_delta) < 10) {
        // Just fine-tune focus
        set_focus_absolute(bus, addr, target_focus, pps);
        return 0;
    }

    printf("=================================================================\n");
    printf("  [PARFOCAL SMOOTH TRACKING] Target: %.1f mm (%.1fx Zoom)\n", 
           focal_mm, focal_mm / 2.7);
    printf("  Trajectory: Zoom %d -> %d | Focus %d -> %d\n",
           start_zoom, target_zoom, start_focus, target_focus);
    printf("=================================================================\n");

    // Break trajectory into synchronized 100-step micro-chunks
    int chunk_size = 100;
    int steps_remaining = abs(total_z_delta);
    int z_dir = (total_z_delta > 0) ? 1 : 0;

    int current_z = start_zoom;
    int current_f = start_focus;

    printf("Tracking along optical curve...");
    fflush(stdout);

    while (steps_remaining > 0) {
        int z_step = (steps_remaining > chunk_size) ? chunk_size : steps_remaining;
        int next_z = z_dir ? (current_z + z_step) : (current_z - z_step);
        int next_f = get_calibrated_focus(next_z);

        int f_step = abs(next_f - current_f);
        int f_dir = (next_f > current_f) ? 1 : 0;

        raw_step_dual_sync(bus, addr, z_step, z_dir, f_step, f_dir, pps);

        current_z = next_z;
        current_f = next_f;
        steps_remaining -= z_step;
    }

    // Final precision alignment
    int final_f_delta = target_focus - current_f;
    if (abs(final_f_delta) > 0) {
        int f_dir = (final_f_delta > 0) ? 1 : 0;
        raw_step_focus_chunk(bus, addr, abs(final_f_delta), f_dir, pps);
    }

    st.zoom_pos = target_zoom;
    st.focus_pos = target_focus;
    save_state(&st);

    printf(" Done.\n");
    printf(">>> Smooth Zoom Complete: Positioned at %.1f mm (Zoom: %d, Focus: %d) <<<\n",
           zoom_to_focal_mm(st.zoom_pos), st.zoom_pos, st.focus_pos);
    return 0;
}

int set_zoom_parfocal(const char *bus, unsigned char addr, int target_zoom, int pps) {
    double focal_mm = zoom_to_focal_mm(target_zoom);
    return set_focal_length_smooth(bus, addr, focal_mm, pps);
}

int main(int argc, char **argv) {
    setbuf(stdout, NULL); // Unbuffered stdout
    atexit(release_lock);
    acquire_lock();

    const char *bus = DEFAULT_I2C_BUS;
    unsigned char addr = DEFAULT_I2C_ADDR;
    int pps = DEFAULT_PPS;
    int steps = DEFAULT_STEPS;

    // Check for OpenIPC standard -d, -s, -j flags: "motor -d [u|d|l|r|i] -s [speed/steps]" or "motor -j"
    if (argc >= 2 && argv[1][0] == '-') {
        char direction = 0;
        int json_output = 0;
        int c;
        for (int i = 1; i < argc; i++) {
            if (strcmp(argv[i], "-j") == 0) {
                json_output = 1;
            } else if (strcmp(argv[i], "-d") == 0 && i + 1 < argc) {
                direction = argv[++i][0];
            } else if (strcmp(argv[i], "-s") == 0 && i + 1 < argc) {
                steps = atoi(argv[++i]);
            } else if (strcmp(argv[i], "-p") == 0 && i + 1 < argc) {
                pps = atoi(argv[++i]);
            } else if (strcmp(argv[i], "-b") == 0 && i + 1 < argc) {
                bus = argv[++i];
            }
        }

        if (json_output) {
            LensState st;
            load_state(&st);
            double f_mm = zoom_to_focal_mm(st.zoom_pos);
            printf("{\"zoom\":%d,\"focus\":%d,\"focal_mm\":%.1f,\"zoom_max\":%d,\"focus_max\":%d,\"calibrated\":%d}\n",
                   st.zoom_pos, st.focus_pos, f_mm, LENS_ZOOM_MAX_STEPS, LENS_FOCUS_MAX_STEPS, st.is_calibrated ? 1 : 0);
            return 0;
        }

        if (direction) {
            if (steps <= 0) steps = 120;
            LensState st;
            load_state(&st);
            switch (direction) {
                case 'u': {
                    int target_z = st.zoom_pos + steps;
                    if (target_z > LENS_ZOOM_MAX_STEPS) target_z = LENS_ZOOM_MAX_STEPS;
                    return set_zoom_parfocal(bus, addr, target_z, pps);
                }
                case 'd': {
                    int target_z = st.zoom_pos - steps;
                    if (target_z < 0) target_z = 0;
                    return set_zoom_parfocal(bus, addr, target_z, pps);
                }
                case 'r': return move_focus_tracked(bus, addr, steps, 1, pps); // Fine Focus Near
                case 'l': return move_focus_tracked(bus, addr, steps, 0, pps); // Fine Focus Far
                case 'i': return do_home(bus, addr, pps);                      // Init / Home
                case 's': return 0; // Stop
                default:
                    printf("Unknown direction: %c\n", direction);
                    return 1;
            }
        }
    }

    // OpenIPC Web UI ptz.cgi syntax: "motor <profile_id> <horizontal> <vertical>" (e.g. "motor 1 0 1")
    if (argc == 4 && (argv[1][0] >= '0' && argv[1][0] <= '9')) {
        int h = atoi(argv[2]);
        int v = atoi(argv[3]);
        if (h == 0 && v == 0) {
            return do_home(bus, addr, pps);
        }
        if (v > 0) {
            LensState st;
            load_state(&st);
            int target_z = st.zoom_pos + abs(v) * 120;
            if (target_z > LENS_ZOOM_MAX_STEPS) target_z = LENS_ZOOM_MAX_STEPS;
            return set_zoom_parfocal(bus, addr, target_z, pps); // Parfocal Zoom In
        }
        if (v < 0) {
            LensState st;
            load_state(&st);
            int target_z = st.zoom_pos - abs(v) * 120;
            if (target_z < 0) target_z = 0;
            return set_zoom_parfocal(bus, addr, target_z, pps); // Parfocal Zoom Out
        }
        if (h > 0) return move_focus_tracked(bus, addr, abs(h) * 80, 1, pps); // Right: Fine Focus Near
        if (h < 0) return move_focus_tracked(bus, addr, abs(h) * 80, 0, pps); // Left: Fine Focus Far
        return 0;
    }

    if (argc < 2) {
        LensState st;
        load_state(&st);
        double f_mm = zoom_to_focal_mm(st.zoom_pos);
        printf("CW-TY207135D14 Motorized Optical Lens Controller (Smooth Parfocal Tracking)\n");
        printf("Verified Limits: Zoom = 0..%d steps (10.8mm lead screw), Focus = 0..%d steps\n",
               LENS_ZOOM_MAX_STEPS, LENS_FOCUS_MAX_STEPS);
        printf("Current State:   Zoom = %d / %d (%.1f mm, %.1fx), Focus = %d / %d [Calibrated: %s]\n\n",
               st.zoom_pos, LENS_ZOOM_MAX_STEPS, f_mm, f_mm / 2.7,
               st.focus_pos, LENS_FOCUS_MAX_STEPS,
               st.is_calibrated ? "YES" : "NO");
        printf("Commands:\n");
        printf("  %s home                 # Full mechanical homing & calibration (Lands at 6.3mm)\n", argv[0]);
        printf("  %s status               # Display current zoom, focus, and focal length (mm)\n", argv[0]);
        printf("  %s setfocal <mm>        # Smooth Simultaneous Parfocal Zoom to mm (e.g. 6.3..13.5)\n", argv[0]);
        printf("  %s setzoom <position>   # Move Zoom to absolute step (0..%d)\n", argv[0], LENS_ZOOM_MAX_STEPS);
        printf("  %s setfocus <position>  # Move Focus to absolute step (0..%d)\n", argv[0], LENS_FOCUS_MAX_STEPS);
        printf("  %s zoomin [steps]       # Step Zoom IN (Tele) [default: 500 steps]\n", argv[0]);
        printf("  %s zoomout [steps]      # Step Zoom OUT (Wide) [default: 500 steps]\n", argv[0]);
        printf("  %s focusin [steps]      # Step Focus NEAR [default: 500 steps]\n", argv[0]);
        printf("  %s focusout [steps]     # Step Focus FAR [default: 500 steps]\n", argv[0]);
        printf("  %s reset                # Reset tracked position to (0, 0)\n\n", argv[0]);
        printf("OpenIPC Flag Syntax:\n");
        printf("  %s -d u -s 10           # Zoom IN (Tele)\n", argv[0]);
        printf("  %s -d d -s 10           # Zoom OUT (Wide)\n", argv[0]);
        printf("  %s -d r -s 10           # Focus NEAR\n", argv[0]);
        printf("  %s -d l -s 10           # Focus FAR\n", argv[0]);
        printf("  %s -d i                 # Init / Home\n", argv[0]);
        return 1;
    }

    if (argc >= 3) steps = atoi(argv[2]);
    if (argc >= 4) pps = atoi(argv[3]);
    if (argc >= 5) bus = argv[4];

    const char *cmd = argv[1];
    if (strcmp(cmd, "home") == 0) {
        do_home(bus, addr, pps);
    } else if (strcmp(cmd, "status") == 0) {
        LensState st;
        load_state(&st);
        double f_mm = zoom_to_focal_mm(st.zoom_pos);
        printf("=== CW-TY207135D14 Lens Status ===\n");
        printf("Zoom Position:  %d / %d steps (%.1f mm, %.1fx Optical Zoom)\n", 
               st.zoom_pos, LENS_ZOOM_MAX_STEPS, f_mm, f_mm / 2.7);
        printf("Focus Position: %d / %d steps (%d%%)\n", 
               st.focus_pos, LENS_FOCUS_MAX_STEPS, (st.focus_pos * 100) / LENS_FOCUS_MAX_STEPS);
        printf("Calibrated:     %s\n", st.is_calibrated ? "YES" : "NO");
    } else if (strcmp(cmd, "setfocal") == 0) {
        if (argc < 3) {
            printf("Usage: %s setfocal <focal_length_mm (e.g. 6.3, 8.0, 10.0, 13.5)>\n", argv[0]);
            return 1;
        }
        double f_mm = atof(argv[2]);
        set_focal_length_smooth(bus, addr, f_mm, pps);
    } else if (strcmp(cmd, "zoomin") == 0) {
        move_zoom_tracked(bus, addr, steps, 1, pps);
    } else if (strcmp(cmd, "zoomout") == 0) {
        move_zoom_tracked(bus, addr, steps, 0, pps);
    } else if (strcmp(cmd, "focusin") == 0) {
        move_focus_tracked(bus, addr, steps, 1, pps);
    } else if (strcmp(cmd, "focusout") == 0) {
        move_focus_tracked(bus, addr, steps, 0, pps);
    } else if (strcmp(cmd, "setzoom") == 0) {
        set_zoom_absolute(bus, addr, steps, pps);
    } else if (strcmp(cmd, "setfocus") == 0) {
        set_focus_absolute(bus, addr, steps, pps);
    } else if (strcmp(cmd, "reset") == 0) {
        LensState st = { .zoom_pos = 0, .focus_pos = 0, .is_calibrated = 0 };
        save_state(&st);
        printf("Tracked position reset to (0, 0).\n");
    } else {
        printf("Unknown command: %s\n", cmd);
        return 1;
    }
    return 0;
}
