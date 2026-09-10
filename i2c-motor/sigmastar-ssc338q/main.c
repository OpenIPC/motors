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
#include <getopt.h>

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

int acquire_lock(void) {
    g_lock_fd = open(LOCK_FILE, O_CREAT | O_RDWR, 0666);
    if (g_lock_fd < 0) {
        perror("Failed to open lock file " LOCK_FILE);
        return -1;
    }
    if (flock(g_lock_fd, LOCK_EX | LOCK_NB) < 0) {
        fprintf(stderr, "motor: device busy (locked by another process)\n");
        close(g_lock_fd);
        g_lock_fd = -1;
        return -1;
    }
    return 0;
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
int set_zoom_smooth(const char *bus, unsigned char addr, int target_zoom, int pps);
int set_zoom_parfocal(const char *bus, unsigned char addr, int target_zoom, int pps);
int set_focal_length_smooth(const char *bus, unsigned char addr, double focal_mm, int pps);
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
        if (fscanf(f, "%d %d %d", &st->zoom_pos, &st->focus_pos, &st->is_calibrated) != 3) {
            st->zoom_pos = LENS_ZOOM_HOME_POS;
            st->focus_pos = LENS_FOCUS_HOME_POS;
            st->is_calibrated = 0;
        }
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

static int check_calibration(const LensState *st, const char *prog_name) {
    if (!st->is_calibrated) {
        fprintf(stderr, "Error: Lens position is uncalibrated after reboot.\n"
                        "Please run '%s home' first to establish optical baseline.\n", prog_name);
        return 0;
    }
    return 1;
}

int riu_init_hardware(void) {
    static int initialized = 0;
    if (initialized) return 0;
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        perror("Failed to open /dev/mem for RIU init");
        return -1;
    }
    // Bank 0x111B Offset 0x06 (Physical 0x1F000000 + 0x111B*0x200 + 0x06*4 = 0x1F223618):
    // Motor driver power rail clock/power gate register
    void *map = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x1F223000);
    if (map == MAP_FAILED) {
        perror("Failed to mmap RIU register 0x1F223000");
        close(fd);
        return -1;
    }
    volatile uint16_t *reg = (volatile uint16_t *)((uint8_t *)map + 0x618); // Bank 0x111B off 0x06
    // Clear gate bit (bit 0) while preserving remaining register bits
    *reg &= ~0x0001;
    munmap(map, 0x1000);
    close(fd);
    initialized = 1;
    return 0;
}

int i2c_write(int fd, unsigned char addr, unsigned char reg, unsigned char val) {
    if (riu_init_hardware() < 0) {
        return -1;
    }
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
    if (ioctl(fd, I2C_RDWR, &rdwr) < 0) {
        return -1;
    }
    return 0;
}

static inline int clamp_pps(int pps) {
    if (pps < 32) return 32;
    if (pps > 16383) return 16383;
    return pps;
}

int set_channel_speeds(int fd, unsigned char addr, int pps_focus, int pps_zoom) {
    pps_focus = clamp_pps(pps_focus);
    pps_zoom = clamp_pps(pps_zoom);

    unsigned int div_f = 24000000 / pps_focus;
    unsigned int div_z = 24000000 / pps_zoom;

    // Channel 1: Focus Speed (Regs 0x01, 0x02)
    if (i2c_write(fd, addr, 0x01, (div_f >> 7) & 0xFF) < 0) return -1;
    if (i2c_write(fd, addr, 0x02, 0x80 | (div_f >> 15)) < 0) return -1;

    // Channel 2: Zoom Speed (Regs 0x05, 0x06)
    if (i2c_write(fd, addr, 0x05, (div_z >> 7) & 0xFF) < 0) return -1;
    if (i2c_write(fd, addr, 0x06, 0x80 | (div_z >> 15)) < 0) return -1;

    return 0;
}

// Single-packet hardware move for Channel 2 (ZOOM: Regs 0x07, 0x08, Trigger 0x4F)
int raw_step_zoom_chunk(const char *bus, unsigned char addr, int chunk_steps, int dir, int pps) {
    if (chunk_steps <= 0) return 0;
    if (chunk_steps > 3500) chunk_steps = 3500;
    pps = clamp_pps(pps);

    int fd = open(bus, O_RDWR);
    if (fd < 0) {
        perror(bus);
        return -1;
    }

    // Wake chip & enable excitation
    if (i2c_write(fd, addr, 0x00, 0x01) < 0 ||
        i2c_write(fd, addr, 0x0A, 0x08) < 0 ||
        set_channel_speeds(fd, addr, pps, pps) < 0) {
        goto fail;
    }

    unsigned char z_lo = chunk_steps & 0xFF;
    unsigned char z_hi = (dir ? 0xC0 : 0x80) | ((chunk_steps >> 8) & 0x0F);

    if (i2c_write(fd, addr, 0x03, 0x00) < 0 ||
        i2c_write(fd, addr, 0x04, 0x00) < 0 ||
        i2c_write(fd, addr, 0x07, z_lo) < 0 ||
        i2c_write(fd, addr, 0x08, z_hi) < 0 ||
        i2c_write(fd, addr, 0x09, 0x4F) < 0) { // Trigger Channel 2 (Zoom)
        goto fail;
    }

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

fail:
    // Attempt coil shutdown on failure
    i2c_write(fd, addr, 0x0A, 0x00);
    i2c_write(fd, addr, 0x00, 0x00);
    close(fd);
    return -1;
}

// Single-packet hardware move for Channel 1 (FOCUS: Regs 0x03, 0x04, Trigger 0x8F)
int raw_step_focus_chunk(const char *bus, unsigned char addr, int chunk_steps, int dir, int pps) {
    if (chunk_steps <= 0) return 0;
    if (chunk_steps > 3500) chunk_steps = 3500;
    pps = clamp_pps(pps);

    int fd = open(bus, O_RDWR);
    if (fd < 0) {
        perror(bus);
        return -1;
    }

    // Wake chip & enable excitation
    if (i2c_write(fd, addr, 0x00, 0x01) < 0 ||
        i2c_write(fd, addr, 0x0A, 0x08) < 0 ||
        set_channel_speeds(fd, addr, pps, pps) < 0) {
        goto fail;
    }

    int hw_dir = dir ? 0 : 1; // Invert hardware bitmask for Focus axis
    unsigned char f_lo = chunk_steps & 0xFF;
    unsigned char f_hi = (hw_dir ? 0xC0 : 0x80) | ((chunk_steps >> 8) & 0x0F);

    if (i2c_write(fd, addr, 0x07, 0x00) < 0 ||
        i2c_write(fd, addr, 0x08, 0x00) < 0 ||
        i2c_write(fd, addr, 0x03, f_lo) < 0 ||
        i2c_write(fd, addr, 0x04, f_hi) < 0 ||
        i2c_write(fd, addr, 0x09, 0x8F) < 0) { // Trigger Channel 1 (Focus)
        goto fail;
    }

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

fail:
    // Attempt coil shutdown on failure
    i2c_write(fd, addr, 0x0A, 0x00);
    i2c_write(fd, addr, 0x00, 0x00);
    close(fd);
    return -1;
}

// Synchronized Simultaneous Dual-Axis Hardware Movement (Trigger 0xCF)
int raw_step_dual_sync(const char *bus, unsigned char addr, int z_steps, int z_dir, int f_steps, int f_dir, int base_pps) {
    if (z_steps <= 0 && f_steps <= 0) return 0;
    if (z_steps > 3500) z_steps = 3500;
    if (f_steps > 3500) f_steps = 3500;
    base_pps = clamp_pps(base_pps);

    int fd = open(bus, O_RDWR);
    if (fd < 0) {
        perror(bus);
        return -1;
    }

    // Wake chip & enable excitation
    if (i2c_write(fd, addr, 0x00, 0x01) < 0 ||
        i2c_write(fd, addr, 0x0A, 0x08) < 0) {
        goto fail;
    }

    // Calculate proportional speeds so both motors start and finish together
    int max_steps = (z_steps > f_steps) ? z_steps : f_steps;
    int pps_z = base_pps;
    int pps_f = base_pps;
    if (max_steps > 0) {
        if (z_steps > 0) pps_z = clamp_pps((base_pps * z_steps) / max_steps);
        if (f_steps > 0) pps_f = clamp_pps((base_pps * f_steps) / max_steps);
    }
    if (set_channel_speeds(fd, addr, pps_f, pps_z) < 0) {
        goto fail;
    }

    int hw_f_dir = f_dir ? 0 : 1; // Invert hardware bitmask for Focus axis
    unsigned char f_lo = f_steps & 0xFF;
    unsigned char f_hi = (hw_f_dir ? 0xC0 : 0x80) | ((f_steps >> 8) & 0x0F);

    unsigned char z_lo = z_steps & 0xFF;
    unsigned char z_hi = (z_dir ? 0xC0 : 0x80) | ((z_steps >> 8) & 0x0F);

    if (i2c_write(fd, addr, 0x03, f_lo) < 0 ||
        i2c_write(fd, addr, 0x04, f_hi) < 0 ||
        i2c_write(fd, addr, 0x07, z_lo) < 0 ||
        i2c_write(fd, addr, 0x08, z_hi) < 0) {
        goto fail;
    }

    unsigned char trigger = 0xCF;
    if (z_steps == 0) trigger = 0x8F;
    else if (f_steps == 0) trigger = 0x4F;
    if (i2c_write(fd, addr, 0x09, trigger) < 0) {
        goto fail;
    }

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

fail:
    i2c_write(fd, addr, 0x0A, 0x00);
    i2c_write(fd, addr, 0x00, 0x00);
    close(fd);
    return -1;
}

// Move Zoom with automatic multi-chunk handling
int move_zoom(const char *bus, unsigned char addr, int total_steps, int dir, int pps) {
    printf(">>> ZOOM: Moving %d steps %s @ %d PPS...",
           total_steps, dir ? "IN (Tele)" : "OUT (Wide)", pps);
    fflush(stdout);

    int remaining = total_steps;
    while (remaining > 0) {
        int chunk = (remaining > 3000) ? 3000 : remaining;
        if (raw_step_zoom_chunk(bus, addr, chunk, dir, pps) < 0) {
            printf(" Failed (I2C error).\n");
            return -1;
        }
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
        if (raw_step_focus_chunk(bus, addr, chunk, dir, pps) < 0) {
            printf(" Failed (I2C error).\n");
            return -1;
        }
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
    return (int)round(((focal_mm - 2.7) / 10.8) * (double)LENS_ZOOM_MAX_STEPS);
}

int do_home(const char *bus, unsigned char addr, int pps) {
    printf("=================================================================\n");
    printf("  Executing Optical Homing Calibration (CW-TY207135D14 5x)\n");
    printf("  Mechanical Limits: Zoom = 0..%d steps, Focus = 0..%d steps\n", 
           LENS_ZOOM_MAX_STEPS, LENS_FOCUS_MAX_STEPS);
    printf("  Calibrated Usable Home: Zoom = %d (6.3mm), Focus = %d (Sharp)\n",
           LENS_ZOOM_HOME_POS, LENS_FOCUS_HOME_POS);
    printf("=================================================================\n");

    printf("[1/4] Homing Zoom Axis to 0 (Full Wide mechanical hard-stop)...\n");
    fflush(stdout);
    if (move_zoom(bus, addr, LENS_ZOOM_MAX_STEPS + 500, 0, pps) < 0) return -1;

    printf("[2/4] Homing Focus Axis to 0 (Infinity mechanical hard-stop)...\n");
    fflush(stdout);
    if (move_focus(bus, addr, LENS_FOCUS_MAX_STEPS + 500, 0, pps) < 0) return -1;

    printf("[3/4] Positioning Zoom to calibrated Wide optical angle (%d steps, %.1f mm)...\n", 
           LENS_ZOOM_HOME_POS, zoom_to_focal_mm(LENS_ZOOM_HOME_POS));
    fflush(stdout);
    if (move_zoom(bus, addr, LENS_ZOOM_HOME_POS, 1, pps) < 0) return -1;

    printf("[4/4] Setting Focus to calibrated sharp focal plane (%d steps NEAR)...\n", LENS_FOCUS_HOME_POS);
    fflush(stdout);
    if (move_focus(bus, addr, LENS_FOCUS_HOME_POS, 1, pps) < 0) return -1;

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
    if (target < LENS_ZOOM_HOME_POS) target = LENS_ZOOM_HOME_POS;

    int actual_steps = abs(target - st.zoom_pos);
    if (actual_steps == 0) {
        printf("Zoom already at %s optical limit (%d / %d steps)\n", 
               (target >= LENS_ZOOM_MAX_STEPS) ? "TELE" : "WIDE", st.zoom_pos, LENS_ZOOM_MAX_STEPS);
        return 0;
    }

    int actual_dir = (target >= st.zoom_pos) ? 1 : 0;
    if (move_zoom(bus, addr, actual_steps, actual_dir, pps) < 0) {
        return -1;
    }

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
               (target >= LENS_FOCUS_MAX_STEPS) ? "NEAR" : "FAR", st.focus_pos, LENS_FOCUS_MAX_STEPS);
        return 0;
    }

    int actual_dir = (target >= st.focus_pos) ? 1 : 0;
    if (move_focus(bus, addr, actual_steps, actual_dir, pps) < 0) {
        return -1;
    }

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
    if (target_pos < LENS_ZOOM_HOME_POS) target_pos = LENS_ZOOM_HOME_POS;

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
int set_zoom_smooth(const char *bus, unsigned char addr, int target_zoom, int pps) {
    LensState st;
    load_state(&st);

    if (target_zoom < LENS_ZOOM_HOME_POS) target_zoom = LENS_ZOOM_HOME_POS;
    if (target_zoom > LENS_ZOOM_MAX_STEPS) target_zoom = LENS_ZOOM_MAX_STEPS;
    int target_focus = get_calibrated_focus(target_zoom);

    int start_zoom = st.zoom_pos;
    int start_focus = st.focus_pos;

    int total_z_delta = target_zoom - start_zoom;
    if (total_z_delta == 0) {
        // Just fine-tune focus to target calibrated plane
        return set_focus_absolute(bus, addr, target_focus, pps);
    }

    double target_focal_mm = zoom_to_focal_mm(target_zoom);
    printf("=================================================================\n");
    printf("  [PARFOCAL SMOOTH TRACKING] Target: %.1f mm (%.1fx Zoom)\n", 
           target_focal_mm, target_focal_mm / 2.7);
    printf("  Trajectory: Zoom %d -> %d | Focus %d -> %d\n",
           start_zoom, target_zoom, start_focus, target_focus);
    printf("=================================================================\n");
    printf("Tracking along optical curve...");
    fflush(stdout);

    // Scale segment count to delta: ~1 segment per 100 steps, clamped 1..25
    int num_segments = abs(total_z_delta) / 100;
    if (num_segments < 1) num_segments = 1;
    if (num_segments > 25) num_segments = 25;

    double dz_seg = (double)total_z_delta / (double)num_segments;
    double df_total = (double)(target_focus - start_focus);
    double df_seg = df_total / (double)num_segments;

    double cur_z = (double)start_zoom;
    double cur_f = (double)start_focus;

    for (int i = 0; i < num_segments; i++) {
        double next_z = cur_z + dz_seg;
        double next_f = cur_f + df_seg;

        int z_steps = abs((int)round(next_z) - (int)round(cur_z));
        int z_dir = (dz_seg > 0) ? 1 : 0;

        int f_steps = abs((int)round(next_f) - (int)round(cur_f));
        int f_dir = (df_seg > 0) ? 1 : 0;

        if (z_steps > 0 || f_steps > 0) {
            if (raw_step_dual_sync(bus, addr, z_steps, z_dir, f_steps, f_dir, pps) < 0) {
                printf(" Failed (I2C error).\n");
                return -1;
            }
        }

        cur_z = next_z;
        cur_f = next_f;
    }

    st.zoom_pos = target_zoom;
    st.focus_pos = target_focus;
    st.is_calibrated = 1;
    save_state(&st);

    printf(" Done.\n");
    printf(">>> Smooth Zoom Complete: Positioned at %.1f mm (Zoom: %d, Focus: %d) <<<\n",
           zoom_to_focal_mm(st.zoom_pos), st.zoom_pos, st.focus_pos);
    return 0;
}

int set_focal_length_smooth(const char *bus, unsigned char addr, double focal_mm, int pps) {
    int target_zoom = focal_mm_to_zoom(focal_mm);
    return set_zoom_smooth(bus, addr, target_zoom, pps);
}

int set_zoom_parfocal(const char *bus, unsigned char addr, int target_zoom, int pps) {
    return set_zoom_smooth(bus, addr, target_zoom, pps);
}

int main(int argc, char **argv) {
    setbuf(stdout, NULL); // Unbuffered stdout

    const char *bus = DEFAULT_I2C_BUS;
    unsigned char addr = DEFAULT_I2C_ADDR;
    int pps = DEFAULT_PPS;
    int steps = DEFAULT_STEPS;

    // Check for OpenIPC standard flags (-d, -s, -p, -n, -x, -y, -j, -i, -b)
    if (argc >= 2 && argv[1][0] == '-') {
        char direction = 0;
        int json_output = 0;
        int opt;

        while ((opt = getopt(argc, argv, "d:s:p:n:x:y:jib:")) != -1) {
            switch (opt) {
                case 'd':
                    direction = optarg[0];
                    break;
                case 's': {
                    int val = atoi(optarg);
                    if (val <= 100) {
                        pps = clamp_pps(val * 80); // Speed 10 -> 800 PPS default
                    } else {
                        pps = clamp_pps(val);
                    }
                    break;
                }
                case 'p':
                    pps = clamp_pps(atoi(optarg));
                    break;
                case 'n':
                case 'x':
                case 'y':
                    steps = atoi(optarg);
                    break;
                case 'j':
                    json_output = 1;
                    break;
                case 'i':
                    json_output = 2;
                    break;
                case 'b':
                    bus = optarg;
                    break;
                default:
                    fprintf(stderr, "Usage: %s [-d u|d|r|l|i|s] [-s speed] [-p pps] [-n steps] [-j|-i] [-b bus]\n", argv[0]);
                    return 1;
            }
        }

        if (json_output) {
            LensState st;
            load_state(&st);
            double f_mm = zoom_to_focal_mm(st.zoom_pos);
            if (json_output == 2) {
                printf("{\"status\":\"0\",\"xpos\":\"%d\",\"ypos\":\"%d\",\"xmax\":\"%d\",\"ymax\":\"%d\",\"speed\":\"%d\",\"zoom\":%d,\"focus\":%d,\"focal_mm\":%.1f,\"zoom_max\":%d,\"focus_max\":%d,\"calibrated\":%d}\n",
                       st.focus_pos, st.zoom_pos, LENS_FOCUS_MAX_STEPS, LENS_ZOOM_MAX_STEPS, pps,
                       st.zoom_pos, st.focus_pos, f_mm, LENS_ZOOM_MAX_STEPS, LENS_FOCUS_MAX_STEPS, st.is_calibrated ? 1 : 0);
            } else {
                printf("{\"status\":\"0\",\"xpos\":\"%d\",\"ypos\":\"%d\",\"speed\":\"%d\",\"zoom\":%d,\"focus\":%d,\"focal_mm\":%.1f,\"zoom_max\":%d,\"focus_max\":%d,\"calibrated\":%d}\n",
                       st.focus_pos, st.zoom_pos, pps,
                       st.zoom_pos, st.focus_pos, f_mm, LENS_ZOOM_MAX_STEPS, LENS_FOCUS_MAX_STEPS, st.is_calibrated ? 1 : 0);
            }
            return 0;
        }

        if (direction) {
            if (direction == 's') {
                // Stop command: synchronous execution means motor is already idle
                return 0;
            }

            if (acquire_lock() < 0) return 1;
            atexit(release_lock);

            if (direction == 'i') {
                return (do_home(bus, addr, pps) < 0) ? 1 : 0;
            }

            LensState st;
            load_state(&st);
            if (!check_calibration(&st, argv[0])) {
                return 1;
            }

            if (steps <= 0) steps = 300;
            switch (direction) {
                case 'u': {
                    int target_z = st.zoom_pos + steps;
                    if (target_z > LENS_ZOOM_MAX_STEPS) target_z = LENS_ZOOM_MAX_STEPS;
                    return (set_zoom_parfocal(bus, addr, target_z, pps) < 0) ? 1 : 0;
                }
                case 'd': {
                    int target_z = st.zoom_pos - steps;
                    if (target_z < LENS_ZOOM_HOME_POS) target_z = LENS_ZOOM_HOME_POS;
                    return (set_zoom_parfocal(bus, addr, target_z, pps) < 0) ? 1 : 0;
                }
                case 'r': return (move_focus_tracked(bus, addr, steps, 1, pps) < 0) ? 1 : 0; // Fine Focus Near
                case 'l': return (move_focus_tracked(bus, addr, steps, 0, pps) < 0) ? 1 : 0; // Fine Focus Far
                default:
                    fprintf(stderr, "Unknown direction: %c\n", direction);
                    return 1;
            }
        }
    }

    // OpenIPC Web UI ptz.cgi syntax: "motor <profile_id> <horizontal> <vertical>" (e.g. "motor 1 0 1")
    if (argc == 4 && (argv[1][0] >= '0' && argv[1][0] <= '9')) {
        int h = atoi(argv[2]);
        int v = atoi(argv[3]);
        if (h == 0 && v == 0) {
            if (acquire_lock() < 0) return 1;
            atexit(release_lock);
            return (do_home(bus, addr, pps) < 0) ? 1 : 0;
        }

        LensState st;
        load_state(&st);
        if (!check_calibration(&st, argv[0])) {
            return 1;
        }

        if (acquire_lock() < 0) return 1;
        atexit(release_lock);

        if (v > 0) {
            int target_z = st.zoom_pos + abs(v) * 300;
            if (target_z > LENS_ZOOM_MAX_STEPS) target_z = LENS_ZOOM_MAX_STEPS;
            return (set_zoom_parfocal(bus, addr, target_z, pps) < 0) ? 1 : 0; // Parfocal Zoom In
        }
        if (v < 0) {
            int target_z = st.zoom_pos - abs(v) * 300;
            if (target_z < LENS_ZOOM_HOME_POS) target_z = LENS_ZOOM_HOME_POS;
            return (set_zoom_parfocal(bus, addr, target_z, pps) < 0) ? 1 : 0; // Parfocal Zoom Out
        }
        if (h > 0) return (move_focus_tracked(bus, addr, abs(h) * 100, 1, pps) < 0) ? 1 : 0; // Right: Fine Focus Near
        if (h < 0) return (move_focus_tracked(bus, addr, abs(h) * 100, 0, pps) < 0) ? 1 : 0; // Left: Fine Focus Far
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
        printf("  %s reset                # Reset tracked position to home baseline\n\n", argv[0]);
        printf("OpenIPC Flag Syntax:\n");
        printf("  %s -d u -s 10           # Zoom IN (Parfocal)\n", argv[0]);
        printf("  %s -d d -s 10           # Zoom OUT (Parfocal)\n", argv[0]);
        printf("  %s -d r -s 5            # Focus NEAR\n", argv[0]);
        printf("  %s -d l -s 5            # Focus FAR\n", argv[0]);
        printf("  %s -d i                 # Init / Home\n", argv[0]);
        printf("  %s -d s                 # Stop\n", argv[0]);
        return 1;
    }

    if (argc >= 3) steps = atoi(argv[2]);
    if (argc >= 4) pps = clamp_pps(atoi(argv[3]));
    if (argc >= 5) bus = argv[4];

    const char *cmd = argv[1];
    if (strcmp(cmd, "status") == 0) {
        LensState st;
        load_state(&st);
        double f_mm = zoom_to_focal_mm(st.zoom_pos);
        printf("=== CW-TY207135D14 Lens Status ===\n");
        printf("Zoom Position:  %d / %d steps (%.1f mm, %.1fx Optical Zoom)\n", 
               st.zoom_pos, LENS_ZOOM_MAX_STEPS, f_mm, f_mm / 2.7);
        printf("Focus Position: %d / %d steps (%d%%)\n", 
               st.focus_pos, LENS_FOCUS_MAX_STEPS, (st.focus_pos * 100) / LENS_FOCUS_MAX_STEPS);
        printf("Calibrated:     %s\n", st.is_calibrated ? "YES" : "NO");
        return 0;
    } else if (strcmp(cmd, "reset") == 0) {
        LensState st = { .zoom_pos = LENS_ZOOM_HOME_POS, .focus_pos = LENS_FOCUS_HOME_POS, .is_calibrated = 0 };
        save_state(&st);
        printf("Tracked position reset to home baseline (%d, %d), uncalibrated.\n",
               LENS_ZOOM_HOME_POS, LENS_FOCUS_HOME_POS);
        return 0;
    }

    if (acquire_lock() < 0) return 1;
    atexit(release_lock);

    if (strcmp(cmd, "home") == 0) {
        return (do_home(bus, addr, pps) < 0) ? 1 : 0;
    }

    LensState st;
    load_state(&st);
    if (!check_calibration(&st, argv[0])) {
        return 1;
    }

    if (strcmp(cmd, "setfocal") == 0) {
        if (argc < 3) {
            fprintf(stderr, "Usage: %s setfocal <focal_length_mm (e.g. 6.3, 8.0, 10.0, 13.5)>\n", argv[0]);
            return 1;
        }
        double f_mm = atof(argv[2]);
        return (set_focal_length_smooth(bus, addr, f_mm, pps) < 0) ? 1 : 0;
    } else if (strcmp(cmd, "zoomin") == 0) {
        return (move_zoom_tracked(bus, addr, steps, 1, pps) < 0) ? 1 : 0;
    } else if (strcmp(cmd, "zoomout") == 0) {
        return (move_zoom_tracked(bus, addr, steps, 0, pps) < 0) ? 1 : 0;
    } else if (strcmp(cmd, "focusin") == 0) {
        return (move_focus_tracked(bus, addr, steps, 1, pps) < 0) ? 1 : 0;
    } else if (strcmp(cmd, "focusout") == 0) {
        return (move_focus_tracked(bus, addr, steps, 0, pps) < 0) ? 1 : 0;
    } else if (strcmp(cmd, "setzoom") == 0) {
        return (set_zoom_absolute(bus, addr, steps, pps) < 0) ? 1 : 0;
    } else if (strcmp(cmd, "setfocus") == 0) {
        return (set_focus_absolute(bus, addr, steps, pps) < 0) ? 1 : 0;
    } else {
        fprintf(stderr, "Unknown command: %s\n", cmd);
        return 1;
    }
    return 0;
}
