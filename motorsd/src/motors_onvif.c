#define _POSIX_C_SOURCE 200809L

#include <arpa/inet.h>
#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <netinet/in.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include <json-c/json.h>

#include "libmotors.h"

/*
 * motors-onvif — lightweight ONVIF Profile-S/T PTZ bridge daemon for motorsd.
 *
 * Exposes /onvif/device_service, /onvif/media_service, and /onvif/ptz_service
 * with full ContinuousMove, RelativeMove (TranslationSpaceFov + Zoom),
 * AbsoluteMove (Zoom + PanTilt), Stop, GetStatus (MoveStatus + StatusPosition),
 * and GetPresets / GotoPreset / SetPreset / RemovePreset / GotoHomePosition
 * backed directly by motorsd (/run/motorsd.sock) and /etc/ptz_presets.tsv.
 */

#define PTZ_POS_TMP "/tmp/ptz_pos.conf"
#define PTZ_POS_ETC "/etc/ptz_pos.conf"
#define PTZ_LIVE_TMP "/tmp/ptz_live.conf"
#define AF_CANCEL_FILE "/tmp/majestic-af.cancel"
#define BOOT_HOME_FILE "/etc/ptz_boot_home"
#define MAJESTIC_YAML "/etc/majestic.yaml"
#define ONVIF_CONF_FILE "/etc/motorsd/onvif.conf"

#define MAX_REQ_BYTES 65536
#define MAX_RESP_BYTES 65536
#define MAX_PRESETS 32

struct onvif_config {
    int port;
    char socket_path[256];
    char presets_path[256];
    char config_path[256];
    char user[64];
    char pass[128];
    double pan_scale_ms;
    double tilt_scale_ms;
    double max_zoom;
    double pan_range_ms;
    double tilt_range_ms;
    unsigned af_settle_ms;
    bool post_zoom_af;
    bool invert_pan;
    bool invert_tilt;
    bool debug;
};

struct preset_entry {
    int id;
    char name[64];
    double mag;
    bool has_mag;
    int run_af;
    int zoom_restore;
    long pan_ms;
    long tilt_ms;
    long focus_ms;
    bool has_focus;
};

enum job_kind {
    JOB_NONE = 0,
    JOB_CONTINUOUS,
    JOB_RELATIVE,
    JOB_ABSOLUTE,
    JOB_PRESET,
    JOB_FOCUS,
};

struct ptz_job {
    enum job_kind kind;
    unsigned gen;
    double vx, vy, vz;
    bool has_pt;
    bool has_zoom;
    unsigned timeout_ms;
    struct preset_entry preset;
};

static struct onvif_config g_cfg = {
    .port = 8899,
    .socket_path = "/run/motorsd.sock",
    .presets_path = "/etc/ptz_presets.tsv",
    .config_path = ONVIF_CONF_FILE,
    .user = "",
    .pass = "",
    .pan_scale_ms = 10600.0,
    .tilt_scale_ms = 6400.0,
    .max_zoom = 57.0,
    .pan_range_ms = 60000.0,
    .tilt_range_ms = 30000.0,
    .af_settle_ms = 1200U,
    .post_zoom_af = true,
    .invert_pan = false,
    .invert_tilt = false,
    .debug = false,
};

static volatile sig_atomic_t g_stop_daemon = 0;

static pthread_mutex_t g_mu = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t g_cv = PTHREAD_COND_INITIALIZER;
static unsigned g_job_gen = 0;
static uint64_t g_cancel_mono_ms = 0;
static bool g_has_pending_job = false;
static struct ptz_job g_pending_job;
static bool g_pt_moving = false;
static bool g_zoom_moving = false;
static double g_cached_mag = 1.0;
static bool g_cached_mag_valid = false;

static void handle_signal(int sig) {
    (void)sig;
    g_stop_daemon = 1;
}

static uint64_t mono_now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000L);
}

static void sleep_ms(unsigned ms) {
    struct timespec ts = {
        .tv_sec = (time_t)(ms / 1000U),
        .tv_nsec = (long)((ms % 1000U) * 1000000L),
    };
    nanosleep(&ts, NULL);
}

static double clamp_d(double v, double lo, double hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void touch_af_cancel(void) {
    FILE *f = fopen(AF_CANCEL_FILE, "w");
    if (f) fclose(f);
}

static void trigger_local_af(void) {
    unlink(AF_CANCEL_FILE);
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return;
    struct timeval tv = { .tv_sec = 1, .tv_usec = 0 };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    struct sockaddr_in sa;
    memset(&sa, 0, sizeof(sa));
    sa.sin_family = AF_INET;
    sa.sin_port = htons(80);
    sa.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (connect(fd, (struct sockaddr *)&sa, sizeof(sa)) == 0) {
        const char req[] = "GET /autofocus HTTP/1.1\r\nHost: 127.0.0.1\r\nConnection: close\r\n\r\n";
        (void)send(fd, req, sizeof(req) - 1, MSG_NOSIGNAL);
        char buf[128];
        (void)recv(fd, buf, sizeof(buf), 0);
    }
    close(fd);
}

/* ---------------- SHA-1 & Base64 (for WSSE PasswordDigest & Basic auth) ---------------- */

struct sha1_ctx {
    uint32_t state[5];
    uint64_t count;
    uint8_t buf[64];
};

static uint32_t rol32(uint32_t v, unsigned bits) {
    return (v << bits) | (v >> (32U - bits));
}

static void sha1_transform(uint32_t state[5], const uint8_t block[64]) {
    uint32_t w[80];
    for (int i = 0; i < 16; i++) {
        w[i] = ((uint32_t)block[i * 4] << 24) |
               ((uint32_t)block[i * 4 + 1] << 16) |
               ((uint32_t)block[i * 4 + 2] << 8) |
               ((uint32_t)block[i * 4 + 3]);
    }
    for (int i = 16; i < 80; i++) {
        w[i] = rol32(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
    }
    uint32_t a = state[0], b = state[1], c = state[2], d = state[3], e = state[4];
    for (int i = 0; i < 80; i++) {
        uint32_t f, k;
        if (i < 20) {
            f = (b & c) | ((~b) & d);
            k = 0x5A827999U;
        } else if (i < 40) {
            f = b ^ c ^ d;
            k = 0x6ED9EBA1U;
        } else if (i < 60) {
            f = (b & c) | (b & d) | (c & d);
            k = 0x8F1BBCDCU;
        } else {
            f = b ^ c ^ d;
            k = 0xCA62C1D6U;
        }
        uint32_t temp = rol32(a, 5) + f + e + k + w[i];
        e = d;
        d = c;
        c = rol32(b, 30);
        b = a;
        a = temp;
    }
    state[0] += a;
    state[1] += b;
    state[2] += c;
    state[3] += d;
    state[4] += e;
}

static void sha1_init(struct sha1_ctx *ctx) {
    ctx->state[0] = 0x67452301U;
    ctx->state[1] = 0xEFCDAB89U;
    ctx->state[2] = 0x98BADCFEU;
    ctx->state[3] = 0x10325476U;
    ctx->state[4] = 0xC3D2E1F0U;
    ctx->count = 0;
}

static void sha1_update(struct sha1_ctx *ctx, const uint8_t *data, size_t len) {
    size_t idx = (size_t)(ctx->count & 63U);
    ctx->count += len;
    size_t i = 0;
    if (idx > 0 && idx + len >= 64) {
        memcpy(ctx->buf + idx, data, 64 - idx);
        sha1_transform(ctx->state, ctx->buf);
        i = 64 - idx;
        idx = 0;
    }
    while (i + 64 <= len) {
        sha1_transform(ctx->state, data + i);
        i += 64;
    }
    if (i < len) {
        memcpy(ctx->buf + idx, data + i, len - i);
    }
}

static void sha1_final(struct sha1_ctx *ctx, uint8_t digest[20]) {
    uint64_t bits = ctx->count * 8ULL;
    uint8_t pad = 0x80;
    sha1_update(ctx, &pad, 1);
    uint8_t zero = 0x00;
    while ((ctx->count & 63U) != 56U) {
        sha1_update(ctx, &zero, 1);
    }
    uint8_t len_be[8];
    for (int i = 0; i < 8; i++) {
        len_be[i] = (uint8_t)((bits >> ((7 - i) * 8)) & 0xFFU);
    }
    sha1_update(ctx, len_be, 8);
    for (int i = 0; i < 5; i++) {
        digest[i * 4] = (uint8_t)((ctx->state[i] >> 24) & 0xFFU);
        digest[i * 4 + 1] = (uint8_t)((ctx->state[i] >> 16) & 0xFFU);
        digest[i * 4 + 2] = (uint8_t)((ctx->state[i] >> 8) & 0xFFU);
        digest[i * 4 + 3] = (uint8_t)(ctx->state[i] & 0xFFU);
    }
}

static int b64_val(int c) {
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a' + 26;
    if (c >= '0' && c <= '9') return c - '0' + 52;
    if (c == '+') return 62;
    if (c == '/') return 63;
    return -1;
}

static size_t b64_decode(const char *in, uint8_t *out, size_t out_max) {
    size_t out_len = 0;
    uint32_t val = 0;
    int bits = -8;
    for (const unsigned char *p = (const unsigned char *)in; *p; p++) {
        if (isspace(*p)) continue;
        if (*p == '=') break;
        int d = b64_val(*p);
        if (d < 0) return 0;
        val = (val << 6) | (uint32_t)d;
        bits += 6;
        if (bits >= 0) {
            if (out_len >= out_max) break;
            out[out_len++] = (uint8_t)((val >> bits) & 0xFFU);
            bits -= 8;
        }
    }
    return out_len;
}

static void b64_encode(const uint8_t *in, size_t len, char *out, size_t out_max) {
    static const char tbl[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    size_t j = 0;
    for (size_t i = 0; i < len && j + 4 < out_max; i += 3) {
        uint32_t a = in[i];
        uint32_t b = (i + 1 < len) ? in[i + 1] : 0;
        uint32_t c = (i + 2 < len) ? in[i + 2] : 0;
        uint32_t triple = (a << 16) | (b << 8) | c;
        out[j++] = tbl[(triple >> 18) & 0x3FU];
        out[j++] = tbl[(triple >> 12) & 0x3FU];
        out[j++] = (i + 1 < len) ? tbl[(triple >> 6) & 0x3FU] : '=';
        out[j++] = (i + 2 < len) ? tbl[triple & 0x3FU] : '=';
    }
    out[j] = '\0';
}

/* ---------------- Position & Presets Persistence ---------------- */

static void ptz_pos_read(long *pan, long *tilt, long *focus) {
    *pan = 0;
    *tilt = 0;
    *focus = 0;
    FILE *f = fopen(PTZ_POS_TMP, "r");
    if (!f) f = fopen(PTZ_POS_ETC, "r");
    if (!f) return;
    char line[128];
    while (fgets(line, sizeof(line), f)) {
        long v = 0;
        if (sscanf(line, "PAN_MS=%ld", &v) == 1) *pan = v;
        else if (sscanf(line, "TILT_MS=%ld", &v) == 1) *tilt = v;
        else if (sscanf(line, "FOCUS_MS=%ld", &v) == 1) *focus = v;
    }
    fclose(f);
}

static void ptz_pos_write(long pan, long tilt, long focus, bool persist) {
    char buf[128];
    int n = snprintf(buf, sizeof(buf), "PAN_MS=%ld\nTILT_MS=%ld\nFOCUS_MS=%ld\n",
                     pan, tilt, focus);
    if (n <= 0 || (size_t)n >= sizeof(buf)) return;
    FILE *f = fopen(PTZ_POS_TMP ".tmp", "w");
    if (f) {
        fwrite(buf, 1, (size_t)n, f);
        fclose(f);
        rename(PTZ_POS_TMP ".tmp", PTZ_POS_TMP);
    }
    if (!persist) return;
    char cur[128] = "";
    FILE *fe = fopen(PTZ_POS_ETC, "r");
    if (fe) {
        size_t r = fread(cur, 1, sizeof(cur) - 1, fe);
        cur[r] = '\0';
        fclose(fe);
    }
    if (strcmp(cur, buf) != 0) {
        FILE *fw = fopen(PTZ_POS_ETC ".tmp", "w");
        if (fw) {
            fwrite(buf, 1, (size_t)n, fw);
            fclose(fw);
            if (rename(PTZ_POS_ETC ".tmp", PTZ_POS_ETC) != 0)
                unlink(PTZ_POS_ETC ".tmp");
        }
    }
}

static void ptz_pos_read_live(long *pan, long *tilt, long *focus, bool *moving) {
    ptz_pos_read(pan, tilt, focus);
    if (moving) *moving = false;
    FILE *f = fopen(PTZ_LIVE_TMP, "r");
    if (!f) return;
    long bp = *pan, bt = *tilt, bf = *focus;
    int dp = 0, dt = 0, df = 0, mv = 0;
    unsigned long long s_ms = 0, e_ms = 0;
    char line[128];
    while (fgets(line, sizeof(line), f)) {
        long lv;
        int iv;
        unsigned long long uv;
        if (sscanf(line, "BASE_PAN=%ld", &lv) == 1) bp = lv;
        else if (sscanf(line, "BASE_TILT=%ld", &lv) == 1) bt = lv;
        else if (sscanf(line, "BASE_FOCUS=%ld", &lv) == 1) bf = lv;
        else if (sscanf(line, "DP=%d", &iv) == 1) dp = iv;
        else if (sscanf(line, "DT=%d", &iv) == 1) dt = iv;
        else if (sscanf(line, "DF=%d", &iv) == 1) df = iv;
        else if (sscanf(line, "MOVING=%d", &iv) == 1) mv = iv;
        else if (sscanf(line, "START_MS=%llu", &uv) == 1) s_ms = uv;
        else if (sscanf(line, "END_MS=%llu", &uv) == 1) e_ms = uv;
    }
    fclose(f);
    uint64_t now = mono_now_ms();
    uint64_t clamped = (e_ms > 0 && now > e_ms) ? e_ms : now;
    long el = (clamped > s_ms) ? (long)(clamped - s_ms) : 0;
    *pan = bp + (long)dp * el;
    *tilt = bt + (long)dt * el;
    *focus = bf + (long)df * el;
    if (moving && mv == 1 && now < e_ms) *moving = true;
}

static void ptz_live_write(long base_pan, long base_tilt, long base_focus,
                           int dp, int dt, int df, uint64_t start_ms, uint64_t end_ms) {
    FILE *f = fopen(PTZ_LIVE_TMP ".tmp", "w");
    if (!f) return;
    fprintf(f,
            "BASE_PAN=%ld\nBASE_TILT=%ld\nBASE_FOCUS=%ld\n"
            "DP=%d\nDT=%d\nDF=%d\n"
            "START_MS=%llu\nEND_MS=%llu\nMOVING=1\n",
            base_pan, base_tilt, base_focus, dp, dt, df,
            (unsigned long long)start_ms, (unsigned long long)end_ms);
    fclose(f);
    rename(PTZ_LIVE_TMP ".tmp", PTZ_LIVE_TMP);
}

static void ptz_live_clear(void) {
    unlink(PTZ_LIVE_TMP);
}

static size_t load_presets(struct preset_entry *out, size_t max_out) {
    FILE *f = fopen(g_cfg.presets_path, "r");
    if (!f) return 0;
    size_t count = 0;
    char line[512];
    while (count < max_out && fgets(line, sizeof(line), f)) {
        size_t len = strlen(line);
        while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r'))
            line[--len] = '\0';
        if (!len || line[0] == '#') continue;

        char *fields[8] = {0};
        int nf = 0;
        char *p = line;
        fields[nf++] = p;
        while (*p && nf < 8) {
            if (*p == '|') {
                *p = '\0';
                fields[nf++] = p + 1;
            }
            p++;
        }
        if (nf < 2) continue;
        int id = atoi(fields[0]);
        if (id < 1 || id > MAX_PRESETS) continue;

        struct preset_entry e;
        memset(&e, 0, sizeof(e));
        e.id = id;
        snprintf(e.name, sizeof(e.name), "%.63s", fields[1]);
        e.run_af = 0;
        e.zoom_restore = 1;
        if (nf >= 3 && fields[2][0] && strcmp(fields[2], "null") != 0) {
            e.mag = atof(fields[2]);
            e.has_mag = (e.mag >= 1.0);
        }
        if (nf >= 4 && fields[3][0]) e.run_af = atoi(fields[3]) ? 1 : 0;
        if (nf >= 5 && fields[4][0]) e.zoom_restore = atoi(fields[4]) ? 1 : 0;
        if (nf >= 6 && fields[5][0]) e.pan_ms = strtol(fields[5], NULL, 10);
        if (nf >= 7 && fields[6][0]) e.tilt_ms = strtol(fields[6], NULL, 10);
        if (nf >= 8 && fields[7][0] && strcmp(fields[7], "null") != 0) {
            e.focus_ms = strtol(fields[7], NULL, 10);
            e.has_focus = true;
        }
        out[count++] = e;
    }
    fclose(f);
    return count;
}

static bool save_presets(const struct preset_entry *list, size_t count) {
    char tmp[300];
    snprintf(tmp, sizeof(tmp), "%s.tmp", g_cfg.presets_path);
    FILE *f = fopen(tmp, "w");
    if (!f) return false;
    for (size_t i = 0; i < count; i++) {
        const struct preset_entry *e = &list[i];
        char mag_buf[32] = "null";
        char foc_buf[32] = "null";
        if (e->has_mag) snprintf(mag_buf, sizeof(mag_buf), "%.1f", e->mag);
        if (e->has_focus) snprintf(foc_buf, sizeof(foc_buf), "%ld", e->focus_ms);
        fprintf(f, "%d|%s|%s|%d|%d|%ld|%ld|%s\n",
                e->id, e->name, mag_buf, e->run_af, e->zoom_restore,
                e->pan_ms, e->tilt_ms, foc_buf);
    }
    fclose(f);
    return rename(tmp, g_cfg.presets_path) == 0;
}

static int read_boot_home_id(void) {
    FILE *f = fopen(BOOT_HOME_FILE, "r");
    if (!f) return 1;
    int id = 1;
    if (fscanf(f, "%d", &id) != 1 || id < 1 || id > MAX_PRESETS) id = 1;
    fclose(f);
    return id;
}

/* ---------------- motorsd Client Helpers ---------------- */

static bool query_zoom_mag(double *mag_out) {
    char err[128] = "";
    struct motors_client *cl = NULL;
    if (motors_open(&cl, g_cfg.socket_path, err, sizeof(err)) != 0) {
        return false;
    }
    char resp[512] = "";
    int rc = motors_request(cl, "{\"version\":1,\"id\":\"onvif\",\"op\":\"status\"}",
                            resp, sizeof(resp), err, sizeof(err));
    motors_close(cl);
    if (rc != 0) return false;

    struct json_object *obj = json_tokener_parse(resp);
    if (!obj) return false;
    struct json_object *zm = NULL;
    bool ok = false;
    if (json_object_object_get_ex(obj, "zoom_magnification", &zm) &&
        (json_object_is_type(zm, json_type_double) || json_object_is_type(zm, json_type_int))) {
        double v = json_object_get_double(zm);
        if (v >= 0.5 && v <= 200.0) {
            *mag_out = v;
            ok = true;
        }
    }
    json_object_put(obj);
    if (ok) {
        pthread_mutex_lock(&g_mu);
        g_cached_mag = *mag_out;
        g_cached_mag_valid = true;
        pthread_mutex_unlock(&g_mu);
    }
    return ok;
}

static double current_mag_or_cached(void) {
    double m = 1.0;
    if (query_zoom_mag(&m)) return m;
    pthread_mutex_lock(&g_mu);
    if (g_cached_mag_valid) m = g_cached_mag;
    pthread_mutex_unlock(&g_mu);
    return m;
}

static void send_motors_stop_now(void) {
    char err[128] = "";
    struct motors_client *cl = NULL;
    if (motors_open(&cl, g_cfg.socket_path, err, sizeof(err)) != 0) return;
    (void)motors_stop_all(cl, err, sizeof(err));
    (void)motors_release(cl, err, sizeof(err));
    motors_close(cl);
}

static bool job_cancelled(unsigned gen) {
    pthread_mutex_lock(&g_mu);
    bool c = (g_job_gen != gen) || g_stop_daemon;
    pthread_mutex_unlock(&g_mu);
    return c;
}

/* Drive one axis (pan/tilt/focus) in chunks with live tracking & preemption support. */
static bool drive_axis_tracked(struct motors_client *cl, enum motors_client_axis axis,
                               enum motors_direction dir, unsigned total_ms,
                               unsigned speed, unsigned gen) {
    int dp = (axis == MOTORS_PAN) ? (dir == MOTORS_RIGHT ? 1 : -1) : 0;
    int dt = (axis == MOTORS_TILT) ? (dir == MOTORS_UP ? 1 : -1) : 0;
    int df = (axis == MOTORS_FOCUS) ? (dir == MOTORS_FAR ? 1 : -1) : 0;
    double speed_scale = (speed > 0 && speed < 63) ? ((double)speed / 63.0) : 1.0;

    unsigned rem = total_ms;
    while (rem >= 30U) {
        if (job_cancelled(gen)) return false;
        unsigned chunk = rem > 3500U ? 3500U : rem;

        long pan = 0, tilt = 0, focus = 0;
        ptz_pos_read(&pan, &tilt, &focus);
        touch_af_cancel();

        char err[128] = "";
        int rc = motors_acquire(cl, MOTORS_MANUAL, axis, chunk + 1000U, err, sizeof(err));
        if (rc == 0) {
            if (speed > 0 && speed < 63)
                rc = motors_move_speed(cl, axis, dir, speed, chunk, err, sizeof(err));
            else
                rc = motors_move(cl, axis, dir, chunk, err, sizeof(err));
        }
        if (rc != 0) {
            ptz_live_clear();
            return false;
        }
        uint64_t t0 = mono_now_ms();
        if (dp || dt || df) {
            ptz_live_write(pan, tilt, focus, dp, dt, df, t0, t0 + chunk);
        }

        /* Wait in small 15ms slices so preemption/Stop is immediate and accurate. */
        while (true) {
            uint64_t now = mono_now_ms();
            if (now >= t0 + chunk) break;
            if (job_cancelled(gen)) {
                (void)motors_stop_all(cl, err, sizeof(err));
                uint64_t t1 = mono_now_ms();
                pthread_mutex_lock(&g_mu);
                uint64_t c_ms = g_cancel_mono_ms;
                pthread_mutex_unlock(&g_mu);
                uint64_t t_end = (c_ms >= t0 && c_ms <= t1) ? c_ms : t1;
                long el = (t_end > t0) ? (long)(t_end - t0) : 0;
                if (el > (long)chunk) el = (long)chunk;
                long scaled = (long)llround((double)el * speed_scale);
                ptz_live_clear();
                if (scaled >= 20) {
                    ptz_pos_write(pan + (long)dp * scaled,
                                  tilt + (long)dt * scaled,
                                  focus + (long)df * scaled, true);
                }
                sleep_ms(100);
                return false;
            }
            unsigned wait_ms = (unsigned)((t0 + chunk) - now);
            if (wait_ms > 15U) wait_ms = 15U;
            sleep_ms(wait_ms);
        }

        ptz_live_clear();
        long scaled = (long)llround((double)chunk * speed_scale);
        ptz_pos_write(pan + (long)dp * scaled,
                      tilt + (long)dt * scaled,
                      focus + (long)df * scaled, false);
        rem -= chunk;
        if (rem >= 30U) sleep_ms(80);
    }
    return true;
}

static bool drive_zoom_to_mag(struct motors_client *cl, double target_mag, unsigned gen) {
    target_mag = clamp_d(target_mag, 1.0, g_cfg.max_zoom);
    double prev_mag = -1.0;
    int stall = 0;

    for (int step = 0; step < 25; step++) {
        if (job_cancelled(gen)) return false;
        double cur = 1.0;
        if (!query_zoom_mag(&cur)) break;

        if (fabs(cur - prev_mag) < 0.05) {
            stall++;
            if (stall >= 5) break;
        } else {
            stall = 0;
        }
        prev_mag = cur;

        double diff = target_mag - cur;
        double adiff = fabs(diff);
        if (target_mag <= 1.15) {
            if (cur <= 1.15) break;
        } else {
            double tol = (target_mag > 15.0) ? 0.8 : 0.4;
            if (adiff < tol) break;
        }

        enum motors_direction dir = (diff > 0.0) ? MOTORS_TELE : MOTORS_WIDE;
        double rate = (cur < 6.0 || target_mag < 6.0) ? 250.0
                    : ((cur < 15.0 || target_mag < 15.0) ? 140.0 : 60.0);
        int ms = (int)(adiff * rate * (1.0 + stall * 0.4));
        int min_ms = (target_mag <= 1.15 && cur > 1.5) ? 450 : 180;
        if (ms < min_ms) ms = min_ms;
        if (ms > 2200) ms = 2200;

        touch_af_cancel();
        char err[128] = "";
        int rc = motors_acquire(cl, MOTORS_MANUAL, MOTORS_ZOOM, (unsigned)ms + 1000U, err, sizeof(err));
        if (rc == 0)
            rc = motors_move(cl, MOTORS_ZOOM, dir, (unsigned)ms, err, sizeof(err));
        if (rc != 0) return false;

        uint64_t t0 = mono_now_ms();
        while (mono_now_ms() < t0 + (uint64_t)ms) {
            if (job_cancelled(gen)) {
                (void)motors_stop_all(cl, err, sizeof(err));
                return false;
            }
            sleep_ms(40);
        }
        sleep_ms(80);
    }
    return !job_cancelled(gen);
}

/* Drive Pan and Tilt together using Pelco-D diagonal bitmask when both axes
 * are non-zero; falls back cleanly to sequential moves if raw is unsupported. */
static bool drive_pan_tilt_tracked(struct motors_client *cl,
                                   int dp, unsigned pan_ms,
                                   int dt, unsigned tilt_ms,
                                   unsigned gen) {
    unsigned rem_p = dp ? pan_ms : 0U;
    unsigned rem_t = dt ? tilt_ms : 0U;
    bool diag_ok = true;

    while (rem_p >= 30U && rem_t >= 30U && diag_ok) {
        if (job_cancelled(gen)) return false;
        unsigned chunk = rem_p < rem_t ? rem_p : rem_t;
        if (chunk > 3500U) chunk = 3500U;

        long pan = 0, tilt = 0, focus = 0;
        ptz_pos_read(&pan, &tilt, &focus);
        touch_af_cancel();

        uint8_t c2 = (uint8_t)((dp > 0 ? 0x02 : 0x04) | (dt > 0 ? 0x08 : 0x10));
        uint8_t d1 = 0x14, d2 = 0x14; /* matches anjoy.conf pan_speed=20, tilt_speed=20 */
        uint8_t csum = (uint8_t)((0x01 + c2 + d1 + d2) & 0xff);
        char req[160], resp[256] = "", err[128] = "";
        snprintf(req, sizeof(req),
                 "{\"version\":1,\"id\":\"diag\",\"op\":\"raw\",\"payload\":{\"bytes\":\"ff0100%02x%02x%02x%02x\"}}",
                 c2, d1, d2, csum);

        int rc = motors_acquire(cl, MOTORS_MANUAL, MOTORS_PAN, chunk + 1000U, err, sizeof(err));
        if (rc == 0) rc = motors_request(cl, req, resp, sizeof(resp), err, sizeof(err));
        if (rc != 0) {
            diag_ok = false;
            break;
        }

        uint64_t t0 = mono_now_ms();
        ptz_live_write(pan, tilt, focus, dp, dt, 0, t0, t0 + chunk);

        while (true) {
            uint64_t now = mono_now_ms();
            if (now >= t0 + chunk) break;
            if (job_cancelled(gen)) {
                (void)motors_stop_all(cl, err, sizeof(err));
                uint64_t t1 = mono_now_ms();
                pthread_mutex_lock(&g_mu);
                uint64_t c_ms = g_cancel_mono_ms;
                pthread_mutex_unlock(&g_mu);
                uint64_t t_end = (c_ms >= t0 && c_ms <= t1) ? c_ms : t1;
                long el = (t_end > t0) ? (long)(t_end - t0) : 0;
                if (el > (long)chunk) el = (long)chunk;
                ptz_live_clear();
                if (el >= 20) {
                    ptz_pos_write(pan + (long)dp * el, tilt + (long)dt * el, focus, true);
                }
                sleep_ms(100);
                return false;
            }
            unsigned wait_ms = (unsigned)((t0 + chunk) - now);
            if (wait_ms > 15U) wait_ms = 15U;
            sleep_ms(wait_ms);
        }

        (void)motors_stop_all(cl, err, sizeof(err));
        ptz_live_clear();
        ptz_pos_write(pan + (long)dp * (long)chunk,
                      tilt + (long)dt * (long)chunk, focus, false);
        rem_p -= chunk;
        rem_t -= chunk;
        if (rem_p >= 30U || rem_t >= 30U) sleep_ms(40);
    }

    bool ok = true;
    if (rem_p >= 30U) {
        enum motors_direction pdir = (dp > 0) ? MOTORS_RIGHT : MOTORS_LEFT;
        ok = drive_axis_tracked(cl, MOTORS_PAN, pdir, rem_p, 0, gen);
    }
    if (ok && rem_t >= 30U) {
        enum motors_direction tdir = (dt > 0) ? MOTORS_UP : MOTORS_DOWN;
        ok = drive_axis_tracked(cl, MOTORS_TILT, tdir, rem_t, 0, gen);
    }
    return ok;
}

static void execute_job(const struct ptz_job *job) {
    char err[128] = "";
    struct motors_client *cl = NULL;
    if (motors_open(&cl, g_cfg.socket_path, err, sizeof(err)) != 0) {
        fprintf(stderr, "motors-onvif: motors_open failed: %s\n", err);
        return;
    }

    if (job->kind == JOB_CONTINUOUS) {
        unsigned dur = job->timeout_ms > 0 ? job->timeout_ms : 5000U;
        if (dur > 15000U) dur = 15000U;

        if (job->has_zoom && fabs(job->vz) > 0.01) {
            pthread_mutex_lock(&g_mu);
            g_zoom_moving = true;
            pthread_mutex_unlock(&g_mu);
            enum motors_direction zdir = (job->vz > 0.0) ? MOTORS_TELE : MOTORS_WIDE;
            (void)drive_axis_tracked(cl, MOTORS_ZOOM, zdir, dur, 0, job->gen);
            (void)query_zoom_mag(&g_cached_mag);
        } else if (job->has_pt) {
            double vx = g_cfg.invert_pan ? -job->vx : job->vx;
            double vy = g_cfg.invert_tilt ? -job->vy : job->vy;
            double ax = fabs(vx), ay = fabs(vy);
            int dp = (ax > 0.01) ? (vx > 0.0 ? 1 : -1) : 0;
            int dt = (ay > 0.01) ? (vy > 0.0 ? 1 : -1) : 0;
            if (dp || dt) {
                pthread_mutex_lock(&g_mu);
                g_pt_moving = true;
                pthread_mutex_unlock(&g_mu);
                /* Always run at the driver's calibrated default speed (speed=0) so
                 * dead-reckoned PAN_MS/TILT_MS stays 1:1 with presets and WebUI controls. */
                (void)drive_pan_tilt_tracked(cl, dp, dp ? dur : 0U, dt, dt ? dur : 0U, job->gen);
            }
        }
    } else if (job->kind == JOB_RELATIVE) {
        double mag = current_mag_or_cached();
        if (mag < 1.0) mag = 1.0;
        double fov_scale = 0.05 + (0.95 / (1.0 + 0.35 * (mag - 1.0)));

        if (job->has_pt) {
            double dx = g_cfg.invert_pan ? -job->vx : job->vx;
            double dy = g_cfg.invert_tilt ? -job->vy : job->vy;
            double adx = fabs(dx), ady = fabs(dy);

            unsigned pms = 0, tms = 0;
            if (adx >= 0.015) {
                pms = (unsigned)clamp_d(round(60.0 + adx * g_cfg.pan_scale_ms * fov_scale), 80.0, 10000.0);
            }
            if (ady >= 0.015) {
                tms = (unsigned)clamp_d(round(60.0 + ady * g_cfg.tilt_scale_ms * fov_scale), 80.0, 8000.0);
            }
            if (g_cfg.debug) {
                fprintf(stderr, "motors-onvif: RelativeMove dx=%.3f dy=%.3f mag=%.1f fov=%.3f -> pms=%u tms=%u\n",
                        dx, dy, mag, fov_scale, pms, tms);
            }
            if (pms > 0 || tms > 0) {
                pthread_mutex_lock(&g_mu);
                g_pt_moving = true;
                pthread_mutex_unlock(&g_mu);
                int dp = pms > 0 ? (dx > 0.0 ? 1 : -1) : 0;
                int dt = tms > 0 ? (dy > 0.0 ? 1 : -1) : 0;
                (void)drive_pan_tilt_tracked(cl, dp, pms, dt, tms, job->gen);
            }
        }
        if (job->has_zoom && fabs(job->vz) >= 0.005 && !job_cancelled(job->gen)) {
            pthread_mutex_lock(&g_mu);
            g_zoom_moving = true;
            pthread_mutex_unlock(&g_mu);
            double target_mag = clamp_d(mag + job->vz * (g_cfg.max_zoom - 1.0), 1.0, g_cfg.max_zoom);
            if (g_cfg.debug) {
                fprintf(stderr, "motors-onvif: RelativeMove zoom dz=%.3f cur=%.1f -> target=%.1f\n",
                        job->vz, mag, target_mag);
            }
            (void)drive_zoom_to_mag(cl, target_mag, job->gen);
        }
    } else if (job->kind == JOB_ABSOLUTE) {
        if (job->has_pt) {
            long cur_p = 0, cur_t = 0, cur_f = 0;
            ptz_pos_read(&cur_p, &cur_t, &cur_f);
            long tgt_p = (long)llround(clamp_d(job->vx, -1.0, 1.0) * g_cfg.pan_range_ms);
            long tgt_t = (long)llround(clamp_d(job->vy, -1.0, 1.0) * g_cfg.tilt_range_ms);
            long dp = tgt_p - cur_p;
            long dt = tgt_t - cur_t;
            if (labs(dp) >= 40 || labs(dt) >= 40) {
                pthread_mutex_lock(&g_mu);
                g_pt_moving = true;
                pthread_mutex_unlock(&g_mu);
                int pdir = labs(dp) >= 40 ? (dp > 0 ? 1 : -1) : 0;
                int tdir = labs(dt) >= 40 ? (dt > 0 ? 1 : -1) : 0;
                (void)drive_pan_tilt_tracked(cl, pdir, pdir ? (unsigned)labs(dp) : 0U,
                                             tdir, tdir ? (unsigned)labs(dt) : 0U, job->gen);
            }
        }
        if (job->has_zoom && !job_cancelled(job->gen)) {
            pthread_mutex_lock(&g_mu);
            g_zoom_moving = true;
            pthread_mutex_unlock(&g_mu);
            double target_mag = 1.0 + clamp_d(job->vz, 0.0, 1.0) * (g_cfg.max_zoom - 1.0);
            if (g_cfg.debug) {
                fprintf(stderr, "motors-onvif: AbsoluteMove zoom z=%.3f -> target=%.1f\n",
                        job->vz, target_mag);
            }
            (void)drive_zoom_to_mag(cl, target_mag, job->gen);
        }
    } else if (job->kind == JOB_PRESET) {
        const struct preset_entry *pr = &job->preset;
        long cur_p = 0, cur_t = 0, cur_f = 0;
        ptz_pos_read(&cur_p, &cur_t, &cur_f);
        long dp = pr->pan_ms - cur_p;
        long dt = pr->tilt_ms - cur_t;
        long df = pr->has_focus ? (pr->focus_ms - cur_f) : 0;

        if (g_cfg.debug) {
            fprintf(stderr, "motors-onvif: GotoPreset #%d '%s' dp=%ld dt=%ld mag=%.1f\n",
                    pr->id, pr->name, dp, dt, pr->mag);
        }

        bool ok = true;
        if (labs(dp) >= 30 || labs(dt) >= 30) {
            pthread_mutex_lock(&g_mu);
            g_pt_moving = true;
            pthread_mutex_unlock(&g_mu);
            if (labs(dp) >= 30) {
                ok = drive_axis_tracked(cl, MOTORS_PAN, dp > 0 ? MOTORS_RIGHT : MOTORS_LEFT,
                                        (unsigned)labs(dp), 0, job->gen);
            }
            if (ok && labs(dt) >= 30) {
                ok = drive_axis_tracked(cl, MOTORS_TILT, dt > 0 ? MOTORS_UP : MOTORS_DOWN,
                                        (unsigned)labs(dt), 0, job->gen);
            }
        }
        if (ok && pr->zoom_restore && pr->has_mag) {
            pthread_mutex_lock(&g_mu);
            g_pt_moving = false;
            g_zoom_moving = true;
            pthread_mutex_unlock(&g_mu);
            ok = drive_zoom_to_mag(cl, pr->mag, job->gen);
        }
        if (ok && pr->has_focus && labs(df) >= 30) {
            ok = drive_axis_tracked(cl, MOTORS_FOCUS, df > 0 ? MOTORS_FAR : MOTORS_NEAR,
                                    (unsigned)labs(df), 0, job->gen);
        }
        if (ok) {
            ptz_pos_write(pr->pan_ms, pr->tilt_ms, pr->has_focus ? pr->focus_ms : cur_f, true);
        }
    } else if (job->kind == JOB_FOCUS) {
        unsigned dur = job->timeout_ms > 0 ? job->timeout_ms : 400U;
        if (dur > 5000U) dur = 5000U;
        if (fabs(job->vz) > 0.01) {
            enum motors_direction fdir = (job->vz > 0.0) ? MOTORS_FAR : MOTORS_NEAR;
            (void)drive_axis_tracked(cl, MOTORS_FOCUS, fdir, dur, 0, job->gen);
        }
    }

    /* Persist final position and release motor lease cleanly. */
    bool should_af = false;
    if (!job_cancelled(job->gen)) {
        long p = 0, t = 0, f = 0;
        ptz_pos_read(&p, &t, &f);
        ptz_pos_write(p, t, f, true);
        if (g_cfg.post_zoom_af) {
            if (job->kind == JOB_PRESET && (job->preset.run_af || job->has_zoom)) {
                should_af = true;
            } else if ((job->kind == JOB_RELATIVE || job->kind == JOB_ABSOLUTE ||
                        job->kind == JOB_CONTINUOUS) && job->has_zoom) {
                should_af = true;
            }
        }
    }
    (void)motors_release(cl, err, sizeof(err));
    motors_close(cl);

    /* Smart post-zoom autofocus: mark IDLE first, wait af_settle_ms for any follow-up
     * tracking commands, and only trigger /autofocus if nothing preempted us. */
    if (should_af && !job_cancelled(job->gen)) {
        pthread_mutex_lock(&g_mu);
        if (g_job_gen == job->gen) {
            g_pt_moving = false;
            g_zoom_moving = false;
        }
        pthread_mutex_unlock(&g_mu);

        uint64_t t0 = mono_now_ms();
        while (mono_now_ms() < t0 + g_cfg.af_settle_ms) {
            if (job_cancelled(job->gen)) return;
            sleep_ms(40);
        }
        if (!job_cancelled(job->gen)) {
            if (g_cfg.debug) fprintf(stderr, "motors-onvif: triggering post-zoom AF settle\n");
            trigger_local_af();
        }
    }
}

static void *worker_main(void *arg) {
    (void)arg;
    while (!g_stop_daemon) {
        pthread_mutex_lock(&g_mu);
        while (!g_has_pending_job && !g_stop_daemon) {
            pthread_cond_wait(&g_cv, &g_mu);
        }
        if (g_stop_daemon) {
            pthread_mutex_unlock(&g_mu);
            break;
        }
        struct ptz_job job = g_pending_job;
        g_has_pending_job = false;
        pthread_mutex_unlock(&g_mu);

        execute_job(&job);

        pthread_mutex_lock(&g_mu);
        if (g_job_gen == job.gen) {
            g_pt_moving = false;
            g_zoom_moving = false;
        }
        pthread_mutex_unlock(&g_mu);
    }
    return NULL;
}

static void submit_job(struct ptz_job job) {
    pthread_mutex_lock(&g_mu);
    g_job_gen++;
    g_cancel_mono_ms = mono_now_ms();
    job.gen = g_job_gen;
    g_pending_job = job;
    g_has_pending_job = true;
    g_pt_moving = job.has_pt || (job.kind == JOB_PRESET);
    g_zoom_moving = job.has_zoom;
    pthread_cond_signal(&g_cv);
    pthread_mutex_unlock(&g_mu);
}

static void cancel_all_jobs(void) {
    pthread_mutex_lock(&g_mu);
    bool was_moving = g_pt_moving || g_zoom_moving;
    g_job_gen++;
    g_cancel_mono_ms = mono_now_ms();
    g_has_pending_job = false;
    g_pt_moving = false;
    g_zoom_moving = false;
    pthread_cond_signal(&g_cv);
    pthread_mutex_unlock(&g_mu);
    if (!was_moving) {
        send_motors_stop_now();
        ptz_live_clear();
    }
}

/* ---------------- Lightweight XML Extraction Helpers ---------------- */

/* Find a tag ignoring namespace prefix, e.g. <tptz:ContinuousMove> or <ContinuousMove>. */
static const char *xml_find_tag_open(const char *xml, const char *local_name) {
    size_t nlen = strlen(local_name);
    const char *p = xml;
    while ((p = strchr(p, '<')) != NULL) {
        p++;
        if (*p == '/' || *p == '?' || *p == '!') continue;
        const char *start = p;
        while (*p && *p != '>' && *p != '/' && !isspace((unsigned char)*p)) p++;
        const char *colon = NULL;
        for (const char *q = start; q < p; q++) {
            if (*q == ':') colon = q;
        }
        const char *lname = colon ? colon + 1 : start;
        size_t llen = (size_t)(p - lname);
        if (llen == nlen && strncasecmp(lname, local_name, nlen) == 0) {
            return start - 1;
        }
    }
    return NULL;
}

static bool xml_get_text(const char *xml, const char *local_name, char *out, size_t out_size) {
    const char *open = xml_find_tag_open(xml, local_name);
    if (!open) return false;
    const char *gt = strchr(open, '>');
    if (!gt || gt[-1] == '/') return false;
    const char *val = gt + 1;
    const char *lt = strchr(val, '<');
    if (!lt) return false;
    while (val < lt && isspace((unsigned char)*val)) val++;
    while (lt > val && isspace((unsigned char)lt[-1])) lt--;
    size_t len = (size_t)(lt - val);
    if (len >= out_size) len = out_size - 1;
    memcpy(out, val, len);
    out[len] = '\0';
    return true;
}

/* Extract the inner slice of <...:local_name>...</...:local_name> into out. */
static bool xml_get_element(const char *xml, const char *local_name, char *out, size_t out_size) {
    const char *open = xml_find_tag_open(xml, local_name);
    if (!open) return false;
    const char *gt = strchr(open, '>');
    if (!gt) return false;
    if (gt[-1] == '/') {
        size_t len = (size_t)(gt - open + 1);
        if (len >= out_size) len = out_size - 1;
        memcpy(out, open, len);
        out[len] = '\0';
        return true;
    }
    size_t nlen = strlen(local_name);
    const char *p = gt + 1;
    while ((p = strchr(p, '<')) != NULL) {
        if (p[1] == '/') {
            const char *s = p + 2;
            const char *e = s;
            while (*e && *e != '>' && !isspace((unsigned char)*e)) e++;
            const char *colon = NULL;
            for (const char *q = s; q < e; q++) if (*q == ':') colon = q;
            const char *lname = colon ? colon + 1 : s;
            if ((size_t)(e - lname) == nlen && strncasecmp(lname, local_name, nlen) == 0) {
                const char *end_gt = strchr(e, '>');
                const char *end = end_gt ? end_gt + 1 : e;
                size_t len = (size_t)(end - open);
                if (len >= out_size) len = out_size - 1;
                memcpy(out, open, len);
                out[len] = '\0';
                return true;
            }
        }
        p++;
    }
    return false;
}

static bool xml_get_attr(const char *tag_start, const char *attr_name, char *out, size_t out_size) {
    const char *gt = strchr(tag_start, '>');
    if (!gt) return false;
    size_t alen = strlen(attr_name);
    const char *p = tag_start;
    while (p < gt) {
        while (p < gt && !isspace((unsigned char)*p)) p++;
        while (p < gt && isspace((unsigned char)*p)) p++;
        if (p >= gt) break;
        const char *k = p;
        while (p < gt && *p != '=' && !isspace((unsigned char)*p) && *p != '>' && *p != '/') p++;
        size_t klen = (size_t)(p - k);
        while (p < gt && isspace((unsigned char)*p)) p++;
        if (p < gt && *p == '=') {
            p++;
            while (p < gt && isspace((unsigned char)*p)) p++;
            if (p < gt && (*p == '"' || *p == '\'')) {
                char q = *p++;
                const char *v = p;
                while (p < gt && *p != q) p++;
                if (klen == alen && strncasecmp(k, attr_name, alen) == 0) {
                    size_t vlen = (size_t)(p - v);
                    if (vlen >= out_size) vlen = out_size - 1;
                    memcpy(out, v, vlen);
                    out[vlen] = '\0';
                    return true;
                }
                if (p < gt && *p == q) p++;
            }
        }
    }
    return false;
}

static void xml_escape(const char *in, char *out, size_t out_size) {
    size_t j = 0;
    for (const char *p = in; *p && j + 6 < out_size; p++) {
        switch (*p) {
            case '&': memcpy(out + j, "&amp;", 5); j += 5; break;
            case '<': memcpy(out + j, "&lt;", 4); j += 4; break;
            case '>': memcpy(out + j, "&gt;", 4); j += 4; break;
            case '"': memcpy(out + j, "&quot;", 6); j += 6; break;
            case '\'': memcpy(out + j, "&apos;", 6); j += 6; break;
            default: out[j++] = *p; break;
        }
    }
    out[j] = '\0';
}

static unsigned parse_iso_duration_ms(const char *iso) {
    /* Parse PT5S / PT1.5S / PT1M */
    if (!iso || !*iso) return 0;
    const char *p = strchr(iso, 'T');
    if (!p) return 0;
    p++;
    double total_sec = 0.0;
    while (*p) {
        char *end = NULL;
        double v = strtod(p, &end);
        if (end == p) break;
        if (*end == 'H' || *end == 'h') total_sec += v * 3600.0;
        else if (*end == 'M' || *end == 'm') total_sec += v * 60.0;
        else if (*end == 'S' || *end == 's') total_sec += v;
        p = (*end) ? end + 1 : end;
    }
    if (total_sec <= 0.0) return 0;
    if (total_sec > 30.0) total_sec = 30.0;
    return (unsigned)(total_sec * 1000.0);
}

/* ---------------- Auth Verification (Basic + WSSE PasswordText / PasswordDigest) ---------------- */

static bool check_auth(const char *headers, const char *body) {
    if (!g_cfg.user[0] || !g_cfg.pass[0]) return true;

    /* 1. Check HTTP Basic Auth header */
    const char *h = headers;
    while (h && *h) {
        const char *eol = strstr(h, "\r\n");
        if (strncasecmp(h, "Authorization:", 14) == 0) {
            const char *v = h + 14;
            while (*v == ' ' || *v == '\t') v++;
            if (strncasecmp(v, "Basic ", 6) == 0) {
                v += 6;
                char b64[256];
                size_t i = 0;
                while (*v && *v != '\r' && *v != '\n' && i + 1 < sizeof(b64))
                    b64[i++] = *v++;
                b64[i] = '\0';
                uint8_t dec[256];
                size_t dlen = b64_decode(b64, dec, sizeof(dec) - 1);
                dec[dlen] = '\0';
                char expected[256];
                snprintf(expected, sizeof(expected), "%s:%s", g_cfg.user, g_cfg.pass);
                if (strcmp((char *)dec, expected) == 0) return true;
            }
        }
        if (!eol) break;
        h = eol + 2;
    }

    /* 2. Check WS-Security UsernameToken in SOAP Header */
    char ws_user[128] = "", ws_pass[256] = "", ws_nonce[256] = "", ws_created[128] = "";
    if (!xml_get_text(body, "Username", ws_user, sizeof(ws_user))) return false;
    if (!xml_get_text(body, "Password", ws_pass, sizeof(ws_pass))) return false;
    if (strcmp(ws_user, g_cfg.user) != 0) return false;

    const char *pass_tag = xml_find_tag_open(body, "Password");
    char pass_type[128] = "";
    if (pass_tag) xml_get_attr(pass_tag, "Type", pass_type, sizeof(pass_type));

    bool is_digest = (strstr(pass_type, "PasswordDigest") != NULL) ||
                     xml_get_text(body, "Nonce", ws_nonce, sizeof(ws_nonce));
    if (!is_digest) {
        return strcmp(ws_pass, g_cfg.pass) == 0;
    }

    if (!ws_nonce[0] && !xml_get_text(body, "Nonce", ws_nonce, sizeof(ws_nonce)))
        return false;
    if (!xml_get_text(body, "Created", ws_created, sizeof(ws_created)))
        return false;

    uint8_t nonce_raw[128];
    size_t nonce_len = b64_decode(ws_nonce, nonce_raw, sizeof(nonce_raw));
    if (nonce_len == 0) return false;

    struct sha1_ctx ctx;
    uint8_t digest[20];
    sha1_init(&ctx);
    sha1_update(&ctx, nonce_raw, nonce_len);
    sha1_update(&ctx, (const uint8_t *)ws_created, strlen(ws_created));
    sha1_update(&ctx, (const uint8_t *)g_cfg.pass, strlen(g_cfg.pass));
    sha1_final(&ctx, digest);

    char expected_b64[64];
    b64_encode(digest, sizeof(digest), expected_b64, sizeof(expected_b64));
    return strcmp(ws_pass, expected_b64) == 0;
}

/* ---------------- SOAP Response Builders ---------------- */

static int soap_wrap(char *out, size_t out_size, bool soap11, const char *body_xml) {
    const char *env_ns = soap11
        ? "http://schemas.xmlsoap.org/soap/envelope/"
        : "http://www.w3.org/2003/05/soap-envelope";
    return snprintf(out, out_size,
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>"
        "<s:Envelope xmlns:s=\"%s\" "
        "xmlns:tt=\"http://www.onvif.org/ver10/schema\" "
        "xmlns:tds=\"http://www.onvif.org/ver10/device/wsdl\" "
        "xmlns:trt=\"http://www.onvif.org/ver10/media/wsdl\" "
        "xmlns:tptz=\"http://www.onvif.org/ver20/ptz/wsdl\" "
        "xmlns:timg=\"http://www.onvif.org/ver20/imaging/wsdl\">"
        "<s:Body>%s</s:Body></s:Envelope>",
        env_ns, body_xml);
}

static void build_ptz_config_xml(char *buf, size_t sz) {
    snprintf(buf, sz,
        "<tt:PTZConfiguration token=\"PTZ_CFG_000\">"
        "<tt:Name>PTZ_CFG_000</tt:Name>"
        "<tt:UseCount>1</tt:UseCount>"
        "<tt:NodeToken>PTZ_NODE_000</tt:NodeToken>"
        "<tt:DefaultAbsolutePantTiltPositionSpace>http://www.onvif.org/ver10/tptz/PanTiltSpaces/PositionGenericSpace</tt:DefaultAbsolutePantTiltPositionSpace>"
        "<tt:DefaultAbsoluteZoomPositionSpace>http://www.onvif.org/ver10/tptz/ZoomSpaces/PositionGenericSpace</tt:DefaultAbsoluteZoomPositionSpace>"
        "<tt:DefaultRelativePanTiltTranslationSpace>http://www.onvif.org/ver10/tptz/PanTiltSpaces/TranslationSpaceFov</tt:DefaultRelativePanTiltTranslationSpace>"
        "<tt:DefaultRelativeZoomTranslationSpace>http://www.onvif.org/ver10/tptz/ZoomSpaces/TranslationGenericSpace</tt:DefaultRelativeZoomTranslationSpace>"
        "<tt:DefaultContinuousPanTiltVelocitySpace>http://www.onvif.org/ver10/tptz/PanTiltSpaces/VelocityGenericSpace</tt:DefaultContinuousPanTiltVelocitySpace>"
        "<tt:DefaultContinuousZoomVelocitySpace>http://www.onvif.org/ver10/tptz/ZoomSpaces/VelocityGenericSpace</tt:DefaultContinuousZoomVelocitySpace>"
        "<tt:DefaultPTZSpeed>"
        "<tt:PanTilt x=\"1\" y=\"1\" space=\"http://www.onvif.org/ver10/tptz/PanTiltSpaces/GenericSpeedSpace\"/>"
        "<tt:Zoom x=\"1\" space=\"http://www.onvif.org/ver10/tptz/ZoomSpaces/ZoomGenericSpeedSpace\"/>"
        "</tt:DefaultPTZSpeed>"
        "<tt:DefaultPTZTimeout>PT10S</tt:DefaultPTZTimeout>"
        "<tt:PanTiltLimits><tt:Range>"
        "<tt:URI>http://www.onvif.org/ver10/tptz/PanTiltSpaces/PositionGenericSpace</tt:URI>"
        "<tt:XRange><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:XRange>"
        "<tt:YRange><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:YRange>"
        "</tt:Range></tt:PanTiltLimits>"
        "<tt:ZoomLimits><tt:Range>"
        "<tt:URI>http://www.onvif.org/ver10/tptz/ZoomSpaces/PositionGenericSpace</tt:URI>"
        "<tt:XRange><tt:Min>0</tt:Min><tt:Max>1</tt:Max></tt:XRange>"
        "</tt:Range></tt:ZoomLimits>"
        "</tt:PTZConfiguration>");
}

static void build_ptz_spaces_xml(char *buf, size_t sz) {
    snprintf(buf, sz,
        "<tt:AbsolutePanTiltPositionSpace>"
        "<tt:URI>http://www.onvif.org/ver10/tptz/PanTiltSpaces/PositionGenericSpace</tt:URI>"
        "<tt:XRange><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:XRange>"
        "<tt:YRange><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:YRange>"
        "</tt:AbsolutePanTiltPositionSpace>"
        "<tt:AbsoluteZoomPositionSpace>"
        "<tt:URI>http://www.onvif.org/ver10/tptz/ZoomSpaces/PositionGenericSpace</tt:URI>"
        "<tt:XRange><tt:Min>0</tt:Min><tt:Max>1</tt:Max></tt:XRange>"
        "</tt:AbsoluteZoomPositionSpace>"
        "<tt:RelativePanTiltTranslationSpace>"
        "<tt:URI>http://www.onvif.org/ver10/tptz/PanTiltSpaces/TranslationSpaceFov</tt:URI>"
        "<tt:XRange><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:XRange>"
        "<tt:YRange><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:YRange>"
        "</tt:RelativePanTiltTranslationSpace>"
        "<tt:RelativePanTiltTranslationSpace>"
        "<tt:URI>http://www.onvif.org/ver10/tptz/PanTiltSpaces/TranslationGenericSpace</tt:URI>"
        "<tt:XRange><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:XRange>"
        "<tt:YRange><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:YRange>"
        "</tt:RelativePanTiltTranslationSpace>"
        "<tt:RelativeZoomTranslationSpace>"
        "<tt:URI>http://www.onvif.org/ver10/tptz/ZoomSpaces/TranslationGenericSpace</tt:URI>"
        "<tt:XRange><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:XRange>"
        "</tt:RelativeZoomTranslationSpace>"
        "<tt:ContinuousPanTiltVelocitySpace>"
        "<tt:URI>http://www.onvif.org/ver10/tptz/PanTiltSpaces/VelocityGenericSpace</tt:URI>"
        "<tt:XRange><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:XRange>"
        "<tt:YRange><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:YRange>"
        "</tt:ContinuousPanTiltVelocitySpace>"
        "<tt:ContinuousZoomVelocitySpace>"
        "<tt:URI>http://www.onvif.org/ver10/tptz/ZoomSpaces/VelocityGenericSpace</tt:URI>"
        "<tt:XRange><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:XRange>"
        "</tt:ContinuousZoomVelocitySpace>"
        "<tt:PanTiltSpeedSpace>"
        "<tt:URI>http://www.onvif.org/ver10/tptz/PanTiltSpaces/GenericSpeedSpace</tt:URI>"
        "<tt:XRange><tt:Min>0</tt:Min><tt:Max>1</tt:Max></tt:XRange>"
        "</tt:PanTiltSpeedSpace>"
        "<tt:ZoomSpeedSpace>"
        "<tt:URI>http://www.onvif.org/ver10/tptz/ZoomSpaces/ZoomGenericSpeedSpace</tt:URI>"
        "<tt:XRange><tt:Min>0</tt:Min><tt:Max>1</tt:Max></tt:XRange>"
        "</tt:ZoomSpeedSpace>");
}

static bool parse_ptz_vector(const char *section_xml, double *x, double *y,
                             bool *has_pt, double *z, bool *has_z) {
    *x = 0.0;
    *y = 0.0;
    *z = 0.0;
    *has_pt = false;
    *has_z = false;
    const char *pt = xml_find_tag_open(section_xml, "PanTilt");
    if (pt) {
        char xs[32] = "", ys[32] = "";
        if (xml_get_attr(pt, "x", xs, sizeof(xs))) { *x = atof(xs); *has_pt = true; }
        if (xml_get_attr(pt, "y", ys, sizeof(ys))) { *y = atof(ys); *has_pt = true; }
    }
    const char *zm = xml_find_tag_open(section_xml, "Zoom");
    if (zm) {
        char zs[32] = "";
        if (xml_get_attr(zm, "x", zs, sizeof(zs))) { *z = atof(zs); *has_z = true; }
    }
    return *has_pt || *has_z;
}

static int handle_soap_request(const char *headers, const char *body,
                               const char *host_port, const char *host_only,
                               char *resp_xml, size_t resp_size) {
    bool soap11 = (strstr(body, "schemas.xmlsoap.org/soap/envelope") != NULL);

    /* Unauthenticated bootstrap actions per ONVIF spec */
    if (xml_find_tag_open(body, "GetSystemDateAndTime")) {
        time_t now = time(NULL);
        struct tm utc;
        gmtime_r(&now, &utc);
        char b[1024];
        snprintf(b, sizeof(b),
            "<tds:GetSystemDateAndTimeResponse><tds:SystemDateAndTime>"
            "<tt:DateTimeType>NTP</tt:DateTimeType>"
            "<tt:DaylightSavings>false</tt:DaylightSavings>"
            "<tt:TimeZone><tt:TZ>UTC0</tt:TZ></tt:TimeZone>"
            "<tt:UTCDateTime>"
            "<tt:Time><tt:Hour>%d</tt:Hour><tt:Minute>%d</tt:Minute><tt:Second>%d</tt:Second></tt:Time>"
            "<tt:Date><tt:Year>%d</tt:Year><tt:Month>%d</tt:Month><tt:Day>%d</tt:Day></tt:Date>"
            "</tt:UTCDateTime>"
            "</tds:SystemDateAndTime></tds:GetSystemDateAndTimeResponse>",
            utc.tm_hour, utc.tm_min, utc.tm_sec,
            utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday);
        soap_wrap(resp_xml, resp_size, soap11, b);
        return 200;
    }

    if (xml_find_tag_open(body, "GetCapabilities")) {
        char b[2048];
        snprintf(b, sizeof(b),
            "<tds:GetCapabilitiesResponse><tds:Capabilities>"
            "<tt:Device><tt:XAddr>http://%s/onvif/device_service</tt:XAddr></tt:Device>"
            "<tt:Imaging><tt:XAddr>http://%s/onvif/image_service</tt:XAddr></tt:Imaging>"
            "<tt:Media><tt:XAddr>http://%s/onvif/media_service</tt:XAddr>"
            "<tt:StreamingCapabilities><tt:RTPMulticast>false</tt:RTPMulticast>"
            "<tt:RTP_TCP>true</tt:RTP_TCP><tt:RTP_RTSP_TCP>true</tt:RTP_RTSP_TCP>"
            "</tt:StreamingCapabilities></tt:Media>"
            "<tt:PTZ><tt:XAddr>http://%s/onvif/ptz_service</tt:XAddr></tt:PTZ>"
            "</tds:Capabilities></tds:GetCapabilitiesResponse>",
            host_port, host_port, host_port, host_port);
        soap_wrap(resp_xml, resp_size, soap11, b);
        return 200;
    }

    if (xml_find_tag_open(body, "GetServices")) {
        char inc_cap[16] = "";
        bool with_caps = xml_get_text(body, "IncludeCapability", inc_cap, sizeof(inc_cap)) &&
                         (strcasecmp(inc_cap, "true") == 0 || strcmp(inc_cap, "1") == 0);
        char b[4096];
        snprintf(b, sizeof(b),
            "<tds:GetServicesResponse>"
            "<tds:Service>"
            "<tds:Namespace>http://www.onvif.org/ver10/device/wsdl</tds:Namespace>"
            "<tds:XAddr>http://%s/onvif/device_service</tds:XAddr>"
            "<tds:Version><tt:Major>2</tt:Major><tt:Minor>6</tt:Minor></tds:Version>"
            "</tds:Service>"
            "<tds:Service>"
            "<tds:Namespace>http://www.onvif.org/ver10/media/wsdl</tds:Namespace>"
            "<tds:XAddr>http://%s/onvif/media_service</tds:XAddr>"
            "<tds:Version><tt:Major>2</tt:Major><tt:Minor>6</tt:Minor></tds:Version>"
            "</tds:Service>"
            "<tds:Service>"
            "<tds:Namespace>http://www.onvif.org/ver20/imaging/wsdl</tds:Namespace>"
            "<tds:XAddr>http://%s/onvif/image_service</tds:XAddr>"
            "<tds:Version><tt:Major>2</tt:Major><tt:Minor>6</tt:Minor></tds:Version>"
            "</tds:Service>"
            "<tds:Service>"
            "<tds:Namespace>http://www.onvif.org/ver20/ptz/wsdl</tds:Namespace>"
            "<tds:XAddr>http://%s/onvif/ptz_service</tds:XAddr>"
            "%s"
            "<tds:Version><tt:Major>2</tt:Major><tt:Minor>6</tt:Minor></tds:Version>"
            "</tds:Service>"
            "</tds:GetServicesResponse>",
            host_port, host_port, host_port, host_port,
            with_caps ? "<tds:Capabilities><tptz:Capabilities EFlip=\"false\" Reverse=\"false\" GetCompatibleConfigurations=\"true\" MoveStatus=\"true\" StatusPosition=\"true\"/></tds:Capabilities>" : "");
        soap_wrap(resp_xml, resp_size, soap11, b);
        return 200;
    }

    /* Authenticated actions */
    if (!check_auth(headers, body)) {
        soap_wrap(resp_xml, resp_size, soap11,
            "<s:Fault><s:Code><s:Value>s:Sender</s:Value>"
            "<s:Subcode><s:Value xmlns:ter=\"http://www.onvif.org/ver10/error\">ter:NotAuthorized</s:Value></s:Subcode></s:Code>"
            "<s:Reason><s:Text xml:lang=\"en\">The security token could not be authenticated.</s:Text></s:Reason></s:Fault>");
        return 401;
    }

    if (xml_find_tag_open(body, "GetDeviceInformation")) {
        soap_wrap(resp_xml, resp_size, soap11,
            "<tds:GetDeviceInformationResponse>"
            "<tds:Manufacturer>OpenIPC</tds:Manufacturer>"
            "<tds:Model>motorsd-PTZ</tds:Model>"
            "<tds:FirmwareVersion>1.0</tds:FirmwareVersion>"
            "<tds:SerialNumber>motorsd-001</tds:SerialNumber>"
            "<tds:HardwareId>openipc-ptz</tds:HardwareId>"
            "</tds:GetDeviceInformationResponse>");
        return 200;
    }

    if (xml_find_tag_open(body, "GetScopes")) {
        soap_wrap(resp_xml, resp_size, soap11,
            "<tds:GetScopesResponse>"
            "<tds:Scopes><tt:ScopeDef>Fixed</tt:ScopeDef><tt:ScopeItem>onvif://www.onvif.org/Profile/Streaming</tt:ScopeItem></tds:Scopes>"
            "<tds:Scopes><tt:ScopeDef>Fixed</tt:ScopeDef><tt:ScopeItem>onvif://www.onvif.org/name/OpenIPC-PTZ</tt:ScopeItem></tds:Scopes>"
            "</tds:GetScopesResponse>");
        return 200;
    }

    if (xml_find_tag_open(body, "GetProfiles") || xml_find_tag_open(body, "GetProfile")) {
        bool single = (xml_find_tag_open(body, "GetProfiles") == NULL);
        char ptz_cfg[2048];
        build_ptz_config_xml(ptz_cfg, sizeof(ptz_cfg));
        char b[4096];
        snprintf(b, sizeof(b),
            "<%s><%s token=\"PROFILE_000\" fixed=\"true\">"
            "<tt:Name>MainStream</tt:Name>"
            "<tt:VideoSourceConfiguration token=\"VSC_000\">"
            "<tt:Name>VSC_000</tt:Name><tt:UseCount>1</tt:UseCount>"
            "<tt:SourceToken>VS_000</tt:SourceToken>"
            "<tt:Bounds x=\"0\" y=\"0\" width=\"1920\" height=\"1080\"/>"
            "</tt:VideoSourceConfiguration>"
            "<tt:VideoEncoderConfiguration token=\"VEC_000\">"
            "<tt:Name>VEC_000</tt:Name><tt:UseCount>1</tt:UseCount>"
            "<tt:Encoding>H264</tt:Encoding>"
            "<tt:Resolution><tt:Width>1920</tt:Width><tt:Height>1080</tt:Height></tt:Resolution>"
            "<tt:Quality>5</tt:Quality>"
            "<tt:RateControl><tt:FrameRateLimit>25</tt:FrameRateLimit><tt:EncodingInterval>1</tt:EncodingInterval><tt:BitrateLimit>4096</tt:BitrateLimit></tt:RateControl>"
            "<tt:H264><tt:GovLength>25</tt:GovLength><tt:H264Profile>Main</tt:H264Profile></tt:H264>"
            "<tt:SessionTimeout>PT60S</tt:SessionTimeout>"
            "</tt:VideoEncoderConfiguration>"
            "%s"
            "</%s></%s>",
            single ? "trt:GetProfileResponse" : "trt:GetProfilesResponse",
            single ? "trt:Profile" : "trt:Profiles",
            ptz_cfg,
            single ? "trt:Profile" : "trt:Profiles",
            single ? "trt:GetProfileResponse" : "trt:GetProfilesResponse");
        soap_wrap(resp_xml, resp_size, soap11, b);
        return 200;
    }

    if (xml_find_tag_open(body, "GetStreamUri")) {
        char b[1024];
        snprintf(b, sizeof(b),
            "<trt:GetStreamUriResponse><trt:MediaUri>"
            "<tt:Uri>rtsp://%s:554/stream=0</tt:Uri>"
            "<tt:InvalidAfterConnect>false</tt:InvalidAfterConnect>"
            "<tt:InvalidAfterReboot>false</tt:InvalidAfterReboot>"
            "<tt:Timeout>PT60S</tt:Timeout>"
            "</trt:MediaUri></trt:GetStreamUriResponse>",
            host_only);
        soap_wrap(resp_xml, resp_size, soap11, b);
        return 200;
    }

    if (xml_find_tag_open(body, "GetSnapshotUri")) {
        char b[1024];
        snprintf(b, sizeof(b),
            "<trt:GetSnapshotUriResponse><trt:MediaUri>"
            "<tt:Uri>http://%s:80/image.jpg</tt:Uri>"
            "<tt:InvalidAfterConnect>false</tt:InvalidAfterConnect>"
            "<tt:InvalidAfterReboot>false</tt:InvalidAfterReboot>"
            "<tt:Timeout>PT60S</tt:Timeout>"
            "</trt:MediaUri></trt:GetSnapshotUriResponse>",
            host_only);
        soap_wrap(resp_xml, resp_size, soap11, b);
        return 200;
    }

    /* ---------------- PTZ Service ---------------- */

    if (xml_find_tag_open(body, "GetServiceCapabilities")) {
        soap_wrap(resp_xml, resp_size, soap11,
            "<tptz:GetServiceCapabilitiesResponse>"
            "<tptz:Capabilities EFlip=\"false\" Reverse=\"false\" "
            "GetCompatibleConfigurations=\"true\" MoveStatus=\"true\" StatusPosition=\"true\"/>"
            "</tptz:GetServiceCapabilitiesResponse>");
        return 200;
    }

    if (xml_find_tag_open(body, "GetConfigurationOptions")) {
        char spaces[3072];
        build_ptz_spaces_xml(spaces, sizeof(spaces));
        char b[4096];
        snprintf(b, sizeof(b),
            "<tptz:GetConfigurationOptionsResponse><tptz:PTZConfigurationOptions>"
            "<tt:Spaces>%s</tt:Spaces>"
            "<tt:PTZTimeout><tt:Min>PT1S</tt:Min><tt:Max>PT120S</tt:Max></tt:PTZTimeout>"
            "</tptz:PTZConfigurationOptions></tptz:GetConfigurationOptionsResponse>",
            spaces);
        soap_wrap(resp_xml, resp_size, soap11, b);
        return 200;
    }

    if (xml_find_tag_open(body, "GetConfigurations") ||
        xml_find_tag_open(body, "GetCompatibleConfigurations") ||
        xml_find_tag_open(body, "GetConfiguration")) {
        char ptz_cfg[2048];
        build_ptz_config_xml(ptz_cfg, sizeof(ptz_cfg));
        const char *tag = xml_find_tag_open(body, "GetCompatibleConfigurations")
            ? "tptz:GetCompatibleConfigurationsResponse"
            : xml_find_tag_open(body, "GetConfigurations")
                ? "tptz:GetConfigurationsResponse"
                : "tptz:GetConfigurationResponse";
        char b[3072];
        snprintf(b, sizeof(b), "<%s>%s</%s>", tag, ptz_cfg, tag);
        soap_wrap(resp_xml, resp_size, soap11, b);
        return 200;
    }

    if (xml_find_tag_open(body, "GetNodes") || xml_find_tag_open(body, "GetNode")) {
        bool single = (xml_find_tag_open(body, "GetNodes") == NULL);
        char spaces[3072];
        build_ptz_spaces_xml(spaces, sizeof(spaces));
        char b[4096];
        snprintf(b, sizeof(b),
            "<%s><%s token=\"PTZ_NODE_000\" FixedHomePosition=\"false\">"
            "<tt:Name>PTZ_NODE_000</tt:Name>"
            "<tt:SupportedPTZSpaces>%s</tt:SupportedPTZSpaces>"
            "<tt:MaximumNumberOfPresets>%d</tt:MaximumNumberOfPresets>"
            "<tt:HomeSupported>true</tt:HomeSupported>"
            "</%s></%s>",
            single ? "tptz:GetNodeResponse" : "tptz:GetNodesResponse",
            single ? "tptz:PTZNode" : "tptz:PTZNode",
            spaces, MAX_PRESETS,
            "tptz:PTZNode",
            single ? "tptz:GetNodeResponse" : "tptz:GetNodesResponse");
        soap_wrap(resp_xml, resp_size, soap11, b);
        return 200;
    }

    if (xml_find_tag_open(body, "GetStatus")) {
        if (strstr(headers, "/image_service") != NULL) {
            long pan = 0, tilt = 0, focus = 0;
            bool live_mv = false;
            ptz_pos_read_live(&pan, &tilt, &focus, &live_mv);
            double fnorm = clamp_d((double)focus / 20000.0, 0.0, 1.0);
            char b[512];
            snprintf(b, sizeof(b),
                "<timg:GetStatusResponse><timg:Status>"
                "<tt:FocusStatus20><tt:Position>%.4f</tt:Position>"
                "<tt:MoveStatus>%s</tt:MoveStatus></tt:FocusStatus20>"
                "</timg:Status></timg:GetStatusResponse>",
                fnorm, live_mv ? "MOVING" : "IDLE");
            soap_wrap(resp_xml, resp_size, soap11, b);
            return 200;
        }
        long pan = 0, tilt = 0, focus = 0;
        bool live_mv = false;
        ptz_pos_read_live(&pan, &tilt, &focus, &live_mv);
        double mag = current_mag_or_cached();
        double pan_norm = clamp_d((double)pan / g_cfg.pan_range_ms, -1.0, 1.0);
        double tilt_norm = clamp_d((double)tilt / g_cfg.tilt_range_ms, -1.0, 1.0);
        double zoom_norm = clamp_d((mag - 1.0) / (g_cfg.max_zoom - 1.0), 0.0, 1.0);

        pthread_mutex_lock(&g_mu);
        bool pt_mv = g_pt_moving || live_mv;
        bool zm_mv = g_zoom_moving;
        pthread_mutex_unlock(&g_mu);

        time_t now = time(NULL);
        struct tm utc;
        gmtime_r(&now, &utc);
        char iso[32];
        strftime(iso, sizeof(iso), "%Y-%m-%dT%H:%M:%SZ", &utc);

        char b[1536];
        snprintf(b, sizeof(b),
            "<tptz:GetStatusResponse><tptz:PTZStatus>"
            "<tt:Position>"
            "<tt:PanTilt x=\"%.4f\" y=\"%.4f\" space=\"http://www.onvif.org/ver10/tptz/PanTiltSpaces/PositionGenericSpace\"/>"
            "<tt:Zoom x=\"%.4f\" space=\"http://www.onvif.org/ver10/tptz/ZoomSpaces/PositionGenericSpace\"/>"
            "</tt:Position>"
            "<tt:MoveStatus><tt:PanTilt>%s</tt:PanTilt><tt:Zoom>%s</tt:Zoom></tt:MoveStatus>"
            "<tt:UtcTime>%s</tt:UtcTime>"
            "</tptz:PTZStatus></tptz:GetStatusResponse>",
            pan_norm, tilt_norm, zoom_norm,
            pt_mv ? "MOVING" : "IDLE",
            zm_mv ? "MOVING" : "IDLE",
            iso);
        soap_wrap(resp_xml, resp_size, soap11, b);
        return 200;
    }

    if (xml_find_tag_open(body, "ContinuousMove")) {
        char vel_xml[1024] = "";
        const char *src = xml_get_element(body, "Velocity", vel_xml, sizeof(vel_xml)) ? vel_xml : body;
        struct ptz_job job;
        memset(&job, 0, sizeof(job));
        job.kind = JOB_CONTINUOUS;
        parse_ptz_vector(src, &job.vx, &job.vy, &job.has_pt, &job.vz, &job.has_zoom);
        char tout[64] = "";
        if (xml_get_text(body, "Timeout", tout, sizeof(tout)))
            job.timeout_ms = parse_iso_duration_ms(tout);
        if (g_cfg.debug) {
            fprintf(stderr, "motors-onvif: ContinuousMove pt=(%.3f,%.3f,%d) z=(%.3f,%d) tout=%u\n",
                    job.vx, job.vy, job.has_pt, job.vz, job.has_zoom, job.timeout_ms);
        }
        submit_job(job);
        soap_wrap(resp_xml, resp_size, soap11, "<tptz:ContinuousMoveResponse/>");
        return 200;
    }

    if (xml_find_tag_open(body, "RelativeMove")) {
        char trans_xml[1024] = "";
        const char *src = xml_get_element(body, "Translation", trans_xml, sizeof(trans_xml)) ? trans_xml : body;
        struct ptz_job job;
        memset(&job, 0, sizeof(job));
        job.kind = JOB_RELATIVE;
        parse_ptz_vector(src, &job.vx, &job.vy, &job.has_pt, &job.vz, &job.has_zoom);
        submit_job(job);
        soap_wrap(resp_xml, resp_size, soap11, "<tptz:RelativeMoveResponse/>");
        return 200;
    }

    if (xml_find_tag_open(body, "AbsoluteMove")) {
        char pos_xml[1024] = "";
        const char *src = xml_get_element(body, "Position", pos_xml, sizeof(pos_xml)) ? pos_xml : body;
        struct ptz_job job;
        memset(&job, 0, sizeof(job));
        job.kind = JOB_ABSOLUTE;
        parse_ptz_vector(src, &job.vx, &job.vy, &job.has_pt, &job.vz, &job.has_zoom);
        submit_job(job);
        soap_wrap(resp_xml, resp_size, soap11, "<tptz:AbsoluteMoveResponse/>");
        return 200;
    }

    if (xml_find_tag_open(body, "Stop")) {
        if (g_cfg.debug) fprintf(stderr, "motors-onvif: Stop\n");
        cancel_all_jobs();
        soap_wrap(resp_xml, resp_size, soap11, "<tptz:StopResponse/>");
        return 200;
    }

    if (xml_find_tag_open(body, "GetPresets")) {
        struct preset_entry list[MAX_PRESETS];
        size_t n = load_presets(list, MAX_PRESETS);
        char b[16384];
        size_t pos = (size_t)snprintf(b, sizeof(b), "<tptz:GetPresetsResponse>");
        for (size_t i = 0; i < n && pos + 512 < sizeof(b); i++) {
            char esc_name[256];
            xml_escape(list[i].name, esc_name, sizeof(esc_name));
            double px = clamp_d((double)list[i].pan_ms / g_cfg.pan_range_ms, -1.0, 1.0);
            double py = clamp_d((double)list[i].tilt_ms / g_cfg.tilt_range_ms, -1.0, 1.0);
            double pz = list[i].has_mag
                ? clamp_d((list[i].mag - 1.0) / (g_cfg.max_zoom - 1.0), 0.0, 1.0)
                : 0.0;
            pos += (size_t)snprintf(b + pos, sizeof(b) - pos,
                "<tptz:Preset token=\"%d\">"
                "<tt:Name>%s</tt:Name>"
                "<tt:PTZPosition>"
                "<tt:PanTilt x=\"%.4f\" y=\"%.4f\" space=\"http://www.onvif.org/ver10/tptz/PanTiltSpaces/PositionGenericSpace\"/>"
                "<tt:Zoom x=\"%.4f\" space=\"http://www.onvif.org/ver10/tptz/ZoomSpaces/PositionGenericSpace\"/>"
                "</tt:PTZPosition>"
                "</tptz:Preset>",
                list[i].id, esc_name, px, py, pz);
        }
        snprintf(b + pos, sizeof(b) - pos, "</tptz:GetPresetsResponse>");
        soap_wrap(resp_xml, resp_size, soap11, b);
        return 200;
    }

    if (xml_find_tag_open(body, "GotoPreset") || xml_find_tag_open(body, "GotoHomePosition")) {
        bool is_home = (xml_find_tag_open(body, "GotoHomePosition") != NULL);
        char tok[64] = "";
        if (is_home) {
            snprintf(tok, sizeof(tok), "%d", read_boot_home_id());
        } else {
            xml_get_text(body, "PresetToken", tok, sizeof(tok));
        }

        struct preset_entry list[MAX_PRESETS];
        size_t n = load_presets(list, MAX_PRESETS);
        int found = -1;
        int want_id = atoi(tok);
        for (size_t i = 0; i < n; i++) {
            if ((want_id > 0 && list[i].id == want_id) ||
                strcasecmp(list[i].name, tok) == 0) {
                found = (int)i;
                break;
            }
        }
        if (found >= 0) {
            struct ptz_job job;
            memset(&job, 0, sizeof(job));
            job.kind = JOB_PRESET;
            job.preset = list[found];
            job.has_pt = true;
            job.has_zoom = list[found].zoom_restore && list[found].has_mag;
            submit_job(job);
        }
        soap_wrap(resp_xml, resp_size, soap11,
                  is_home ? "<tptz:GotoHomePositionResponse/>" : "<tptz:GotoPresetResponse/>");
        return 200;
    }

    if (xml_find_tag_open(body, "SetPreset")) {
        char name[64] = "", tok[64] = "";
        xml_get_text(body, "PresetName", name, sizeof(name));
        xml_get_text(body, "PresetToken", tok, sizeof(tok));

        struct preset_entry list[MAX_PRESETS];
        size_t n = load_presets(list, MAX_PRESETS);
        int want_id = atoi(tok);
        int idx = -1;
        if (want_id > 0) {
            for (size_t i = 0; i < n; i++) {
                if (list[i].id == want_id) { idx = (int)i; break; }
            }
        }
        if (idx < 0 && name[0]) {
            for (size_t i = 0; i < n; i++) {
                if (strcasecmp(list[i].name, name) == 0) { idx = (int)i; break; }
            }
        }
        if (idx < 0) {
            int next_id = 1;
            for (int cand = 1; cand <= MAX_PRESETS; cand++) {
                bool used = false;
                for (size_t i = 0; i < n; i++) if (list[i].id == cand) used = true;
                if (!used) { next_id = cand; break; }
            }
            if (n < MAX_PRESETS) {
                idx = (int)n++;
                memset(&list[idx], 0, sizeof(list[idx]));
                list[idx].id = want_id > 0 && want_id <= MAX_PRESETS ? want_id : next_id;
                list[idx].zoom_restore = 1;
            }
        }
        int saved_id = 1;
        if (idx >= 0) {
            long pan = 0, tilt = 0, focus = 0;
            ptz_pos_read(&pan, &tilt, &focus);
            double mag = current_mag_or_cached();
            if (name[0]) snprintf(list[idx].name, sizeof(list[idx].name), "%.63s", name);
            else if (!list[idx].name[0]) snprintf(list[idx].name, sizeof(list[idx].name), "Preset %d", list[idx].id);
            list[idx].pan_ms = pan;
            list[idx].tilt_ms = tilt;
            list[idx].focus_ms = focus;
            list[idx].has_focus = true;
            list[idx].mag = mag;
            list[idx].has_mag = true;
            save_presets(list, n);
            saved_id = list[idx].id;
        }
        char b[256];
        snprintf(b, sizeof(b),
            "<tptz:SetPresetResponse><tptz:PresetToken>%d</tptz:PresetToken></tptz:SetPresetResponse>",
            saved_id);
        soap_wrap(resp_xml, resp_size, soap11, b);
        return 200;
    }

    if (xml_find_tag_open(body, "RemovePreset")) {
        char tok[64] = "";
        xml_get_text(body, "PresetToken", tok, sizeof(tok));
        int want_id = atoi(tok);
        struct preset_entry list[MAX_PRESETS];
        size_t n = load_presets(list, MAX_PRESETS);
        size_t out_n = 0;
        for (size_t i = 0; i < n; i++) {
            if (want_id > 0 && list[i].id == want_id) continue;
            list[out_n++] = list[i];
        }
        save_presets(list, out_n);
        soap_wrap(resp_xml, resp_size, soap11, "<tptz:RemovePresetResponse/>");
        return 200;
    }

    if (xml_find_tag_open(body, "SetHomePosition")) {
        soap_wrap(resp_xml, resp_size, soap11, "<tptz:SetHomePositionResponse/>");
        return 200;
    }

    /* ---------------- Imaging Service (Focus / AF) ---------------- */

    if (xml_find_tag_open(body, "GetMoveOptions")) {
        soap_wrap(resp_xml, resp_size, soap11,
            "<timg:GetMoveOptionsResponse><timg:MoveOptions>"
            "<tt:Relative><tt:Distance><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:Distance>"
            "<tt:Speed><tt:Min>0</tt:Min><tt:Max>1</tt:Max></tt:Speed></tt:Relative>"
            "<tt:Continuous><tt:Speed><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:Speed></tt:Continuous>"
            "</timg:MoveOptions></timg:GetMoveOptionsResponse>");
        return 200;
    }

    if (xml_find_tag_open(body, "GetImagingSettings")) {
        soap_wrap(resp_xml, resp_size, soap11,
            "<timg:GetImagingSettingsResponse><timg:ImagingSettings>"
            "<tt:Focus><tt:AutoFocusMode>AUTO</tt:AutoFocusMode><tt:DefaultSpeed>0.5</tt:DefaultSpeed></tt:Focus>"
            "</timg:ImagingSettings></timg:GetImagingSettingsResponse>");
        return 200;
    }

    if (xml_find_tag_open(body, "SetImagingSettings")) {
        char mode[32] = "";
        if (xml_get_text(body, "AutoFocusMode", mode, sizeof(mode)) &&
            strcasecmp(mode, "AUTO") == 0) {
            trigger_local_af();
        }
        soap_wrap(resp_xml, resp_size, soap11, "<timg:SetImagingSettingsResponse/>");
        return 200;
    }

    if (xml_find_tag_open(body, "Move") && strstr(headers, "/image_service") != NULL) {
        char spd[32] = "", dist[32] = "";
        struct ptz_job job;
        memset(&job, 0, sizeof(job));
        job.kind = JOB_FOCUS;
        if (xml_get_text(body, "Distance", dist, sizeof(dist))) {
            double d = atof(dist);
            job.vz = d;
            job.timeout_ms = (unsigned)clamp_d(fabs(d) * 2000.0, 80.0, 3000.0);
        } else if (xml_get_text(body, "Speed", spd, sizeof(spd))) {
            job.vz = atof(spd);
            job.timeout_ms = 2000U;
        }
        if (fabs(job.vz) > 0.01) submit_job(job);
        soap_wrap(resp_xml, resp_size, soap11, "<timg:MoveResponse/>");
        return 200;
    }

    soap_wrap(resp_xml, resp_size, soap11,
        "<s:Fault><s:Code><s:Value>s:Receiver</s:Value>"
        "<s:Subcode><s:Value xmlns:ter=\"http://www.onvif.org/ver10/error\">ter:ActionNotSupported</s:Value></s:Subcode></s:Code>"
        "<s:Reason><s:Text xml:lang=\"en\">Action not supported</s:Text></s:Reason></s:Fault>");
    return 400;
}

/* ---------------- HTTP Server ---------------- */

static void send_all(int fd, const char *buf, size_t len) {
    size_t sent = 0;
    while (sent < len) {
        ssize_t n = send(fd, buf + sent, len - sent, MSG_NOSIGNAL);
        if (n <= 0) {
            if (n < 0 && errno == EINTR) continue;
            break;
        }
        sent += (size_t)n;
    }
}

static void handle_client(int cfd) {
    struct timeval tv = { .tv_sec = 5, .tv_usec = 0 };
    setsockopt(cfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(cfd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    char *req = calloc(1, MAX_REQ_BYTES);
    char *resp_xml = calloc(1, MAX_RESP_BYTES);
    if (!req || !resp_xml) {
        free(req);
        free(resp_xml);
        close(cfd);
        return;
    }

    size_t total = 0;
    char *hdr_end = NULL;
    size_t content_len = 0;
    while (total + 1 < MAX_REQ_BYTES) {
        ssize_t n = recv(cfd, req + total, MAX_REQ_BYTES - 1 - total, 0);
        if (n <= 0) break;
        total += (size_t)n;
        req[total] = '\0';
        if (!hdr_end) {
            hdr_end = strstr(req, "\r\n\r\n");
            if (hdr_end) {
                const char *cl = req;
                while (cl < hdr_end) {
                    if (strncasecmp(cl, "Content-Length:", 15) == 0) {
                        content_len = (size_t)strtoul(cl + 15, NULL, 10);
                        break;
                    }
                    const char *next = strstr(cl, "\r\n");
                    if (!next) break;
                    cl = next + 2;
                }
            }
        }
        if (hdr_end) {
            size_t body_got = total - (size_t)((hdr_end + 4) - req);
            if (body_got >= content_len) break;
        }
    }

    if (hdr_end) {
        *hdr_end = '\0';
        const char *body = hdr_end + 4;

        /* Determine Host header or local IP */
        char host_port[128] = "";
        const char *h = req;
        while (h && *h) {
            if (strncasecmp(h, "Host:", 5) == 0) {
                const char *v = h + 5;
                while (*v == ' ' || *v == '\t') v++;
                size_t i = 0;
                while (*v && *v != '\r' && *v != '\n' && i + 1 < sizeof(host_port))
                    host_port[i++] = *v++;
                host_port[i] = '\0';
                break;
            }
            const char *nl = strstr(h, "\r\n");
            if (!nl) break;
            h = nl + 2;
        }
        if (!host_port[0]) {
            struct sockaddr_in sa;
            socklen_t sl = sizeof(sa);
            char ip[64] = "127.0.0.1";
            if (getsockname(cfd, (struct sockaddr *)&sa, &sl) == 0) {
                inet_ntop(AF_INET, &sa.sin_addr, ip, sizeof(ip));
            }
            snprintf(host_port, sizeof(host_port), "%s:%d", ip, g_cfg.port);
        }
        char host_only[128];
        snprintf(host_only, sizeof(host_only), "%s", host_port);
        char *colon = strchr(host_only, ':');
        if (colon) *colon = '\0';

        int status = 200;
        if (strncmp(req, "POST ", 5) == 0) {
            status = handle_soap_request(req, body, host_port, host_only, resp_xml, MAX_RESP_BYTES);
        } else {
            snprintf(resp_xml, MAX_RESP_BYTES, "motors-onvif listening on port %d\n", g_cfg.port);
        }

        const char *status_text = (status == 200) ? "OK"
                                : (status == 401) ? "Unauthorized"
                                : "Bad Request";
        const char *ctype = (strncmp(req, "POST ", 5) == 0)
            ? (strstr(body, "schemas.xmlsoap.org/soap/envelope")
                ? "text/xml; charset=utf-8"
                : "application/soap+xml; charset=utf-8")
            : "text/plain; charset=utf-8";

        char hdr[512];
        int hlen = snprintf(hdr, sizeof(hdr),
            "HTTP/1.1 %d %s\r\n"
            "Content-Type: %s\r\n"
            "%s"
            "Content-Length: %zu\r\n"
            "Connection: close\r\n\r\n",
            status, status_text, ctype,
            status == 401 ? "WWW-Authenticate: Basic realm=\"ONVIF Device\"\r\n" : "",
            strlen(resp_xml));
        send_all(cfd, hdr, (size_t)hlen);
        send_all(cfd, resp_xml, strlen(resp_xml));
    }

    free(req);
    free(resp_xml);
    close(cfd);
}

/* Read onvif.username / onvif.password from /etc/majestic.yaml if not passed via CLI. */
static void load_majestic_onvif_creds(void) {
    if (g_cfg.user[0] && g_cfg.pass[0]) return;
    FILE *f = fopen(MAJESTIC_YAML, "r");
    if (!f) return;
    bool in_onvif = false;
    char line[256];
    while (fgets(line, sizeof(line), f)) {
        if (!isspace((unsigned char)line[0]) && line[0] != '#' && line[0] != '\n') {
            in_onvif = (strncmp(line, "onvif:", 6) == 0);
            continue;
        }
        if (!in_onvif) continue;
        char *p = line;
        while (*p == ' ' || *p == '\t') p++;
        char *nl = p + strlen(p);
        while (nl > p && (nl[-1] == '\n' || nl[-1] == '\r' || nl[-1] == ' ' || nl[-1] == '\t'))
            *--nl = '\0';
        if (strncmp(p, "username:", 9) == 0 && !g_cfg.user[0]) {
            char *v = p + 9;
            while (*v == ' ' || *v == '\t' || *v == '"' || *v == '\'') v++;
            size_t len = strlen(v);
            while (len > 0 && (v[len - 1] == '"' || v[len - 1] == '\'')) v[--len] = '\0';
            snprintf(g_cfg.user, sizeof(g_cfg.user), "%.63s", v);
        } else if (strncmp(p, "password:", 9) == 0 && !g_cfg.pass[0]) {
            char *v = p + 9;
            while (*v == ' ' || *v == '\t' || *v == '"' || *v == '\'') v++;
            size_t len = strlen(v);
            while (len > 0 && (v[len - 1] == '"' || v[len - 1] == '\'')) v[--len] = '\0';
            snprintf(g_cfg.pass, sizeof(g_cfg.pass), "%.127s", v);
        }
    }
    fclose(f);
}

static void load_onvif_conf(const char *path) {
    FILE *f = fopen(path, "r");
    if (!f) return;
    char line[256];
    while (fgets(line, sizeof(line), f)) {
        char *p = line;
        while (*p == ' ' || *p == '\t') p++;
        if (!*p || *p == '#' || *p == '\n' || *p == '\r') continue;
        char *eq = strchr(p, '=');
        if (!eq) continue;
        *eq = '\0';
        char *k = p;
        char *ke = eq;
        while (ke > k && (ke[-1] == ' ' || ke[-1] == '\t')) *--ke = '\0';
        char *v = eq + 1;
        while (*v == ' ' || *v == '\t') v++;
        char *ve = v + strlen(v);
        while (ve > v && (ve[-1] == '\n' || ve[-1] == '\r' || ve[-1] == ' ' || ve[-1] == '\t'))
            *--ve = '\0';
        if (!strcmp(k, "port")) g_cfg.port = atoi(v);
        else if (!strcmp(k, "socket")) snprintf(g_cfg.socket_path, sizeof(g_cfg.socket_path), "%.255s", v);
        else if (!strcmp(k, "presets")) snprintf(g_cfg.presets_path, sizeof(g_cfg.presets_path), "%.255s", v);
        else if (!strcmp(k, "user")) snprintf(g_cfg.user, sizeof(g_cfg.user), "%.63s", v);
        else if (!strcmp(k, "pass")) snprintf(g_cfg.pass, sizeof(g_cfg.pass), "%.127s", v);
        else if (!strcmp(k, "pan_scale")) g_cfg.pan_scale_ms = atof(v);
        else if (!strcmp(k, "tilt_scale")) g_cfg.tilt_scale_ms = atof(v);
        else if (!strcmp(k, "max_zoom")) g_cfg.max_zoom = atof(v);
        else if (!strcmp(k, "af_settle_ms")) g_cfg.af_settle_ms = (unsigned)atoi(v);
        else if (!strcmp(k, "post_zoom_af")) g_cfg.post_zoom_af = (strcmp(v, "1") == 0 || strcasecmp(v, "true") == 0);
        else if (!strcmp(k, "invert_pan")) g_cfg.invert_pan = (strcmp(v, "1") == 0 || strcasecmp(v, "true") == 0);
        else if (!strcmp(k, "invert_tilt")) g_cfg.invert_tilt = (strcmp(v, "1") == 0 || strcasecmp(v, "true") == 0);
    }
    fclose(f);
}

int main(int argc, char **argv) {
    for (int i = 1; i < argc - 1; i++) {
        if (!strcmp(argv[i], "--config")) {
            snprintf(g_cfg.config_path, sizeof(g_cfg.config_path), "%.255s", argv[i + 1]);
            break;
        }
    }
    load_onvif_conf(g_cfg.config_path);

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--config") && i + 1 < argc) {
            i++;
        } else if (!strcmp(argv[i], "--port") && i + 1 < argc) {
            g_cfg.port = atoi(argv[++i]);
        } else if (!strcmp(argv[i], "--socket") && i + 1 < argc) {
            snprintf(g_cfg.socket_path, sizeof(g_cfg.socket_path), "%.255s", argv[++i]);
        } else if (!strcmp(argv[i], "--presets") && i + 1 < argc) {
            snprintf(g_cfg.presets_path, sizeof(g_cfg.presets_path), "%.255s", argv[++i]);
        } else if (!strcmp(argv[i], "--user") && i + 1 < argc) {
            snprintf(g_cfg.user, sizeof(g_cfg.user), "%.63s", argv[++i]);
        } else if (!strcmp(argv[i], "--pass") && i + 1 < argc) {
            snprintf(g_cfg.pass, sizeof(g_cfg.pass), "%.127s", argv[++i]);
        } else if (!strcmp(argv[i], "--pan-scale") && i + 1 < argc) {
            g_cfg.pan_scale_ms = atof(argv[++i]);
        } else if (!strcmp(argv[i], "--tilt-scale") && i + 1 < argc) {
            g_cfg.tilt_scale_ms = atof(argv[++i]);
        } else if (!strcmp(argv[i], "--max-zoom") && i + 1 < argc) {
            g_cfg.max_zoom = atof(argv[++i]);
        } else if (!strcmp(argv[i], "--no-post-zoom-af")) {
            g_cfg.post_zoom_af = false;
        } else if (!strcmp(argv[i], "--invert-pan")) {
            g_cfg.invert_pan = true;
        } else if (!strcmp(argv[i], "--invert-tilt")) {
            g_cfg.invert_tilt = true;
        } else if (!strcmp(argv[i], "--debug")) {
            g_cfg.debug = true;
        } else if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h")) {
            fprintf(stderr,
                "Usage: motors-onvif [--config /etc/motorsd/onvif.conf] [--port 8899]\n"
                "                    [--socket /run/motorsd.sock] [--presets /etc/ptz_presets.tsv]\n"
                "                    [--user USER --pass PASS] [--pan-scale 10600] [--tilt-scale 6400]\n"
                "                    [--max-zoom 57.0] [--no-post-zoom-af] [--invert-pan] [--invert-tilt] [--debug]\n");
            return 0;
        }
    }

    load_majestic_onvif_creds();

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);
    signal(SIGPIPE, SIG_IGN);

    pthread_t worker;
    if (pthread_create(&worker, NULL, worker_main, NULL) != 0) {
        perror("pthread_create");
        return 1;
    }

    int sfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sfd < 0) {
        perror("socket");
        return 1;
    }
    int opt = 1;
    setsockopt(sfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons((uint16_t)g_cfg.port);

    if (bind(sfd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        perror("bind");
        close(sfd);
        return 1;
    }
    if (listen(sfd, 16) != 0) {
        perror("listen");
        close(sfd);
        return 1;
    }

    fprintf(stderr, "motors-onvif: listening on :%d (socket=%s user=%s pan_scale=%.0f tilt_scale=%.0f)\n",
            g_cfg.port, g_cfg.socket_path, g_cfg.user[0] ? g_cfg.user : "<none>",
            g_cfg.pan_scale_ms, g_cfg.tilt_scale_ms);

    while (!g_stop_daemon) {
        struct sockaddr_in peer;
        socklen_t plen = sizeof(peer);
        int cfd = accept(sfd, (struct sockaddr *)&peer, &plen);
        if (cfd < 0) {
            if (errno == EINTR) continue;
            break;
        }
        handle_client(cfd);
    }

    close(sfd);
    pthread_mutex_lock(&g_mu);
    g_stop_daemon = 1;
    pthread_cond_broadcast(&g_cv);
    pthread_mutex_unlock(&g_mu);
    pthread_join(worker, NULL);
    return 0;
}
