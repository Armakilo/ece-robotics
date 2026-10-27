#include "telemetry.h"
#include "FS.h"
#include "SPIFFS.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define LOG_SECONDS     300  // keep the newest 300 s of data (~6 laps with stops and passes), the oldest rows are dropped
                             // (was 90 s, only ~2 laps). 1500 rows x 172 B = 258 KB in PSRAM.
#define LOG_RATE_HZ     5    // the UNO sends 5 rows per second
#define LOG_ROWS        (LOG_SECONDS * LOG_RATE_HZ)
#define LOG_HEADER      "time_ms,heading_gyro,heading_pred,x_pred_m,y_pred_m,heading_est,x_est_m,y_est_m," \
                        "lap_time_s,lap_dist_m,laps,state,pos_sigma_m,speed_scale," \
                        "sign_stop_x_m,sign_stop_y_m,sign_x_m,sign_y_m,cam_fix_dx_m,cam_fix_dy_m,map_match," \
                        "lap_done,lap_done_time_s,lap_done_dist_m,lap_avg_speed_mps," \
                        "corner_leg,corner_car_x_m,corner_car_y_m,corner_fix_dx_m,corner_fix_dy_m,corner_used," \
                        "obst_x_m,obst_y_m,pass_side,obst_fix_dx_m,obst_fix_dy_m,obst_fix_used," \
                        "pass_time_s,pass_line_found," \
                        "light_car_x_m,light_car_y_m,light_x_m,light_y_m,light_fix_dx_m,light_fix_dy_m\n"
#define SNAPSHOT_PATH   "/log.bin"
// Saving ~40 KB to flash blocks the ESP32 for about a second, and the UNO rows sent during that
// time were lost (the 16:2x log was missing ~1 s every 5 s, including a stop-sign event).
// So the snapshot is only saved while the car is E-stopped, never during a run.
#define DATA_STALE_MS   2000 // page shows "no data" if nothing arrived for this long

struct LogRow {
    uint32_t t;      // UNO millis()
    int32_t actual;  // tenths of a degree
    int32_t pred;    // tenths of a degree
    bool pred_valid; // false until the kinematic model sends a value
    int32_t x_mm;    // predicted position (model only), millimetres
    int32_t y_mm;
    int32_t est;     // fused heading estimate, tenths of a degree
    int32_t xe_mm;   // fused position estimate, millimetres
    int32_t ye_mm;
    uint16_t lap_ds; // current lap time, tenths of a second
    uint16_t lap_cm; // current lap distance, cm
    uint8_t laps;    // laps completed
    uint16_t sig_mm; // EKF position uncertainty (1 sigma), mm
    uint16_t scale;  // EKF speed scale x 1000
    char state[13];
    // Events, only filled on the row right after they happened (blank in the CSV otherwise)
    bool sign;           // stopped at a stop sign ({SG,...} from the UNO)
    int32_t stop_x_mm;   // fused estimate where the car stopped, before the camera fix
    int32_t stop_y_mm;
    int32_t sign_x_mm;   // estimated stop sign position
    int32_t sign_y_mm;
    int32_t fix_dx_mm;   // camera correction applied to the estimate
    int32_t fix_dy_mm;
    int8_t match;        // STOP_MAP entry used, -1 = none in range (no correction)
    bool lap;            // a lap was completed ({LP,...} from the UNO)
    uint8_t lap_n;
    uint32_t lap_ms;
    uint32_t lap_mm;
    bool cf;             // corner fix ({CF,leg,car_x,car_y,fix_dx,fix_dy,used})
    int8_t cf_leg;
    int32_t cf_x, cf_y, cf_dx, cf_dy; // car estimate before the fix, and the fix (mm)
    bool cf_used;
    bool ob;             // overtake started ({OB,obst_x,obst_y,side,fix_dx,fix_dy,used})
    int32_t ob_x, ob_y, ob_dx, ob_dy; // obstacle estimate (before the moose fix), and the fix (mm)
    uint8_t ob_side;     // 0 = passed on the left, 1 = right
    bool ob_used;
    bool ov;             // overtake finished ({OV,ms,side,line_found})
    uint32_t ov_ms;
    bool ov_found;
    bool tl;             // traffic light seen ({TL,car_x,car_y,light_x,light_y,fix_dx,fix_dy})
    int32_t tl_cx, tl_cy, tl_x, tl_y, tl_dx, tl_dy;
};

// Events waiting for the next {T,...} row, and the latest ones for the web page
static LogRow pending = {};
static LogRow last_sign = {};
static LogRow last_lap = {};

// Ring buffer of the newest LOG_ROWS rows. Written by loop(), read by the web server task.
static SemaphoreHandle_t log_mutex = NULL;
static LogRow *rows = NULL; // LOG_ROWS rows (~258 KB), allocated in PSRAM by telemetry_init()
#define ROWS_BYTES (sizeof(LogRow) * LOG_ROWS)
static uint16_t head = 0;  // slot the next row goes in
static uint16_t count = 0; // rows stored
static bool fs_ok = false;
static bool unsaved = false; // rows have arrived since the last snapshot
static unsigned long last_rx = 0; // ESP32 millis() when the last row arrived

// i-th oldest row. Call with log_mutex held.
static const LogRow &row_at(uint16_t i)
{
    return rows[(head + LOG_ROWS - count + i) % LOG_ROWS];
}

// Saves the ring buffer to flash. Call with log_mutex held.
static void save_snapshot(void)
{
    File f = SPIFFS.open(SNAPSHOT_PATH, FILE_WRITE);
    if (!f)
        return;
    f.write((const uint8_t *)&head, sizeof(head));
    f.write((const uint8_t *)&count, sizeof(count));
    f.write((const uint8_t *)rows, ROWS_BYTES);
    f.close();
}

// Loads the rows saved before the last reset, if any
static void load_snapshot(void)
{
    File f = SPIFFS.open(SNAPSHOT_PATH, FILE_READ);
    if (!f)
        return;
    if (f.size() == sizeof(head) + sizeof(count) + ROWS_BYTES) {
        f.read((uint8_t *)&head, sizeof(head));
        f.read((uint8_t *)&count, sizeof(count));
        f.read((uint8_t *)rows, ROWS_BYTES);
        if (head >= LOG_ROWS || count > LOG_ROWS)
            head = count = 0;
    }
    f.close();
}

void telemetry_init(void)
{
    log_mutex = xSemaphoreCreateMutex();
    rows = (LogRow *)ps_calloc(LOG_ROWS, sizeof(LogRow));
    if (!rows)
        rows = (LogRow *)calloc(LOG_ROWS, sizeof(LogRow));
    if (!rows) {
        Serial.println("telemetry: out of memory for the log");
        return;
    }
    // true = format the flash the first time (takes a few seconds once)
    fs_ok = SPIFFS.begin(true);
    if (!fs_ok) {
        Serial.println("telemetry: SPIFFS mount failed, check Partition Scheme = Huge APP (3MB No OTA/1MB SPIFFS)");
        return;
    }
    SPIFFS.remove("/log.csv"); // old ever-growing log from the previous version
    load_snapshot();
    Serial.printf("telemetry: keeping the last %d s, %u rows restored from flash\n", LOG_SECONDS, count);
}

// Formats tenths of a degree as "12.3" / "-0.5"
static void tenths_to_str(long v, char *out, size_t len)
{
    snprintf(out, len, "%s%ld.%ld", v < 0 ? "-" : "", labs(v) / 10, labs(v) % 10);
}

// Formats millimetres as metres, "1.234" / "-0.050"
static void mm_to_str(long v, char *out, size_t len)
{
    snprintf(out, len, "%s%ld.%03ld", v < 0 ? "-" : "", labs(v) / 1000, labs(v) % 1000);
}

void telemetry_handle_uno_msg(const String &msg)
{
    // Messages can have leftover bytes in front (e.g. "\r\n" from a println), so start at the last '{'
    int start = msg.lastIndexOf('{');
    if (start < 0)
        return;

    // Print everything else the UNO says (state changes, E-stop replies, parse errors) for debugging
    String extra = msg.substring(0, start);
    extra.trim();
    if (extra.length())
        Serial.println("UNO: " + extra);

    // Events (attached to the next {T,...} row):
    //   {SG,car_x,car_y,sign_x,sign_y,fix_dx,fix_dy,match}  stop sign     {LP,n,ms,mm}  lap done
    //   {CF,leg,car_x,car_y,fix_dx,fix_dy,used}             corner fix
    //   {OB,obst_x,obst_y,side,fix_dx,fix_dy,used}          overtake started
    //   {OV,ms,side,line_found}                             overtake finished
    //   {TL,car_x,car_y,light_x,light_y,fix_dx,fix_dy}      traffic light fix
    static const char *const EVENTS[] = {"{SG,", "{LP,", "{CF,", "{OB,", "{OV,", "{TL,"};
    int ev_i = -1;
    for (int k = 0; k < 6; k++)
        if (msg.startsWith(EVENTS[k], start))
            ev_i = k;
    if (ev_i >= 0) {
        Serial.println("UNO: " + msg.substring(start));
        char ev[112];
        msg.substring(start + 4).toCharArray(ev, sizeof(ev));
        long v[8] = {0};
        char *p = ev;
        for (int i = 0; i < 8 && *p && *p != '}'; i++) {
            v[i] = strtol(p, &p, 10);
            if (*p == ',')
                p++;
        }
        xSemaphoreTake(log_mutex, portMAX_DELAY);
        switch (ev_i) {
        case 0:
            pending.sign = true;
            pending.stop_x_mm = v[0];
            pending.stop_y_mm = v[1];
            pending.sign_x_mm = v[2];
            pending.sign_y_mm = v[3];
            pending.fix_dx_mm = v[4];
            pending.fix_dy_mm = v[5];
            pending.match = v[6];
            last_sign = pending;
            break;
        case 1:
            pending.lap = true;
            pending.lap_n = v[0];
            pending.lap_ms = v[1];
            pending.lap_mm = v[2];
            last_lap = pending;
            break;
        case 2:
            pending.cf = true;
            pending.cf_leg = v[0];
            pending.cf_x = v[1];
            pending.cf_y = v[2];
            pending.cf_dx = v[3];
            pending.cf_dy = v[4];
            pending.cf_used = v[5] != 0;
            break;
        case 3:
            pending.ob = true;
            pending.ob_x = v[0];
            pending.ob_y = v[1];
            pending.ob_side = v[2];
            pending.ob_dx = v[3];
            pending.ob_dy = v[4];
            pending.ob_used = v[5] != 0;
            break;
        case 4:
            pending.ov = true;
            pending.ov_ms = v[0];
            pending.ov_found = v[2] != 0;
            break;
        case 5:
            pending.tl = true;
            pending.tl_cx = v[0];
            pending.tl_cy = v[1];
            pending.tl_x = v[2];
            pending.tl_y = v[3];
            pending.tl_dx = v[4];
            pending.tl_dy = v[5];
            break;
        }
        xSemaphoreGive(log_mutex);
        return;
    }
    if (!msg.startsWith("{T,", start)) {
        Serial.println("UNO: " + msg.substring(start));
        return;
    }

    if (!rows)
        return;
    // Split "{T,time,gyro,pred,x,y,est,xe,ye,lap_ds,lap_cm,laps,sig_mm,scale,state}" into fields.
    // pred can be empty, so no strtok.
    const int NF = 14;
    char buf[160];
    msg.substring(start + 3).toCharArray(buf, sizeof(buf));
    char *f[NF];
    char *p = buf;
    for (int i = 0; i < NF; i++) {
        f[i] = p;
        char *sep = strpbrk(p, i < NF - 1 ? "," : "}");
        if (!sep)
            return; // incomplete message (or an older UNO sketch with fewer fields)
        *sep = 0;
        p = sep + 1;
    }

    LogRow r = {};
    r.t = strtoul(f[0], NULL, 10);
    r.actual = strtol(f[1], NULL, 10);
    r.pred_valid = f[2][0] != 0;
    r.pred = r.pred_valid ? strtol(f[2], NULL, 10) : 0;
    r.x_mm = strtol(f[3], NULL, 10);
    r.y_mm = strtol(f[4], NULL, 10);
    r.est = strtol(f[5], NULL, 10);
    r.xe_mm = strtol(f[6], NULL, 10);
    r.ye_mm = strtol(f[7], NULL, 10);
    r.lap_ds = strtoul(f[8], NULL, 10);
    r.lap_cm = strtoul(f[9], NULL, 10);
    r.laps = strtoul(f[10], NULL, 10);
    r.sig_mm = strtoul(f[11], NULL, 10);
    r.scale = strtoul(f[12], NULL, 10);
    strlcpy(r.state, f[13], sizeof(r.state));

    xSemaphoreTake(log_mutex, portMAX_DELAY);
    // attach any events that arrived since the last row
    if (pending.sign) {
        r.sign = true;
        r.stop_x_mm = pending.stop_x_mm;
        r.stop_y_mm = pending.stop_y_mm;
        r.sign_x_mm = pending.sign_x_mm;
        r.sign_y_mm = pending.sign_y_mm;
        r.fix_dx_mm = pending.fix_dx_mm;
        r.fix_dy_mm = pending.fix_dy_mm;
        r.match = pending.match;
    }
    if (pending.lap) {
        r.lap = true;
        r.lap_n = pending.lap_n;
        r.lap_ms = pending.lap_ms;
        r.lap_mm = pending.lap_mm;
    }
    if (pending.cf) {
        r.cf = true;
        r.cf_leg = pending.cf_leg;
        r.cf_x = pending.cf_x;
        r.cf_y = pending.cf_y;
        r.cf_dx = pending.cf_dx;
        r.cf_dy = pending.cf_dy;
        r.cf_used = pending.cf_used;
    }
    if (pending.ob) {
        r.ob = true;
        r.ob_x = pending.ob_x;
        r.ob_y = pending.ob_y;
        r.ob_side = pending.ob_side;
        r.ob_dx = pending.ob_dx;
        r.ob_dy = pending.ob_dy;
        r.ob_used = pending.ob_used;
    }
    if (pending.ov) {
        r.ov = true;
        r.ov_ms = pending.ov_ms;
        r.ov_found = pending.ov_found;
    }
    if (pending.tl) {
        r.tl = true;
        r.tl_cx = pending.tl_cx;
        r.tl_cy = pending.tl_cy;
        r.tl_x = pending.tl_x;
        r.tl_y = pending.tl_y;
        r.tl_dx = pending.tl_dx;
        r.tl_dy = pending.tl_dy;
    }
    pending.sign = pending.lap = pending.cf = pending.ob = pending.ov = pending.tl = false;
    rows[head] = r;
    head = (head + 1) % LOG_ROWS;
    if (count < LOG_ROWS)
        count++;
    last_rx = millis();
    // Snapshot once per E-stop, and only if there is new data: with 300 s of rows a save takes several
    // seconds, so re-saving every few seconds (as with the 90 s log) would block the ESP32 the whole stop.
    if (strcmp(r.state, "ESTOP") != 0)
        unsaved = true;
    else if (fs_ok && unsaved) {
        unsaved = false;
        save_snapshot();
    }
    xSemaphoreGive(log_mutex);
}

// GET /data -> {"t":12345,"actual":90.3,"pred":88.1,"x":1.234,"y":-0.050,"est":90.0,"xe":1.200,"ye":0.010,
//                "state":"LINE_FOLLOW","age":120,"stale":false,"log_secs":90}
//                "lapt":12.3,"lapd":4.567,"laps":0,
//                "lap":{"n":1,"t":31.2,"d":8.512,"v":0.273} or null,
//                "sign":{"sx":2.100,"sy":-0.200,"dx":0.050,"dy":-0.010,"m":0} or null}
static esp_err_t data_handler(httpd_req_t *req)
{
    char json[680];
    char a[16] = "0.0", pr[16] = "null", xs[16], ys[16], es[16], xes[16], yes[16];
    char lapj[96] = "null", signj[128] = "null";
    LogRow r = {};
    strcpy(r.state, "-");
    LogRow ls, ll;
    if (!rows)
        return httpd_resp_send_500(req);
    bool ok;
    unsigned long age;
    uint16_t n;

    xSemaphoreTake(log_mutex, portMAX_DELAY);
    n = count;
    ok = n > 0 && last_rx != 0;
    if (n > 0)
        r = row_at(n - 1);
    age = millis() - last_rx;
    ls = last_sign;
    ll = last_lap;
    xSemaphoreGive(log_mutex);

    if (ll.lap) {
        char d[16];
        mm_to_str(ll.lap_mm, d, sizeof(d));
        float v = ll.lap_ms ? (float)ll.lap_mm / (float)ll.lap_ms : 0; // mm/ms = m/s
        snprintf(lapj, sizeof(lapj), "{\"n\":%u,\"t\":%.1f,\"d\":%s,\"v\":%.3f}",
                 ll.lap_n, ll.lap_ms / 1000.0f, d, v);
    }
    if (ls.sign) {
        char sx[16], sy[16], dx[16], dy[16];
        mm_to_str(ls.sign_x_mm, sx, sizeof(sx));
        mm_to_str(ls.sign_y_mm, sy, sizeof(sy));
        mm_to_str(ls.fix_dx_mm, dx, sizeof(dx));
        mm_to_str(ls.fix_dy_mm, dy, sizeof(dy));
        snprintf(signj, sizeof(signj), "{\"sx\":%s,\"sy\":%s,\"dx\":%s,\"dy\":%s,\"m\":%d}",
                 sx, sy, dx, dy, ls.match);
    }

    tenths_to_str(r.actual, a, sizeof(a));
    if (r.pred_valid)
        tenths_to_str(r.pred, pr, sizeof(pr));
    mm_to_str(r.x_mm, xs, sizeof(xs));
    mm_to_str(r.y_mm, ys, sizeof(ys));
    tenths_to_str(r.est, es, sizeof(es));
    mm_to_str(r.xe_mm, xes, sizeof(xes));
    mm_to_str(r.ye_mm, yes, sizeof(yes));

    snprintf(json, sizeof(json),
             "{\"t\":%u,\"actual\":%s,\"pred\":%s,\"x\":%s,\"y\":%s,\"est\":%s,\"xe\":%s,\"ye\":%s,"
             "\"state\":\"%s\",\"age\":%lu,\"stale\":%s,\"log_secs\":%u,"
             "\"lapt\":%u.%u,\"lapd\":%u.%02u,\"laps\":%u,\"lap\":%s,\"sign\":%s,"
             "\"sig\":%u.%03u,\"scale\":%u.%03u}",
             r.t, a, pr, xs, ys, es, xes, yes, r.state, age, (!ok || age > DATA_STALE_MS) ? "true" : "false",
             (unsigned)(n / LOG_RATE_HZ),
             r.lap_ds / 10, r.lap_ds % 10, r.lap_cm / 100, r.lap_cm % 100, r.laps, lapj, signj,
             r.sig_mm / 1000, r.sig_mm % 1000, r.scale / 1000, r.scale % 1000);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, json, strlen(json));
}

// GET /log.csv -> the last LOG_SECONDS of data as a file download
static esp_err_t log_download_handler(httpd_req_t *req)
{
    // ~390 KB, so it lives in PSRAM (allocated on the first download) instead of internal RAM
    static const size_t CSV_SIZE = sizeof(LOG_HEADER) + LOG_ROWS * 260; // rows with events are ~230 chars (~390 KB)
    static char *csv = NULL;
    if (!rows)
        return httpd_resp_send_500(req);
    if (!csv)
        csv = (char *)ps_malloc(CSV_SIZE);
    if (!csv)
        return httpd_resp_send_500(req);
    size_t len = strlcpy(csv, LOG_HEADER, CSV_SIZE);

    xSemaphoreTake(log_mutex, portMAX_DELAY);
    for (uint16_t i = 0; i < count && len < CSV_SIZE; i++) {
        const LogRow &r = row_at(i);
        char a[16], pr[16] = "", xs[16], ys[16], es[16], xes[16], yes[16];
        tenths_to_str(r.actual, a, sizeof(a));
        if (r.pred_valid)
            tenths_to_str(r.pred, pr, sizeof(pr));
        mm_to_str(r.x_mm, xs, sizeof(xs));
        mm_to_str(r.y_mm, ys, sizeof(ys));
        tenths_to_str(r.est, es, sizeof(es));
        mm_to_str(r.xe_mm, xes, sizeof(xes));
        mm_to_str(r.ye_mm, yes, sizeof(yes));
        len += snprintf(csv + len, CSV_SIZE - len, "%u,%s,%s,%s,%s,%s,%s,%s,%u.%u,%u.%02u,%u,%s,%u.%03u,%u.%03u,",
                        r.t, a, pr, xs, ys, es, xes, yes,
                        r.lap_ds / 10, r.lap_ds % 10, r.lap_cm / 100, r.lap_cm % 100, r.laps, r.state,
                        r.sig_mm / 1000, r.sig_mm % 1000, r.scale / 1000, r.scale % 1000);
        if (len >= CSV_SIZE)
            break;
        if (r.sign) {
            char s1[16], s2[16], s3[16], s4[16], s5[16], s6[16];
            mm_to_str(r.stop_x_mm, s1, sizeof(s1));
            mm_to_str(r.stop_y_mm, s2, sizeof(s2));
            mm_to_str(r.sign_x_mm, s3, sizeof(s3));
            mm_to_str(r.sign_y_mm, s4, sizeof(s4));
            mm_to_str(r.fix_dx_mm, s5, sizeof(s5));
            mm_to_str(r.fix_dy_mm, s6, sizeof(s6));
            len += snprintf(csv + len, CSV_SIZE - len, "%s,%s,%s,%s,%s,%s,%d,", s1, s2, s3, s4, s5, s6, r.match);
        } else
            len += strlcpy(csv + len, ",,,,,,,", CSV_SIZE - len);
        if (len >= CSV_SIZE)
            break;
        if (r.lap) {
            char d[16];
            mm_to_str(r.lap_mm, d, sizeof(d));
            len += snprintf(csv + len, CSV_SIZE - len, "%u,%.1f,%s,%.3f,", r.lap_n, r.lap_ms / 1000.0f, d,
                            r.lap_ms ? (float)r.lap_mm / (float)r.lap_ms : 0.0f);
        } else
            len += strlcpy(csv + len, ",,,,", CSV_SIZE - len);
        if (len >= CSV_SIZE)
            break;
        char m1[16], m2[16], m3[16], m4[16], m5[16], m6[16];
        if (r.cf) {
            mm_to_str(r.cf_x, m1, sizeof(m1));
            mm_to_str(r.cf_y, m2, sizeof(m2));
            mm_to_str(r.cf_dx, m3, sizeof(m3));
            mm_to_str(r.cf_dy, m4, sizeof(m4));
            len += snprintf(csv + len, CSV_SIZE - len, "%d,%s,%s,%s,%s,%d,", r.cf_leg, m1, m2, m3, m4, r.cf_used);
        } else
            len += strlcpy(csv + len, ",,,,,,", CSV_SIZE - len);
        if (len >= CSV_SIZE)
            break;
        if (r.ob) {
            mm_to_str(r.ob_x, m1, sizeof(m1));
            mm_to_str(r.ob_y, m2, sizeof(m2));
            mm_to_str(r.ob_dx, m3, sizeof(m3));
            mm_to_str(r.ob_dy, m4, sizeof(m4));
            len += snprintf(csv + len, CSV_SIZE - len, "%s,%s,%s,%s,%s,%d,", m1, m2, r.ob_side ? "right" : "left",
                            m3, m4, r.ob_used);
        } else
            len += strlcpy(csv + len, ",,,,,,", CSV_SIZE - len);
        if (len >= CSV_SIZE)
            break;
        if (r.ov)
            len += snprintf(csv + len, CSV_SIZE - len, "%.2f,%d,", r.ov_ms / 1000.0f, r.ov_found);
        else
            len += strlcpy(csv + len, ",,", CSV_SIZE - len);
        if (len >= CSV_SIZE)
            break;
        if (r.tl) {
            mm_to_str(r.tl_cx, m1, sizeof(m1));
            mm_to_str(r.tl_cy, m2, sizeof(m2));
            mm_to_str(r.tl_x, m3, sizeof(m3));
            mm_to_str(r.tl_y, m4, sizeof(m4));
            mm_to_str(r.tl_dx, m5, sizeof(m5));
            mm_to_str(r.tl_dy, m6, sizeof(m6));
            len += snprintf(csv + len, CSV_SIZE - len, "%s,%s,%s,%s,%s,%s\n", m1, m2, m3, m4, m5, m6);
        } else
            len += strlcpy(csv + len, ",,,,,\n", CSV_SIZE - len);
    }
    xSemaphoreGive(log_mutex);
    if (len > CSV_SIZE - 1)
        len = CSV_SIZE - 1;

    httpd_resp_set_type(req, "text/csv");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"robot_log.csv\"");
    return httpd_resp_send(req, csv, len);
}

// GET /log_clear -> delete all logged rows
static esp_err_t log_clear_handler(httpd_req_t *req)
{
    xSemaphoreTake(log_mutex, portMAX_DELAY);
    head = count = 0;
    last_sign = last_lap = pending = LogRow{};
    if (fs_ok)
        SPIFFS.remove(SNAPSHOT_PATH);
    xSemaphoreGive(log_mutex);

    Serial.println("telemetry: log cleared");
    return httpd_resp_send(req, "ok", 2);
}

// GET /estop?on=1 engages the UNO's emergency stop, /estop?on=0 releases it.
// The page shows the result from the UNO's telemetry state ("ESTOP"), not from this reply.
static esp_err_t estop_handler(httpd_req_t *req)
{
    char query[16] = "", on[4] = "";
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "on", on, sizeof(on)) != ESP_OK)
        return httpd_resp_send_404(req);

    bool engage = on[0] == '1';
    Serial2.print(engage ? "{\"N\":300,\"D1\":1}" : "{\"N\":300,\"D1\":0}");
    Serial.println(engage ? "web: E-STOP engage sent" : "web: E-STOP release sent");
    return httpd_resp_send(req, "ok", 2);
}

// GET /pose_reset -> the UNO sets the kinematic model's origin (X=0, Y=0, heading=0) to
// where the car is now, and zeroes the gyro heading. Press it with the car at the start line.
static esp_err_t pose_reset_handler(httpd_req_t *req)
{
    Serial2.print("{\"N\":301}");
    Serial.println("web: pose reset sent");
    return httpd_resp_send(req, "ok", 2);
}

void telemetry_register_handlers(httpd_handle_t server)
{
    httpd_uri_t data_uri = {
        .uri = "/data", .method = HTTP_GET, .handler = data_handler, .user_ctx = NULL};
    httpd_uri_t log_uri = {
        .uri = "/log.csv", .method = HTTP_GET, .handler = log_download_handler, .user_ctx = NULL};
    httpd_uri_t clear_uri = {
        .uri = "/log_clear", .method = HTTP_GET, .handler = log_clear_handler, .user_ctx = NULL};
    httpd_uri_t estop_uri = {
        .uri = "/estop", .method = HTTP_GET, .handler = estop_handler, .user_ctx = NULL};

    httpd_register_uri_handler(server, &data_uri);
    httpd_register_uri_handler(server, &log_uri);
    httpd_register_uri_handler(server, &clear_uri);
    httpd_uri_t pose_uri = {
        .uri = "/pose_reset", .method = HTTP_GET, .handler = pose_reset_handler, .user_ctx = NULL};

    httpd_register_uri_handler(server, &estop_uri);
    httpd_register_uri_handler(server, &pose_uri);
}
