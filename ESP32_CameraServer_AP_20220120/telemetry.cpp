#include "telemetry.h"
#include "FS.h"
#include "SPIFFS.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define LOG_SECONDS     30   // keep only the newest 30 s of data, the oldest rows are dropped
#define LOG_RATE_HZ     5    // the UNO sends 5 rows per second
#define LOG_ROWS        (LOG_SECONDS * LOG_RATE_HZ)
#define LOG_HEADER      "time_ms,heading_actual,heading_predicted,state\n"
#define SNAPSHOT_PATH   "/log.bin"
#define SNAPSHOT_MS     5000 // save the rows to flash this often, so they survive an ESP32 reset
#define DATA_STALE_MS   2000 // page shows "no data" if nothing arrived for this long

struct LogRow {
    uint32_t t;      // UNO millis()
    int32_t actual;  // tenths of a degree
    int32_t pred;    // tenths of a degree
    bool pred_valid; // false until the kinematic model sends a value
    char state[13];
};

// Ring buffer of the newest LOG_ROWS rows. Written by loop(), read by the web server task.
static SemaphoreHandle_t log_mutex = NULL;
static LogRow rows[LOG_ROWS];
static uint16_t head = 0;  // slot the next row goes in
static uint16_t count = 0; // rows stored
static bool fs_ok = false;
static unsigned long last_snapshot = 0;
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
    f.write((const uint8_t *)rows, sizeof(rows));
    f.close();
}

// Loads the rows saved before the last reset, if any
static void load_snapshot(void)
{
    File f = SPIFFS.open(SNAPSHOT_PATH, FILE_READ);
    if (!f)
        return;
    if (f.size() == sizeof(head) + sizeof(count) + sizeof(rows)) {
        f.read((uint8_t *)&head, sizeof(head));
        f.read((uint8_t *)&count, sizeof(count));
        f.read((uint8_t *)rows, sizeof(rows));
        if (head >= LOG_ROWS || count > LOG_ROWS)
            head = count = 0;
    }
    f.close();
}

void telemetry_init(void)
{
    log_mutex = xSemaphoreCreateMutex();
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
    if (!msg.startsWith("{T,", start)) {
        Serial.println("UNO: " + msg.substring(start));
        return;
    }

    // Split "{T,time,actual,pred,state}" into fields. pred can be empty, so no strtok.
    char buf[64];
    msg.substring(start + 3).toCharArray(buf, sizeof(buf));
    char *f[4];
    char *p = buf;
    for (int i = 0; i < 4; i++) {
        f[i] = p;
        char *sep = strpbrk(p, i < 3 ? "," : "}");
        if (!sep)
            return; // incomplete message
        *sep = 0;
        p = sep + 1;
    }

    LogRow r;
    r.t = strtoul(f[0], NULL, 10);
    r.actual = strtol(f[1], NULL, 10);
    r.pred_valid = f[2][0] != 0;
    r.pred = r.pred_valid ? strtol(f[2], NULL, 10) : 0;
    strlcpy(r.state, f[3], sizeof(r.state));

    xSemaphoreTake(log_mutex, portMAX_DELAY);
    rows[head] = r;
    head = (head + 1) % LOG_ROWS;
    if (count < LOG_ROWS)
        count++;
    last_rx = millis();
    if (fs_ok && millis() - last_snapshot > SNAPSHOT_MS) {
        last_snapshot = millis();
        save_snapshot();
    }
    xSemaphoreGive(log_mutex);
}

// GET /data -> {"t":12345,"actual":90.3,"pred":null,"state":"LINE_FOLLOW","age":120,"stale":false,"log_secs":30}
static esp_err_t data_handler(httpd_req_t *req)
{
    char json[200];
    char a[16] = "0.0", pr[16] = "null";
    LogRow r = {0, 0, 0, false, "-"};
    bool ok;
    unsigned long age;
    uint16_t n;

    xSemaphoreTake(log_mutex, portMAX_DELAY);
    n = count;
    ok = n > 0 && last_rx != 0;
    if (n > 0)
        r = row_at(n - 1);
    age = millis() - last_rx;
    xSemaphoreGive(log_mutex);

    tenths_to_str(r.actual, a, sizeof(a));
    if (r.pred_valid)
        tenths_to_str(r.pred, pr, sizeof(pr));

    snprintf(json, sizeof(json),
             "{\"t\":%u,\"actual\":%s,\"pred\":%s,\"state\":\"%s\",\"age\":%lu,\"stale\":%s,\"log_secs\":%u}",
             r.t, a, pr, r.state, age, (!ok || age > DATA_STALE_MS) ? "true" : "false",
             (unsigned)(n / LOG_RATE_HZ));

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, json, strlen(json));
}

// GET /log.csv -> the last LOG_SECONDS of data as a file download
static esp_err_t log_download_handler(httpd_req_t *req)
{
    static char csv[sizeof(LOG_HEADER) + LOG_ROWS * 48];
    size_t len = strlcpy(csv, LOG_HEADER, sizeof(csv));

    xSemaphoreTake(log_mutex, portMAX_DELAY);
    for (uint16_t i = 0; i < count && len < sizeof(csv); i++) {
        const LogRow &r = row_at(i);
        char a[16], pr[16] = "";
        tenths_to_str(r.actual, a, sizeof(a));
        if (r.pred_valid)
            tenths_to_str(r.pred, pr, sizeof(pr));
        len += snprintf(csv + len, sizeof(csv) - len, "%u,%s,%s,%s\n", r.t, a, pr, r.state);
    }
    xSemaphoreGive(log_mutex);
    if (len > sizeof(csv) - 1)
        len = sizeof(csv) - 1;

    httpd_resp_set_type(req, "text/csv");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"robot_log.csv\"");
    return httpd_resp_send(req, csv, len);
}

// GET /log_clear -> delete all logged rows
static esp_err_t log_clear_handler(httpd_req_t *req)
{
    xSemaphoreTake(log_mutex, portMAX_DELAY);
    head = count = 0;
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
    httpd_register_uri_handler(server, &estop_uri);
}
