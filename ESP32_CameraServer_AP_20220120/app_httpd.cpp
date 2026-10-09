// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "camera_index.h"
#include "Arduino.h"

#include "edge_impulse.h"

#include "fb_gfx.h"

#include "app_httpd.h"
#include "telemetry.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// Face detection / recognition removed.

#include "fd_forward.h"


#define FACE_COLOR_WHITE  0x00FFFFFF
#define FACE_COLOR_BLACK  0x00000000
#define FACE_COLOR_RED    0x000000FF
#define FACE_COLOR_GREEN  0x0000FF00
#define FACE_COLOR_BLUE   0x00FF0000
#define FACE_COLOR_YELLOW (FACE_COLOR_RED | FACE_COLOR_GREEN)
#define FACE_COLOR_CYAN   (FACE_COLOR_BLUE | FACE_COLOR_GREEN)
#define FACE_COLOR_PURPLE (FACE_COLOR_BLUE | FACE_COLOR_RED)

#define STOP_CHECK_MIN_PX 100   // only run the stop sign model on red blobs at least this big
                                // KEEP THIS EQUAL TO RED_STOP_MIN_PX ON THE UNO
                                // (was 150: glare makes the sign count fewer red pixels, so it
                                //  only reached 150 when the car was already too close)
#define DETECT_DELAY_MS   100   // pause between detection frames (was 500)

#define I_RES        2
#define DIST_THRESH  10
#define MAX_BLOBS    6

// ---- Red colour thresholds (FIX 2) ----
// Red is stricter than green/blue because orange, skin, wood and warm light were
// passing as red. Measure your sign with HSV_PROBE below and adjust these.
// (first try was 12 / 345 / 0.50 / 0.30 - too strict, missed the sign)
#define RED_HUE_LOW_MAX   20     // red if hue <= this ...
#define RED_HUE_HIGH_MIN  340    // ... or hue >= this (red wraps around 0/360)
#define RED_MIN_SAT       0.40f  // raise to reject pinkish / washed-out red, lower if the sign is missed
#define RED_MIN_VAL       0.25f  // raise to reject dark reddish shadows

// 1 = every ~1 s, print the H/S/V of the centre pixel (draws a white cross there
//     on the /stream view). Point the centre of the camera at the sign, then at
//     whatever causes false stops, and set the RED_ thresholds between the two.
// 0 = off (normal running)
#define HSV_PROBE         1      // ON for tuning - set back to 0 when the thresholds are right

// ---- Red blob shape filter (FIX 3) ----
// A real sign is a solid, roughly square blob. Lighting noise and distant objects
// make sparse, stretched blobs that used to add up past STOP_CHECK_MIN_PX.
#define RED_MIN_FILL      0.15f  // red pixels must cover at least this much of the blob's box
                                 // (measured: the real sign reads 0.22; tried 0.40 and 0.30, both rejected it)
#define RED_MIN_ASPECT    0.30f  // allowed box width/height ratio - only rejects thin strips (tape, edges)
#define RED_MAX_ASPECT    3.30f  // (skipped when the box touches the frame edge, i.e. sign is very close)
                                 // first try was 0.60-1.60, but glare makes only part of the sign count as
                                 // red, so it measured ~20x52 (0.38) and was rejected

typedef struct
{
    size_t size;
    size_t index;
    size_t count;
    int    sum;
    int   *values;
} ra_filter_t;

typedef struct
{
    httpd_req_t *req;
    size_t       len;
} jpg_chunking_t;

static SemaphoreHandle_t camera_mutex = NULL;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;

static const char *_STREAM_BOUNDARY = "\r\n";
static const char *_STREAM_PART     = "Len: %u\r\n";

static const char *_STREAM_BOUNDARY_test = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART_test     = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static ra_filter_t ra_filter;
httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

static volatile bool stream_active = false;

static ra_filter_t *ra_filter_init(ra_filter_t *filter, size_t sample_size)
{
    memset(filter, 0, sizeof(ra_filter_t));
    filter->values = (int *)malloc(sample_size * sizeof(int));
    if (!filter->values) {
        return NULL;
    }
    memset(filter->values, 0, sample_size * sizeof(int));
    filter->size = sample_size;
    return filter;
}

static int ra_filter_run(ra_filter_t *filter, int value)
{
    if (!filter->values) {
        return value;
    }
    filter->sum -= filter->values[filter->index];
    filter->values[filter->index] = value;
    filter->sum += filter->values[filter->index];
    filter->index++;
    filter->index = filter->index % filter->size;
    if (filter->count < filter->size) {
        filter->count++;
    }
    return filter->sum / filter->count;
}

// ---- Data logging placeholders ----
static blob_t best_red;
static bool   best_red_valid = false;

#define DIST_A 0.0f
#define DIST_B -1.0f

// ---- HSV conversion ----
hsv_t rgb_to_hsv(float r, float g, float b)
{
    hsv_t hsv = {0, 0, 0};

    r = r / 255.0;
    g = g / 255.0;
    b = b / 255.0;

    // FIX: fd_forward.h (esp-face) #defines max() and min() as macros, which breaks
    // std::max / std::min. fmaxf / fminf are plain float functions, so no clash.
    float cmax = fmaxf(r, fmaxf(g, b));
    float cmin = fminf(r, fminf(g, b));
    float diff = cmax - cmin;
    float h = -1, s = -1;

    if (cmax == cmin)
        h = 0;
    else if (cmax == r)
        h = fmodf(60 * ((g - b) / diff) + 360, 360);
    else if (cmax == g)
        h = fmodf(60 * ((b - r) / diff) + 120, 360);
    else if (cmax == b)
        h = fmodf(60 * ((r - g) / diff) + 240, 360);

    if (cmax == 0)
        s = 0;
    else
        s = (diff / cmax);

    float v = cmax;

    return {h, s, v};
}

static void fresh_blob(blob_t *b, char colour)
{
    b->x_min  = UINT16_MAX;
    b->y_min  = UINT16_MAX;
    b->x_max  = 0;
    b->y_max  = 0;
    b->sum_x  = 0;
    b->sum_y  = 0;
    b->sizePX = 0;
    b->colour = colour;
}

static void fresh_blob_at(blob_t *b, char colour, uint16_t x, uint16_t y)
{
    b->x_min  = x;
    b->x_max  = x;
    b->y_min  = y;
    b->y_max  = y;
    b->sum_x  = x;
    b->sum_y  = y;
    b->sizePX = 1;
    b->colour = colour;
}

void draw_box(dl_matrix3du_t *image_matrix, blob_t box, char colour_choice)
{
    fb_data_t fb;
    fb.width           = image_matrix->w;
    fb.height          = image_matrix->h;
    fb.data            = image_matrix->item;
    fb.bytes_per_pixel = 3;
    fb.format          = FB_BGR888;

    int x = box.x_min;
    int y = box.y_min;
    int w = box.x_max - box.x_min;
    int h = box.y_max - box.y_min;

    uint32_t color = FACE_COLOR_PURPLE;
    if (colour_choice == 'R') color = FACE_COLOR_RED;
    if (colour_choice == 'G') color = FACE_COLOR_GREEN;
    if (colour_choice == 'B') color = FACE_COLOR_BLUE;

    fb_gfx_drawFastHLine(&fb, x, y, w, color);
    fb_gfx_drawFastHLine(&fb, x, y + h, w, color);
    fb_gfx_drawFastVLine(&fb, x, y, h, color);
    fb_gfx_drawFastVLine(&fb, x + w, y, h, color);
}

void draw_centroid(dl_matrix3du_t *image_matrix, int cx, int cy)
{
    fb_data_t fb;
    fb.width           = image_matrix->w;
    fb.height          = image_matrix->h;
    fb.data            = image_matrix->item;
    fb.bytes_per_pixel = 3;
    fb.format          = FB_BGR888;

    fb_gfx_drawFastHLine(&fb, cx - 5, cy, 10, FACE_COLOR_GREEN);
    fb_gfx_drawFastVLine(&fb, cx, cy - 5, 10, FACE_COLOR_GREEN);
}

bool is_red(uint8_t r, uint8_t g, uint8_t b)
{
    int maxc = max(r, max(g, b));
    int minc = min(r, min(g, b));
    int diff = maxc - minc;

    if (diff < 35) return false;
    if (r != maxc) return false;
    if (r - g < 30) return false;
    if (r - b < 30) return false;
    return true;
}

bool is_green(uint8_t r, uint8_t g, uint8_t b)
{
    int maxc = max(r, max(g, b));
    int minc = min(r, min(g, b));
    int diff = maxc - minc;

    if (diff < 35) return false;
    if (g != maxc) return false;
    if (g - r < 30) return false;
    if (g - b < 30) return false;
    return true;
}

bool is_blue(uint8_t r, uint8_t g, uint8_t b)
{
    int maxc = max(r, max(g, b));
    int minc = min(r, min(g, b));
    int diff = maxc - minc;

    if (diff < 50) return false;
    if (b != maxc) return false;
    if (b - r < 30) return false;
    if (b - g < 30) return false;
    return true;
}

char find_colour(uint8_t r, uint8_t g, uint8_t b)
{
    hsv_t c_hsv = rgb_to_hsv(r, g, b);

    if (c_hsv.V < 0.25 || c_hsv.S < 0.35) {
        return '0';
    }

    if (210 <= c_hsv.H && c_hsv.H <= 270) {
        return 'B';
    } else if (90 <= c_hsv.H && c_hsv.H <= 150) {   // <-- was `90 <= H <= 150`
        return 'G';
    } else if ((c_hsv.H <= RED_HUE_LOW_MAX || c_hsv.H >= RED_HUE_HIGH_MIN) &&
               c_hsv.S >= RED_MIN_SAT && c_hsv.V >= RED_MIN_VAL) {
        // FIX 2: was hue <= 30 / >= 330 with S >= 0.35, which let orange and warm light through
        return 'R';
    }
    return '0';
}

blob_t current_obj;

// FIX 3: true if a red blob looks like a sign: solid and roughly square.
// b->sizePX must still be the SAMPLED pixel count (before the * I_RES * I_RES).
static bool red_blob_ok(const blob_t *b, int width, int height)
{
    int w = b->x_max - b->x_min + I_RES;   // box size in full-frame pixels
    int h = b->y_max - b->y_min + I_RES;
    float fill = (float)(b->sizePX * I_RES * I_RES) / (float)(w * h);
    float aspect = (float)w / (float)h;

    // A sign right in front of the camera gets cut off by the frame edge, which
    // changes its shape, so only check the shape when the whole blob is in view.
    bool touches_edge = b->x_min < I_RES || b->y_min < I_RES ||
                        b->x_max >= width - I_RES || b->y_max >= height - I_RES;

    bool fill_ok   = fill >= RED_MIN_FILL;
    bool aspect_ok = touches_edge || (aspect >= RED_MIN_ASPECT && aspect <= RED_MAX_ASPECT);

#if !LOG_CSV
    // Shows WHY a red blob passed or failed, so the limits can be set from real numbers
    Serial.printf("  red blob %dx%d fill=%.2f%s aspect=%.2f%s%s\n", w, h,
                  fill, fill_ok ? "" : "(LOW)",
                  aspect, aspect_ok ? "" : "(BAD)",
                  touches_edge ? " edge" : "");
#endif
    return fill_ok && aspect_ok;
}

#if HSV_PROBE
static void hsv_probe(dl_matrix3du_t *img_m, bool draw)
{
    static unsigned long last = 0;
    int cx = img_m->w / 2, cy = img_m->h / 2;
    uint8_t *p = img_m->item + (cy * img_m->w + cx) * 3;   // BGR order
    if (millis() - last > 1000) {
        last = millis();
        hsv_t c = rgb_to_hsv(p[2], p[1], p[0]);
        Serial.printf("[HSV centre] H=%.0f S=%.2f V=%.2f  -> '%c'\n", c.H, c.S, c.V,
                      find_colour(p[2], p[1], p[0]));
    }
    if (draw) {   // white cross so you can see where the probe is on /stream
        fb_data_t fb;
        fb.width = img_m->w; fb.height = img_m->h; fb.data = img_m->item;
        fb.bytes_per_pixel = 3; fb.format = FB_BGR888;
        fb_gfx_drawFastHLine(&fb, cx - 4, cy, 9, FACE_COLOR_WHITE);
        fb_gfx_drawFastVLine(&fb, cx, cy - 4, 9, FACE_COLOR_WHITE);
    }
}
#endif

// draw = true draws boxes/centroids into the image (for /stream).
// FIX 1: the detection task passes false, so the stop sign model gets a clean image
// (before, the model was shown the sign with rectangles and a crosshair drawn over it).
static blob_t colour_detect(dl_matrix3du_t *img_m, bool draw)
{
    int blob_count = 0;

    fresh_blob(&current_obj, '?');

    blob_t blob[MAX_BLOBS];

    int width  = img_m->w;
    int height = img_m->h;

    for (int i = 0; i < MAX_BLOBS; i++) fresh_blob(&blob[i], '?');

    for (int y = 0; y < height; y = y + I_RES) {
        for (int x = 0; x < width; x = x + I_RES) {
            uint8_t *p = img_m->item + (y * img_m->w + x) * 3;
            uint8_t b = p[0];
            uint8_t g = p[1];
            uint8_t r = p[2];

            //char colour = 0;  

            // if(is_red(r, g, b)) colour = 'R';
            // if(is_blue(r, g, b)) colour = 'B';
            // if(is_green(r, g, b)) colour = 'G'; //old implementation

            char colour = find_colour(r, g, b);
            if (colour == '0') continue;

            int matchedto = -1;
            for (int z = 0; z < blob_count; z++) {
                if (blob[z].colour != colour) continue;
                if (x >= blob[z].x_min - I_RES * DIST_THRESH &&
                    x <= blob[z].x_max + I_RES * DIST_THRESH &&
                    y >= blob[z].y_min - I_RES * DIST_THRESH &&
                    y <= blob[z].y_max + I_RES * DIST_THRESH) {
                    matchedto = z;
                    break;
                }
            }

            if (matchedto < 0) {
                if (blob_count >= MAX_BLOBS) continue;
                matchedto = blob_count++;
                fresh_blob_at(&blob[matchedto], colour, x, y);
                continue;
            }
            //Extend the matched blob
            blob_t *bl = &blob[matchedto]; //b1 is an alias for blob, can write stuff shorthand
            if (x < bl->x_min) bl->x_min = x;
            if (x > bl->x_max) bl->x_max = x;
            if (y < bl->y_min) bl->y_min = y;
            if (y > bl->y_max) bl->y_max = y;
            bl->sum_x += x; //use -> when dealing with pointers, '.' with variables
            bl->sum_y += y;
            bl->sizePX++;
        }
    }

#if !LOG_CSV
    if (blob_count > 0) Serial.println("Starting Frame Analysis");
#endif

    StopSignResult ss;
    peek_stop_sign_result(&ss);


    // FIX 3: pick the largest red blob that passes the shape filter. Only if there is
    // none, fall back to the largest green/blue blob. (Before, the largest blob of ANY
    // colour won, so a big green/blue blob could hide a stop sign from the UNO, and
    // sparse red noise counted as a sign.)
    blob_t pick_red, pick_other;
    fresh_blob(&pick_red, '?');
    fresh_blob(&pick_other, '?');

    for (uint16_t i = 0; i < blob_count; i++) {
        if (blob[i].sizePX < 30) continue;

        blob[i].x_cent = blob[i].sum_x / blob[i].sizePX;
        blob[i].y_cent = blob[i].sum_y / blob[i].sizePX;

        bool rejected = (blob[i].colour == 'R') && !red_blob_ok(&blob[i], width, height);

        blob[i].sizePX = blob[i].sizePX * I_RES * I_RES;

        if (!rejected) {
            blob_t *best = (blob[i].colour == 'R') ? &pick_red : &pick_other;
            if (best->sizePX < blob[i].sizePX)
                *best = blob[i];
        }

#if !LOG_CSV
        Serial.printf("centroid of %d pixel %c object exists at (%d, %d)%s\n",
                      blob[i].sizePX, blob[i].colour, blob[i].x_cent, blob[i].y_cent,
                      rejected ? "  [rejected: not sign-shaped]" : "");
#endif

        if (draw) {
            // rejected red blobs are drawn purple so you can see them on /stream
            draw_box(img_m, blob[i], rejected ? 'X' : blob[i].colour);
            draw_centroid(img_m, blob[i].x_cent, blob[i].y_cent);
        }
    }

#if HSV_PROBE
    hsv_probe(img_m, draw);
#endif

    return (pick_red.sizePX > 0) ? pick_red : pick_other;
}

static uint8_t log_csv_row(void)
{
    StopSignResult r;
    peek_stop_sign_result(&r);

    uint16_t red_w = 0, red_h = 0;
    long red_px = 0;
    int dist_cm = -1;

    if (best_red_valid) {
        red_w   = best_red.x_max - best_red.x_min + I_RES;
        red_h   = best_red.y_max - best_red.y_min + I_RES;
        red_px  = best_red.sizePX;
        dist_cm = dist_class(red_px);
    }

    Serial.printf("%lu,%d,%d,%u,%u,%u,%u,%u,%ld,%d,%u\r\n",
                  (unsigned long)millis(), r.found ? 1 : 0, r.conf,
                  r.fx, r.fy, r.w, red_w, red_h, red_px, dist_cm, r.infer_ms);

    return dist_cm;
}

static void object_detect_task(void *arg)
{
    bool ok = false;
    bool stopsign_possible = false;
    Serial.println("Starting task");
    Serial.printf("object_detect_task: camera_mutex = %p\n", camera_mutex);

    while (1) {
        ok = false;
        stopsign_possible = false;
        if (xSemaphoreTake(camera_mutex, portMAX_DELAY) == pdTRUE) {
            // FIX (latency): with fb_count = 2 the camera keeps a frame waiting in its
            // buffer, so the first frame we get can be ~0.5 s old. Throw it away and
            // take the next one, so we react to where the sign is NOW.
            camera_fb_t *fb = esp_camera_fb_get();
            if (fb) {
                esp_camera_fb_return(fb);
                fb = esp_camera_fb_get();
            }
            if (fb) {
                dl_matrix3du_t *im = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
                if (im) {
                    if (fmt2rgb888(fb->buf, fb->len, fb->format, im->item)) {
                        // FIX 1: draw = false, so im->item stays clean for classify_rgb888() below
                        blob_t the_strongest = colour_detect(im, false);
                        bool has_blob = (the_strongest.sizePX > 0);
                        int dist2obj = has_blob ? dist_class(the_strongest.sizePX) : -1;

                        // CHANGED: send what we see RIGHT AWAY, before running the slow
                        // (~1.2 s) stop sign model, so the UNO can stop on red immediately.
                        // This message always has "D1":"N" (not checked yet).
                        if (has_blob) {
                            SendObjectToUno(the_strongest, dist2obj);
                        }

                        // Only run the model on red blobs big enough to be a nearby sign.
                        if (the_strongest.colour == 'R' && the_strongest.sizePX >= STOP_CHECK_MIN_PX) {
                            stopsign_possible = true;
                        }

                        if (stopsign_possible) {
#if !LOG_CSV
                            Serial.println("Classifying");
#endif
                            ok = classify_rgb888(im->item, fb->width, fb->height);

                            // CHANGED: send a second message with the model's answer:
                            // "D1":"S" if it's a stop sign, "D1":"N" if not.
                            if (ok) {
                                SendObjectToUno(the_strongest, dist2obj);
                            }
                        }

#if LOG_CSV
                        if (ok) log_csv_row();
#endif
                    }
                    dl_matrix3du_free(im);
                }
                esp_camera_fb_return(fb);
            }
            xSemaphoreGive(camera_mutex);
        }
        // FIX (latency): was 500 ms between every frame. The model already takes ~1.2 s
        // when it runs, so only a short pause is needed to let the web server have the camera.
        vTaskDelay(pdMS_TO_TICKS(DETECT_DELAY_MS));
    }
}

static size_t jpg_encode_stream(void *arg, size_t index, const void *data, size_t len)
{
    jpg_chunking_t *j = (jpg_chunking_t *)arg;
    if (!index) {
        j->len = 0;
    }
    if (httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK) {
        return 0;
    }
    j->len += len;
    return len;
}

// ---- /capture : single JPEG, no face detection, no big allocations ----
static esp_err_t capture_handler(httpd_req_t *req)
{
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    int64_t fr_start = esp_timer_get_time();

    stream_active = true;

    if (xSemaphoreTake(camera_mutex, portMAX_DELAY) == pdTRUE) {
        fb = esp_camera_fb_get();
        xSemaphoreGive(camera_mutex);
    }

    if (!fb) {
        Serial.println("Camera capture failed");
        httpd_resp_send_500(req);
        stream_active = false;
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    size_t fb_len = 0;
    if (fb->format == PIXFORMAT_JPEG) {
        fb_len = fb->len;
        res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    } else {
        jpg_chunking_t jchunk = {req, 0};
        res = frame2jpg_cb(fb, 80, jpg_encode_stream, &jchunk) ? ESP_OK : ESP_FAIL;
        httpd_resp_send_chunk(req, NULL, 0);
        fb_len = jchunk.len;
    }

    esp_camera_fb_return(fb);
    stream_active = false;

    int64_t fr_end = esp_timer_get_time();
    Serial.printf("JPG: %uB %ums\n",
                  (uint32_t)fb_len,
                  (uint32_t)((fr_end - fr_start) / 1000));
    return res;
}


static esp_err_t stream_handler(httpd_req_t *req)
{
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    uint8_t *_jpg_buf = NULL;
    char part_buf[64];
    dl_matrix3du_t *image_matrix = NULL;
    int64_t fr_start = 0;

    static int64_t last_frame = 0;
    if (!last_frame) {
        last_frame = esp_timer_get_time();
    }

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK) {
        return res;
    }
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    stream_active = true;

    while (true) {
        if (xSemaphoreTake(camera_mutex, portMAX_DELAY) != pdTRUE) {
            res = ESP_FAIL;
            break;
        }

        fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("Camera capture failed");
            xSemaphoreGive(camera_mutex);
            res = ESP_FAIL;
            break;
        }

        fr_start = esp_timer_get_time();

        if (fb->width > 400) {
            // Large frames: passthrough JPEG, no processing
            if (fb->format != PIXFORMAT_JPEG) {
                bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
                esp_camera_fb_return(fb);
                fb = NULL;
                if (!jpeg_converted) {
                    Serial.println("JPEG compression failed");
                    xSemaphoreGive(camera_mutex);
                    res = ESP_FAIL;
                    break;
                }
            } else {
                _jpg_buf_len = fb->len;
                _jpg_buf     = fb->buf;
            }
        } else {
            if (!image_matrix ||
                image_matrix->w != fb->width ||
                image_matrix->h != fb->height) {
                if (image_matrix) {
                    dl_matrix3du_free(image_matrix);
                    image_matrix = NULL;
                }
                image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
            }

            if (!image_matrix) {
                Serial.println("dl_matrix3du_alloc failed");
                esp_camera_fb_return(fb);
                fb = NULL;
                xSemaphoreGive(camera_mutex);
                res = ESP_FAIL;
                break;
            }

            if (!fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item)) {
                Serial.println("fmt2rgb888 failed");
                esp_camera_fb_return(fb);
                fb = NULL;
                xSemaphoreGive(camera_mutex);
                res = ESP_FAIL;
                break;
            }

            esp_camera_fb_return(fb);
            fb = NULL;

            colour_detect(image_matrix, true);   // /stream view: draw the boxes

            if (!fmt2jpg(image_matrix->item,
                         image_matrix->w * image_matrix->h * 3,
                         image_matrix->w, image_matrix->h,
                         PIXFORMAT_RGB888, 90,
                         &_jpg_buf, &_jpg_buf_len)) {
                Serial.println("fmt2jpg failed");
                xSemaphoreGive(camera_mutex);
                res = ESP_FAIL;
                break;
            }
        }

        xSemaphoreGive(camera_mutex);

        if (res == ESP_OK) {
            size_t hlen = snprintf(part_buf, sizeof(part_buf),
                                   _STREAM_PART_test, _jpg_buf_len);
            res = httpd_resp_send_chunk(req, part_buf, hlen);
        }
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
        }
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY_test,
                                        strlen(_STREAM_BOUNDARY_test));
        }

        if (fb) {
            esp_camera_fb_return(fb);
            fb = NULL;
            _jpg_buf = NULL;
        } else if (_jpg_buf) {
            free(_jpg_buf);
            _jpg_buf = NULL;
        }

        if (res != ESP_OK) {
            break;
        }
    }

    if (image_matrix) {
        dl_matrix3du_free(image_matrix);
        image_matrix = NULL;
    }

    stream_active = false;
    last_frame = 0;
    return res;
}

static esp_err_t cmd_handler(httpd_req_t *req)
{
    char *buf;
    size_t buf_len;
    char variable[32] = {0};
    char value[32]    = {0};

    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = (char *)malloc(buf_len);
        if (!buf) {
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            if (httpd_query_key_value(buf, "var", variable, sizeof(variable)) == ESP_OK &&
                httpd_query_key_value(buf, "val", value, sizeof(value)) == ESP_OK) {
                // ok
            } else {
                free(buf);
                httpd_resp_send_404(req);
                return ESP_FAIL;
            }
        } else {
            free(buf);
            httpd_resp_send_404(req);
            return ESP_FAIL;
        }
        free(buf);
    } else {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    int val = atoi(value);
    Serial.println(val);
    Serial.println(variable);
    sensor_t *s = esp_camera_sensor_get();
    int res = 0;

    if (!strcmp(variable, "framesize")) {
        if (s->pixformat == PIXFORMAT_JPEG)
            res = s->set_framesize(s, (framesize_t)val);
    }
    else if (!strcmp(variable, "quality"))        res = s->set_quality(s, val);
    else if (!strcmp(variable, "contrast"))       res = s->set_contrast(s, val);
    else if (!strcmp(variable, "brightness"))     res = s->set_brightness(s, val);
    else if (!strcmp(variable, "saturation"))     res = s->set_saturation(s, val);
    else if (!strcmp(variable, "gainceiling"))    res = s->set_gainceiling(s, (gainceiling_t)val);
    else if (!strcmp(variable, "colorbar"))       res = s->set_colorbar(s, val);
    else if (!strcmp(variable, "awb"))            res = s->set_whitebal(s, val);
    else if (!strcmp(variable, "agc"))            res = s->set_gain_ctrl(s, val);
    else if (!strcmp(variable, "aec"))            res = s->set_exposure_ctrl(s, val);
    else if (!strcmp(variable, "hmirror"))        res = s->set_hmirror(s, val);
    else if (!strcmp(variable, "vflip"))          res = s->set_vflip(s, val);
    else if (!strcmp(variable, "awb_gain"))       res = s->set_awb_gain(s, val);
    else if (!strcmp(variable, "agc_gain"))       res = s->set_agc_gain(s, val);
    else if (!strcmp(variable, "aec_value"))      res = s->set_aec_value(s, val);
    else if (!strcmp(variable, "aec2"))           res = s->set_aec2(s, val);
    else if (!strcmp(variable, "dcw"))            res = s->set_dcw(s, val);
    else if (!strcmp(variable, "bpc"))            res = s->set_bpc(s, val);
    else if (!strcmp(variable, "wpc"))            res = s->set_wpc(s, val);
    else if (!strcmp(variable, "raw_gma"))        res = s->set_raw_gma(s, val);
    else if (!strcmp(variable, "lenc"))           res = s->set_lenc(s, val);
    else if (!strcmp(variable, "special_effect")) res = s->set_special_effect(s, val);
    else if (!strcmp(variable, "wb_mode"))        res = s->set_wb_mode(s, val);
    else if (!strcmp(variable, "ae_level"))       res = s->set_ae_level(s, val);
    else {
        res = -1;
    }

    if (res) {
        return httpd_resp_send_500(req);
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t status_handler(httpd_req_t *req)
{
    static char json_response[1024];

    sensor_t *s = esp_camera_sensor_get();
    char *p = json_response;
    *p++ = '{';

    p += sprintf(p, "\"framesize\":%u,",      s->status.framesize);
    p += sprintf(p, "\"quality\":%u,",        s->status.quality);
    p += sprintf(p, "\"brightness\":%d,",     s->status.brightness);
    p += sprintf(p, "\"contrast\":%d,",       s->status.contrast);
    p += sprintf(p, "\"saturation\":%d,",     s->status.saturation);
    p += sprintf(p, "\"sharpness\":%d,",      s->status.sharpness);
    p += sprintf(p, "\"special_effect\":%u,", s->status.special_effect);
    p += sprintf(p, "\"wb_mode\":%u,",        s->status.wb_mode);
    p += sprintf(p, "\"awb\":%u,",            s->status.awb);
    p += sprintf(p, "\"awb_gain\":%u,",       s->status.awb_gain);
    p += sprintf(p, "\"aec\":%u,",            s->status.aec);
    p += sprintf(p, "\"aec2\":%u,",           s->status.aec2);
    p += sprintf(p, "\"ae_level\":%d,",       s->status.ae_level);
    p += sprintf(p, "\"aec_value\":%u,",      s->status.aec_value);
    p += sprintf(p, "\"agc\":%u,",            s->status.agc);
    p += sprintf(p, "\"agc_gain\":%u,",       s->status.agc_gain);
    p += sprintf(p, "\"gainceiling\":%u,",    s->status.gainceiling);
    p += sprintf(p, "\"bpc\":%u,",            s->status.bpc);
    p += sprintf(p, "\"wpc\":%u,",            s->status.wpc);
    p += sprintf(p, "\"raw_gma\":%u,",        s->status.raw_gma);
    p += sprintf(p, "\"lenc\":%u,",           s->status.lenc);
    p += sprintf(p, "\"vflip\":%u,",          s->status.vflip);
    p += sprintf(p, "\"hmirror\":%u,",        s->status.hmirror);
    p += sprintf(p, "\"dcw\":%u,",            s->status.dcw);
    p += sprintf(p, "\"colorbar\":%u",        s->status.colorbar);
    *p++ = '}';
    *p++ = 0;

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json_response, strlen(json_response));
}

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    sensor_t *s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID) {
        return httpd_resp_send(req, (const char *)index_ov3660_html_gz,
                               index_ov3660_html_gz_len);
    }
    return httpd_resp_send(req, (const char *)index_ov2640_html_gz,
                           index_ov2640_html_gz_len);
}

static esp_err_t Test1_handler(httpd_req_t *req)
{
    Serial.println("Test1_handler...");
    char *buf;
    size_t buf_len;
    char variable[32] = {0};

    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = (char *)malloc(buf_len);
        if (!buf) {
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            if (httpd_query_key_value(buf, "var", variable, sizeof(variable)) == ESP_OK) {
                // ok
            } else {
                free(buf);
                httpd_resp_send_404(req);
                return ESP_FAIL;
            }
        } else {
            free(buf);
            httpd_resp_send_404(req);
            return ESP_FAIL;
        }
        free(buf);
    } else {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t Test2_handler(httpd_req_t *req)
{
    Serial.println("Test2_handler...");
    httpd_resp_send(req, (const char *)"index", 5);
    return ESP_OK;
}

void startCameraServer()
{
    // Create the mutex FIRST, before any HTTP server or task uses it.
    camera_mutex = xSemaphoreCreateMutex();
    if (camera_mutex == NULL) {
        Serial.println("BAD!!!: could not create camera_mutex");
        return;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 16; // default is 8, the data logger adds 4 more

    httpd_uri_t index_uri = {
        .uri = "/", .method = HTTP_GET, .handler = index_handler, .user_ctx = NULL};

    httpd_uri_t status_uri = {
        .uri = "/status", .method = HTTP_GET, .handler = status_handler, .user_ctx = NULL};

    httpd_uri_t cmd_uri = {
        .uri = "/control", .method = HTTP_GET, .handler = cmd_handler, .user_ctx = NULL};

    httpd_uri_t capture_uri = {
        .uri = "/capture", .method = HTTP_GET, .handler = capture_handler, .user_ctx = NULL};

    httpd_uri_t stream_uri = {
        .uri = "/stream", .method = HTTP_GET, .handler = stream_handler, .user_ctx = NULL};

    httpd_uri_t Test_uri = {
        .uri = "/Test", .method = HTTP_GET, .handler = stream_handler, .user_ctx = NULL};

    httpd_uri_t Test1_uri = {
        .uri = "/test1", .method = HTTP_GET, .handler = Test1_handler, .user_ctx = NULL};

    httpd_uri_t Test2_uri = {
        .uri = "/test2", .method = HTTP_GET, .handler = Test2_handler, .user_ctx = NULL};

    ra_filter_init(&ra_filter, 20);

    Serial.printf("Starting web server on port: '%d'\n", config.server_port);
    if (httpd_start(&camera_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(camera_httpd, &index_uri);
        httpd_register_uri_handler(camera_httpd, &cmd_uri);
        httpd_register_uri_handler(camera_httpd, &status_uri);
        httpd_register_uri_handler(camera_httpd, &capture_uri);
        httpd_register_uri_handler(camera_httpd, &Test_uri);
        httpd_register_uri_handler(camera_httpd, &Test1_uri);
        httpd_register_uri_handler(camera_httpd, &Test2_uri);
        telemetry_register_handlers(camera_httpd); // /data, /log.csv, /log_clear, /estop
    }

    config.server_port += 1;
    config.ctrl_port += 1;
    Serial.printf("Starting stream server on port: '%d'\n", config.server_port);
    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &stream_uri);
    }

    xTaskCreatePinnedToCore(object_detect_task, "colour", 16384, NULL, 1, NULL, 0);
}