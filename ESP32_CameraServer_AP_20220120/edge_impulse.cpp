/* Edge Impulse Arduino examples
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// These sketches are tested with 2.0.4 ESP32 Arduino Core
// https://github.com/espressif/arduino-esp32/releases/tag/2.0.4

//code taken from edgeimpulse example folder then modified

// Modified by Adam Bateman and Corey White


#include "esp_camera.h"
#include "edge_impulse.h"
#include "CameraWebServer_AP.h"
#include "camera_pins.h"


#include "Corey_317-project-1_inferencing.h"
#include <edge-impulse-sdk/dsp/image/image.hpp>





static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
static bool is_initialised = false;
uint8_t *snapshot_buf; //points to the output of the capture

// KEY -> needed to redifine EI functions to use PSRAM, otherwise we get memory issues

void *ei_malloc(size_t n)           { void *p = ps_malloc(n);    return p ? p : malloc(n); }
void *ei_calloc(size_t n, size_t s) { void *p = ps_calloc(n, s); return p ? p : calloc(n, s); }
void  ei_free(void *p)              { free(p); }


// ---------------- Stop sign result hand-off ----------------
// Detection runs in object_detect_task (core 0); Serial2 is written from
// loop() (core 1). The result is stored here under a spinlock so the two
// tasks never write Serial2 at the same time.

#define STOP_LABEL      "Stop sign"   // must match your Edge Impulse label exactly
#define STOP_MIN_CONF   0.60f    // ignore weaker detections

static portMUX_TYPE     stop_mux = portMUX_INITIALIZER_UNLOCKED;
static StopSignResult   latest_stop = {false, 0, 0, 0, 0, 0, 0, 0, 0};
static bool             stop_result_new = false;

int dist_class(uint32_t size)
{
    int dist = -1;
    if(size > 1500){
        return 5;
    }
    else if(size > 800){
        return 10;
    }
    else if(size > 600){
        return 15;
    }
    else if(size > 500){
        return 20;
    }
    else if(size > 275){
        return 25;
    }
    else{
        return -1;
    }

    return dist;

}

void SendObjectToUno(blob_t input, int distance) //in the future, change this to send the largest object in the frame to the uno the largest option can be assumed to be the closest
//figure out how to import colour of object
{
  StopSignResult r;
  get_stop_sign_result(&r);
//   if (!get_stop_sign_result(&r))
//     return;

  char msg[128]; //might need to update this later to accomidate memory
  snprintf(msg, sizeof(msg),
           "{\"N\":200,\"D1\":%c,\"D2\":%u,\"D3\":%u,\"D4\":%u,\"D5\":%u,\"D6\":%c,\"D7\":%u,\"D8\":%u}", //CHECK THIS TO MAKE SURE DATA IS THE SAME
           r.found ? 'S' : 'N', input.x_cent, input.y_cent, (input.x_max-input.x_min), (input.y_max-input.y_min), input.colour, input.sizePX, distance); //changed stop sign result to send a "type" char instead. 
           // dictionary: 'S' = stop sign, 'O' = obstacle, 'C' = car, 'N' = none. 
  Serial2.print(msg);
  Serial.print(msg);

#if !LOG_CSV
  if (r.found)
    Serial.printf("[STOP] cx=%u cy=%u w=%u h=%u conf=%u%%\n", r.cx, r.cy, r.w, r.h, r.conf);
#endif
}


static uint8_t pct(uint32_t v, uint32_t full) {
    uint32_t p = (v * 100) / full;
    return p > 100 ? 100 : (uint8_t)p;
}

// The model input is a centre crop of the camera frame, scaled to the model
// size. This maps a point in model coordinates back to camera-frame pixels,
// so it can be matched with the colour blobs (which use frame pixels).
static void model_to_frame(float mx, float my, uint16_t *fx, uint16_t *fy)
{
    const float sw = EI_CAMERA_RAW_FRAME_BUFFER_COLS, sh = EI_CAMERA_RAW_FRAME_BUFFER_ROWS;
    const float mw = EI_CLASSIFIER_INPUT_WIDTH,       mh = EI_CLASSIFIER_INPUT_HEIGHT;
    float crop_w = sw, crop_h = sh;
    if (sw / sh > mw / mh) crop_w = sh * mw / mh;   // frame wider than model: crop sides
    else                   crop_h = sw * mh / mw;   // frame taller than model: crop top/bottom
    *fx = (uint16_t)((sw - crop_w) / 2 + mx * crop_w / mw);
    *fy = (uint16_t)((sh - crop_h) / 2 + my * crop_h / mh);
}

static void publish_stop_result(const ei_impulse_result_t &result)
{
    StopSignResult r = {false, 0, 0, 0, 0, 0, 0, 0, 0};
    r.infer_ms = result.timing.dsp + result.timing.classification;

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    uint32_t best_area = 0;
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        const ei_impulse_result_bounding_box_t &bb = result.bounding_boxes[i];
        if (bb.value < STOP_MIN_CONF) continue;
        if (strcmp(bb.label, STOP_LABEL) != 0) continue;

        uint32_t area = bb.width * bb.height;
        if (area > best_area) {            // keep the biggest (closest) sign
            best_area = area;
            r.found = true;
            r.cx   = pct(bb.x + bb.width / 2,  EI_CLASSIFIER_INPUT_WIDTH);
            r.cy   = pct(bb.y + bb.height / 2, EI_CLASSIFIER_INPUT_HEIGHT);
            r.w    = pct(bb.width,  EI_CLASSIFIER_INPUT_WIDTH);
            r.h    = pct(bb.height, EI_CLASSIFIER_INPUT_HEIGHT);
            r.conf = (uint8_t)(bb.value * 100);
            model_to_frame(bb.x + bb.width / 2.0f, bb.y + bb.height / 2.0f, &r.fx, &r.fy);
        }
    }
#else
    #warning "Model is classification-only: stop sign position/size will not be available"
#endif

    portENTER_CRITICAL(&stop_mux);
    latest_stop = r;
    stop_result_new = true;
    portEXIT_CRITICAL(&stop_mux);
}

bool get_stop_sign_result(StopSignResult *out)
{
    portENTER_CRITICAL(&stop_mux);
    bool fresh = stop_result_new;
    *out = latest_stop;
    stop_result_new = false;
    portEXIT_CRITICAL(&stop_mux);
    return fresh;
}

void peek_stop_sign_result(StopSignResult *out)
{
    portENTER_CRITICAL(&stop_mux);
    *out = latest_stop;
    portEXIT_CRITICAL(&stop_mux);
}


//function already uses existing rgb888 data

bool classify_rgb888(const uint8_t *rgb, uint32_t src_w, uint32_t src_h)
{
    if (!is_initialised && !ei_camera_init()) {
        return false;
    }

    // Allocate enough for raw EI size and model input size
    size_t raw_pixels = src_w * src_h;
    size_t model_pixels = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    size_t max_pixels = raw_pixels > model_pixels ? raw_pixels : model_pixels;

    uint8_t *buf = (uint8_t*)ps_malloc(max_pixels * EI_CAMERA_FRAME_BYTE_SIZE);
    if (!buf) {
        ei_printf("ERR: Failed to allocate inference buffer\n");
        return false;
    }

    // Copy or resize from app_http frame to EI raw size -> might have to remove this later
    if (src_w == EI_CAMERA_RAW_FRAME_BUFFER_COLS &&
        src_h == EI_CAMERA_RAW_FRAME_BUFFER_ROWS) {
        memcpy(buf, rgb, raw_pixels * EI_CAMERA_FRAME_BYTE_SIZE);
    } else {
        ei::image::processing::crop_and_interpolate_rgb888(
            (uint8_t*)rgb, src_w, src_h,
            buf, EI_CAMERA_RAW_FRAME_BUFFER_COLS, EI_CAMERA_RAW_FRAME_BUFFER_ROWS);
    }

    // If the model expects a different size, resize in-place
    if (EI_CLASSIFIER_INPUT_WIDTH != EI_CAMERA_RAW_FRAME_BUFFER_COLS ||
        EI_CLASSIFIER_INPUT_HEIGHT != EI_CAMERA_RAW_FRAME_BUFFER_ROWS) {
        ei::image::processing::crop_and_interpolate_rgb888(
            buf, EI_CAMERA_RAW_FRAME_BUFFER_COLS, EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
            buf, EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT);
    }

    // Point the existing get_data callback at our buffer
    uint8_t *old_snapshot = snapshot_buf;
    snapshot_buf = buf;

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);

    // Restore old pointer and free temporary buffer
    snapshot_buf = old_snapshot;
    free(buf);

    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        return false;
    }

    // Hand the result to loop(), which forwards it to the UNO
    publish_stop_result(result);

#if !LOG_CSV
    ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
                result.timing.dsp, result.timing.classification, result.timing.anomaly);

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    ei_printf("Object detection bounding boxes:\r\n");
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        if (bb.value == 0) {
            continue;
        }
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                bb.label,
                bb.value,
                bb.x,
                bb.y,
                bb.width,
                bb.height);
    }

    // Print the prediction results (classification)
#else
    ei_printf("Predictions:\r\n");
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        ei_printf("  %s: ", ei_classifier_inferencing_categories[i]);
        ei_printf("%.5f\r\n", result.classification[i].value);
    }
#endif

    // Print anomaly result (if it exists)
#if EI_CLASSIFIER_HAS_ANOMALY
    ei_printf("Anomaly prediction: %.3f\r\n", result.anomaly);
#endif

#if EI_CLASSIFIER_HAS_VISUAL_ANOMALY
    ei_printf("Visual anomalies:\r\n");
    for (uint32_t i = 0; i < result.visual_ad_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.visual_ad_grid_cells[i];
        if (bb.value == 0) {
            continue;
        }
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n", //can get bounding box
                bb.label,
                bb.value,
                bb.x,
                bb.y,
                bb.width,
                bb.height);
    }
#endif

#endif

    return true;
}














void find_stop()
{

    // instead of wait_ms, we'll wait on the signal, this allows threads to cancel us...
    
    
    if (ei_sleep(5) != EI_IMPULSE_OK) {
        return;
    }

    start_edging();


    Serial.printf("Free heap before alloc: %u bytes\n", ESP.getFreeHeap());    
    snapshot_buf = (uint8_t*)ps_malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);
    Serial.printf("Free heap after alloc: %u bytes\n", ESP.getFreeHeap());
    // check if allocation was successful
    if(snapshot_buf == nullptr) {
        ei_printf("ERR: Failed to allocate snapshot buffer!\n");
        return;
    }



    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false) {
        ei_printf("Failed to capture image\r\n");
        free(snapshot_buf);
        return;
    }

    // Run the classifier
    ei_impulse_result_t result = { 0 };

    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        return;
    }

    // print the predictions
    ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
                result.timing.dsp, result.timing.classification, result.timing.anomaly);

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    ei_printf("Object detection bounding boxes:\r\n");
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        if (bb.value == 0) {
            continue;
        }
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                bb.label,
                bb.value,
                bb.x,
                bb.y,
                bb.width,
                bb.height);
    }

    // Print the prediction results (classification)
#else
    ei_printf("Predictions:\r\n");
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        ei_printf("  %s: ", ei_classifier_inferencing_categories[i]);
        ei_printf("%.5f\r\n", result.classification[i].value);
    }
#endif

    // Print anomaly result (if it exists)
#if EI_CLASSIFIER_HAS_ANOMALY
    ei_printf("Anomaly prediction: %.3f\r\n", result.anomaly);
#endif

#if EI_CLASSIFIER_HAS_VISUAL_ANOMALY
    ei_printf("Visual anomalies:\r\n");
    for (uint32_t i = 0; i < result.visual_ad_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.visual_ad_grid_cells[i];
        if (bb.value == 0) {
            continue;
        }
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n", //can get bounding box
                bb.label,
                bb.value,
                bb.x,
                bb.y,
                bb.width,
                bb.height);
    }
#endif


    free(snapshot_buf);

}

bool ei_camera_init(void) {

    if (is_initialised) return true;

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

    //initialize the camera
    // esp_err_t err = esp_camera_init(&camera_config);
    // if (err != ESP_OK) {
    //   Serial.printf("Camera init failed with error 0x%x\n", err);
    //   return false;
    // }

    sensor_t * s = esp_camera_sensor_get();
    // initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV3660_PID) {
      s->set_vflip(s, 1); // flip it back
      s->set_brightness(s, 1); // up the brightness just a bit
      s->set_saturation(s, 0); // lower the saturation
    }

// #if defined(CAMERA_MODEL_M5STACK_WIDE)
//     s->set_vflip(s, 1);
//     s->set_hmirror(s, 1);
// #elif defined(CAMERA_MODEL_ESP_EYE)
//     s->set_vflip(s, 1);
//     s->set_hmirror(s, 1);
//     s->set_awb_gain(s, 1);
// #endif

    is_initialised = true;
    return true;
}


/**
 * @brief      Capture, rescale and crop image
 *
 * @param[in]  img_width     width of output image
 * @param[in]  img_height    height of output image
 * @param[in]  out_buf       pointer to store output image, NULL may be used
 *                           if ei_camera_frame_buffer is to be used for capture and resize/cropping.
 *
 * @retval     false if not initialised, image captured, rescaled or cropped failed
 *
 */
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    bool do_resize = false;

    if (!is_initialised) {
        ei_printf("ERR: Camera is not initialized\r\n");
        return false;
    }

    camera_fb_t *fb = esp_camera_fb_get();

    if (!fb) {
        ei_printf("Camera capture failed\n");
        return false;
    }

   bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);

   esp_camera_fb_return(fb);

   if(!converted){
       ei_printf("Conversion failed\n");
       return false;
   }

    if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS)
        || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
        do_resize = true;
    }

    if (do_resize) {
        ei::image::processing::crop_and_interpolate_rgb888(
        out_buf,
        EI_CAMERA_RAW_FRAME_BUFFER_COLS,
        EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
        out_buf,
        img_width,
        img_height);
    }


    return true;
}

int ei_camera_get_data(size_t offset, size_t length, float *out_ptr)
{
    // we already have a RGB888 buffer, so recalculate offset into pixel index
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        // Swap BGR to RGB here
        // due to https://github.com/espressif/esp32-camera/issues/379
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix + 2] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix];

        // go to the next pixel
        out_ptr_ix++;
        pixel_ix+=3;
        pixels_left--;
    }
    // and done!
    return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif






