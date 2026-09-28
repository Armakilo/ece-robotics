
#ifndef EDGE_H
#define EDGE_H

// #define EI_CLASSIFIER_ALLOCATION_STATIC 1
// #define EI_CLASSIFIER_TFLITE_ENABLE_ESP_NN 0
#define EIDSP_USE_ESP_DSP 0
// #define EI_TENSOR_ARENA_LOCATION ".psram"


#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           160 //might have to relplace src_w/src_h with these
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           120
#define EI_CAMERA_FRAME_BYTE_SIZE                 3

void start_edging(void);
int ei_camera_get_data(size_t offset, size_t length, float *out_ptr);
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) ;
void find_stop(void);
bool classify_rgb888(const uint8_t *rgb, uint32_t src_w, uint32_t src_h);

extern uint8_t *snapshot_buf;

// ---- Stop sign result shared with the main loop (sent to the UNO) ----
// All positions/sizes are percent of the model's input image (0-100),
// so the UNO doesn't need to know the model resolution.
struct StopSignResult {
    bool    found;
    uint8_t cx;    // centre x, 0 = left edge, 100 = right edge
    uint8_t cy;    // centre y, 0 = top, 100 = bottom
    uint8_t w;     // box width
    uint8_t h;     // box height
    uint8_t conf;  // confidence 0-100
};

// Copies the latest result into *out. Returns true only if a new
// inference has finished since the last call.
bool get_stop_sign_result(StopSignResult *out);

#endif