
#ifndef EDGE_H
#define EDGE_H




#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           96//320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           96//240
#define EI_CAMERA_FRAME_BYTE_SIZE                 3

void start_edging(void);
int ei_camera_get_data(size_t offset, size_t length, float *out_ptr);
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) ;
void find_stop(void);

extern uint8_t *snapshot_buf;

#endif