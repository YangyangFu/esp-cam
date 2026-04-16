#line 1 "/Users/yangyang.fu/github/esp-cam/resources/C/Sketches/Sketch_Fridge_CAM/avi_writer.h"
#ifndef AVI_WRITER_H
#define AVI_WRITER_H

#include <FS.h>
#include <stdint.h>
#include <stddef.h>

// Maximum number of frames we can index in RAM (~28KB at 3600 entries)
#define AVI_MAX_FRAMES 3600

bool avi_start(fs::FS &fs, const char *path, uint16_t width, uint16_t height, uint8_t fps);
bool avi_add_frame(const uint8_t *jpg_buf, size_t jpg_len);
bool avi_end();
bool avi_is_open();
uint32_t avi_frame_count();

#endif // AVI_WRITER_H
