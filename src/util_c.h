#ifndef _VVC_UTIL_C_H
#define _VVC_UTIL_C_H

#include "global.h"
#include "sspi.h"
#include "peripherals.h"

// C-languages utility method signatures.

// Methods for interacting with specific I2C devices.
void ssd1331_start_sequence();

// Methods for writing to the 1KB OLED framebuffer.
// These don't actually write through to the screen until
// the 'i2c_display_framebuffer' method is called (see util.S)
void oled_draw_h_line(int x, int y, int w, uint8_t color);
void oled_draw_v_line(int x, int y, int h, uint8_t color);
void oled_draw_rect(int x, int y, int w, int h,
                    int outline, uint8_t color);
void oled_write_pixel(int x, int y, uint8_t color);
void oled_draw_letter(int x, int y, unsigned int w0, unsigned int w1, uint8_t color, char size);
void oled_draw_letter_c(int x, int y, char c, uint8_t color, char size);
void oled_draw_letter_i(int x, int y, int ic, uint8_t color, char size);
void oled_draw_text(int x, int y, char* cc, uint8_t color, char size);
void sspi_stream_framebuffer(void);

// ESP8266 test/drawing methods.
void draw_esp_text_outlines(void);
void redraw_trx_state(int x, char* state);
void redraw_at_cmd(void);
void redraw_rx(void);

#endif
