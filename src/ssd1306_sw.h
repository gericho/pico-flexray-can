#ifndef SSD1306_SW_H
#define SSD1306_SW_H

#include <stdbool.h>
#include <stdint.h>

#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64
#define SSD1306_NUM_PAGES (SSD1306_HEIGHT / 8)
#define SSD1306_BUF_LEN (SSD1306_WIDTH * SSD1306_NUM_PAGES)

bool ssd1306_init(void);
bool ssd1306_is_ready(void);
void ssd1306_clear(void);
void ssd1306_draw_string(uint8_t x, uint8_t y, const char *str);
void ssd1306_present(void);

#endif
