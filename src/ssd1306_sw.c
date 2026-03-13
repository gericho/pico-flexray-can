#include "ssd1306_sw.h"

#include <ctype.h>
#include <string.h>

#include "hardware/gpio.h"
#include "pico/stdlib.h"

#include "ssd1306_font.h"

#define SSD1306_I2C_SDA_PIN 29
#define SSD1306_I2C_SCL_PIN 30
#define SSD1306_I2C_DELAY_US 5

#define SSD1306_SET_MEM_MODE 0x20
#define SSD1306_SET_COL_ADDR 0x21
#define SSD1306_SET_PAGE_ADDR 0x22
#define SSD1306_SET_SCROLL 0x2E
#define SSD1306_SET_DISP_START_LINE 0x40
#define SSD1306_SET_CONTRAST 0x81
#define SSD1306_SET_CHARGE_PUMP 0x8D
#define SSD1306_SET_SEG_REMAP 0xA0
#define SSD1306_SET_ENTIRE_ON 0xA4
#define SSD1306_SET_NORM_DISP 0xA6
#define SSD1306_SET_MUX_RATIO 0xA8
#define SSD1306_SET_DISP 0xAE
#define SSD1306_SET_COM_OUT_DIR 0xC0
#define SSD1306_SET_DISP_OFFSET 0xD3
#define SSD1306_SET_DISP_CLK_DIV 0xD5
#define SSD1306_SET_PRECHARGE 0xD9
#define SSD1306_SET_COM_PIN_CFG 0xDA
#define SSD1306_SET_VCOM_DESEL 0xDB

static bool ssd1306_ready = false;
static uint8_t ssd1306_addr = 0x3C;
static uint8_t ssd1306_buf[SSD1306_BUF_LEN];

static inline void ssd1306_i2c_delay(void) {
  sleep_us(SSD1306_I2C_DELAY_US);
}

static inline void ssd1306_sda_high(void) {
  gpio_set_dir(SSD1306_I2C_SDA_PIN, GPIO_IN);
}

static inline void ssd1306_sda_low(void) {
  gpio_put(SSD1306_I2C_SDA_PIN, 0);
  gpio_set_dir(SSD1306_I2C_SDA_PIN, GPIO_OUT);
}

static inline void ssd1306_scl_high(void) {
  gpio_set_dir(SSD1306_I2C_SCL_PIN, GPIO_IN);
}

static inline void ssd1306_scl_low(void) {
  gpio_put(SSD1306_I2C_SCL_PIN, 0);
  gpio_set_dir(SSD1306_I2C_SCL_PIN, GPIO_OUT);
}

static inline bool ssd1306_read_sda(void) {
  return gpio_get(SSD1306_I2C_SDA_PIN);
}

static void ssd1306_i2c_init(void) {
  gpio_init(SSD1306_I2C_SDA_PIN);
  gpio_init(SSD1306_I2C_SCL_PIN);
  gpio_pull_up(SSD1306_I2C_SDA_PIN);
  gpio_pull_up(SSD1306_I2C_SCL_PIN);
  ssd1306_sda_high();
  ssd1306_scl_high();
  ssd1306_i2c_delay();
}

static void ssd1306_i2c_start(void) {
  ssd1306_sda_high();
  ssd1306_scl_high();
  ssd1306_i2c_delay();
  ssd1306_sda_low();
  ssd1306_i2c_delay();
  ssd1306_scl_low();
  ssd1306_i2c_delay();
}

static void ssd1306_i2c_stop(void) {
  ssd1306_sda_low();
  ssd1306_i2c_delay();
  ssd1306_scl_high();
  ssd1306_i2c_delay();
  ssd1306_sda_high();
  ssd1306_i2c_delay();
}

static bool ssd1306_i2c_write_byte(uint8_t byte) {
  for (int i = 7; i >= 0; --i) {
    if (byte & (1u << i)) {
      ssd1306_sda_high();
    } else {
      ssd1306_sda_low();
    }
    ssd1306_i2c_delay();
    ssd1306_scl_high();
    ssd1306_i2c_delay();
    ssd1306_scl_low();
    ssd1306_i2c_delay();
  }

  ssd1306_sda_high();
  ssd1306_i2c_delay();
  ssd1306_scl_high();
  ssd1306_i2c_delay();
  bool ack = !ssd1306_read_sda();
  ssd1306_scl_low();
  ssd1306_i2c_delay();
  return ack;
}

static bool ssd1306_i2c_write(uint8_t addr, const uint8_t *data, size_t len) {
  ssd1306_i2c_start();
  if (!ssd1306_i2c_write_byte((uint8_t)(addr << 1))) {
    ssd1306_i2c_stop();
    return false;
  }
  for (size_t i = 0; i < len; ++i) {
    if (!ssd1306_i2c_write_byte(data[i])) {
      ssd1306_i2c_stop();
      return false;
    }
  }
  ssd1306_i2c_stop();
  return true;
}

static bool ssd1306_probe(uint8_t addr) {
  ssd1306_i2c_start();
  bool ack = ssd1306_i2c_write_byte((uint8_t)(addr << 1));
  ssd1306_i2c_stop();
  return ack;
}

static void ssd1306_send_cmd(uint8_t cmd) {
  uint8_t buf[2] = {0x00, cmd};
  (void)ssd1306_i2c_write(ssd1306_addr, buf, sizeof(buf));
}

static void ssd1306_send_cmd_list(const uint8_t *cmds, size_t len) {
  for (size_t i = 0; i < len; ++i) {
    ssd1306_send_cmd(cmds[i]);
  }
}

static void ssd1306_send_data(const uint8_t *buf, size_t len) {
  uint8_t temp[17];
  temp[0] = 0x40;
  size_t offset = 0;
  while (offset < len) {
    size_t chunk = len - offset;
    if (chunk > 16) {
      chunk = 16;
    }
    memcpy(&temp[1], &buf[offset], chunk);
    (void)ssd1306_i2c_write(ssd1306_addr, temp, chunk + 1);
    offset += chunk;
  }
}

static int ssd1306_font_index(uint8_t ch) {
  if (ch >= 'A' && ch <= 'Z') {
    return ch - 'A' + 1;
  }
  if (ch >= '0' && ch <= '9') {
    return ch - '0' + 27;
  }
  return 0;
}

bool ssd1306_init(void) {
  ssd1306_i2c_init();
  if (ssd1306_probe(0x3C)) {
    ssd1306_addr = 0x3C;
  } else if (ssd1306_probe(0x3D)) {
    ssd1306_addr = 0x3D;
  } else {
    ssd1306_ready = false;
    return false;
  }

  static const uint8_t cmds[] = {
    SSD1306_SET_DISP,
    SSD1306_SET_MEM_MODE, 0x00,
    SSD1306_SET_DISP_START_LINE,
    SSD1306_SET_SEG_REMAP | 0x01,
    SSD1306_SET_MUX_RATIO, SSD1306_HEIGHT - 1,
    SSD1306_SET_COM_OUT_DIR | 0x08,
    SSD1306_SET_DISP_OFFSET, 0x00,
    SSD1306_SET_COM_PIN_CFG, 0x12,
    SSD1306_SET_DISP_CLK_DIV, 0x80,
    SSD1306_SET_PRECHARGE, 0xF1,
    SSD1306_SET_VCOM_DESEL, 0x30,
    SSD1306_SET_CONTRAST, 0xFF,
    SSD1306_SET_ENTIRE_ON,
    SSD1306_SET_NORM_DISP,
    SSD1306_SET_CHARGE_PUMP, 0x14,
    SSD1306_SET_SCROLL,
    SSD1306_SET_DISP | 0x01,
  };
  ssd1306_send_cmd_list(cmds, sizeof(cmds));
  ssd1306_ready = true;
  ssd1306_clear();
  ssd1306_present();
  return true;
}

bool ssd1306_is_ready(void) {
  return ssd1306_ready;
}

void ssd1306_clear(void) {
  memset(ssd1306_buf, 0, sizeof(ssd1306_buf));
}

void ssd1306_draw_string(uint8_t x, uint8_t y, const char *str) {
  if (!ssd1306_ready || y >= SSD1306_HEIGHT || x >= SSD1306_WIDTH) {
    return;
  }

  uint8_t page = y / 8;
  size_t pos = (size_t)page * SSD1306_WIDTH + x;
  while (*str && pos + 8 <= sizeof(ssd1306_buf)) {
    unsigned char ch = (unsigned char)toupper((unsigned char)*str++);
    int idx = ssd1306_font_index(ch);
    memcpy(&ssd1306_buf[pos], &ssd1306_font[idx * 8], 8);
    pos += 8;
    if ((pos % SSD1306_WIDTH) > (SSD1306_WIDTH - 8)) {
      break;
    }
  }
}

void ssd1306_present(void) {
  if (!ssd1306_ready) {
    return;
  }

  static const uint8_t cmds[] = {
    SSD1306_SET_COL_ADDR, 0, SSD1306_WIDTH - 1,
    SSD1306_SET_PAGE_ADDR, 0, SSD1306_NUM_PAGES - 1,
  };
  ssd1306_send_cmd_list(cmds, sizeof(cmds));
  ssd1306_send_data(ssd1306_buf, sizeof(ssd1306_buf));
}
