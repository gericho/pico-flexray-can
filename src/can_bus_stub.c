#include "can_bus.h"

#include <string.h>

#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "hardware/timer.h"

#define CAN_DEFAULT_BITRATE 500000u
#define CAN_RX_RING_CAP 32u
#define CAN_SPI_CS_PIN 12u
#define CAN_SPI_MISO_PIN 13u
#define CAN_SPI_MOSI_PIN 14u
#define CAN_SPI_SCK_PIN 15u
#define CAN_INT_PIN 11u
#define MCP_CMD_RESET 0x0u
#define MCP_CMD_READ 0x3u
#define MCP_CMD_WRITE 0x2u
#define MCP_REG_C1CON 0x000u
#define MCP_REG_C1NBTCFG 0x004u
#define MCP_REG_C1DBTCFG 0x008u
#define MCP_REG_C1TDC 0x00Cu
#define MCP_REG_C1INT 0x01Cu
#define MCP_REG_C1BDIAG0 0x038u
#define MCP_REG_C1BDIAG1 0x03Cu
#define MCP_REG_C1TEFCON 0x040u
#define MCP_REG_C1TXQCON 0x050u
#define MCP_REG_C1FIFOCON1 0x05Cu
#define MCP_REG_C1FIFOSTA1 0x060u
#define MCP_REG_C1FIFOUA1 0x064u
#define MCP_REG_C1TREC 0x034u
#define MCP_REG_C1FLTCON0 0x1D0u
#define MCP_REG_C1FLTOBJ0 0x1F0u
#define MCP_REG_C1MASK0 0x1F4u
#define MCP_REG_OSC 0xE00u
#define MCP_RAM_START 0x400u
#define MCP_RAM_END   0xC00u
#define MCP_CON_TXQEN_BITS (1u << 20)
#define MCP_FIFO_TFNRFNIF_BITS (1u << 0)
#define MCP_TREC_TXBO_BITS (1u << 21)
#define MCP_CON_REQOP_SHIFT 24u
#define MCP_CON_OPMOD_SHIFT 21u
#define MCP_MODE_LISTEN_ONLY 3u
#define MCP_MODE_CONFIG 4u
#define MCP_MODE_CAN20 6u
#define MCP_INT_RXIE_BITS (1u << 17)
#define MCP_FIFO_FRESET_BITS (1u << 10)
#define MCP_FIFO_TXEN_BITS (1u << 7)
#define MCP_FIFO_RXTSEN_BITS (1u << 5)
#define MCP_FIFO_UINC_BITS (1u << 8)

static volatile can_bus_frame_t g_can_frames[CAN_RX_RING_CAP];
static volatile uint32_t g_can_head;
static volatile uint32_t g_can_tail;
static can_bus_status_t g_status;
static bool g_pins_initialized;
static uint32_t g_last_read;
static bool g_mode_ok;
static bool g_timing_ok;
static bool g_can20_ok;
static bool g_int_ok;
static bool g_fifo_ok;
static bool g_filter_ok;
static bool g_poll_ok;
static bool g_ua_ok;
static bool g_obj_ok;
static bool g_uinc_ok;
static bool g_parse_ok;
static bool g_push_ok;
static uint32_t g_debug_int_low_count;
static uint32_t g_debug_fifo_nonempty_count;
static uint32_t g_debug_last_fifosta;
static uint32_t g_debug_last_trec;
static uint32_t g_debug_last_con;
static uint32_t g_debug_last_int;
static uint32_t g_debug_last_bdiag0;
static uint32_t g_debug_last_bdiag1;
static uint32_t g_debug_con_after_reset;
static uint32_t g_debug_con_after_config_req;
static uint32_t g_debug_con_after_can20_req;
static uint32_t g_debug_last_osc;
static uint32_t g_debug_last_nbtcfg;
static uint32_t g_debug_init_flags;
static uint32_t g_debug_queue_flags;
static uint32_t g_debug_last_txqcon;
static uint32_t g_debug_last_fifocon1;

static inline void can_spi_delay(void) {
  __asm volatile("nop\nnop\nnop\nnop");
}

static void can_spi_select(void) {
  gpio_put(CAN_SPI_CS_PIN, 0);
  can_spi_delay();
}

static void can_spi_deselect(void) {
  can_spi_delay();
  gpio_put(CAN_SPI_CS_PIN, 1);
  can_spi_delay();
}

static uint8_t can_spi_transfer_byte(uint8_t tx) {
  uint8_t rx = 0;
  for (int bit = 7; bit >= 0; --bit) {
    gpio_put(CAN_SPI_MOSI_PIN, (tx >> bit) & 0x1u);
    can_spi_delay();
    gpio_put(CAN_SPI_SCK_PIN, 1);
    can_spi_delay();
    rx = (uint8_t)((rx << 1) | (gpio_get(CAN_SPI_MISO_PIN) ? 1u : 0u));
    gpio_put(CAN_SPI_SCK_PIN, 0);
  }
  return rx;
}

static void mcp_send_header(uint8_t cmd, uint16_t addr) {
  uint16_t header = (uint16_t)(((uint16_t)cmd << 12) | (addr & 0x0fffu));
  (void)can_spi_transfer_byte((uint8_t)(header >> 8));
  (void)can_spi_transfer_byte((uint8_t)(header & 0xffu));
}

static void mcp_reset_device(void) {
  can_spi_select();
  mcp_send_header(MCP_CMD_RESET, 0u);
  can_spi_deselect();
  sleep_ms(2);
}

static uint32_t mcp_read32(uint16_t addr) {
  uint8_t raw[4];
  can_spi_select();
  mcp_send_header(MCP_CMD_READ, addr);
  for (int i = 0; i < 4; ++i) {
    raw[i] = can_spi_transfer_byte(0xffu);
  }
  can_spi_deselect();
  return (uint32_t)raw[0]
       | ((uint32_t)raw[1] << 8)
       | ((uint32_t)raw[2] << 16)
       | ((uint32_t)raw[3] << 24);
}

static void mcp_read_bytes(uint16_t addr, uint8_t *dst, size_t len) {
  can_spi_select();
  mcp_send_header(MCP_CMD_READ, addr);
  for (size_t i = 0; i < len; ++i) {
    dst[i] = can_spi_transfer_byte(0xffu);
  }
  can_spi_deselect();
}

static void mcp_write32(uint16_t addr, uint32_t value) {
  uint8_t raw[4];
  raw[0] = (uint8_t)(value & 0xffu);
  raw[1] = (uint8_t)((value >> 8) & 0xffu);
  raw[2] = (uint8_t)((value >> 16) & 0xffu);
  raw[3] = (uint8_t)((value >> 24) & 0xffu);
  can_spi_select();
  mcp_send_header(MCP_CMD_WRITE, addr);
  for (int i = 0; i < 4; ++i) {
    (void)can_spi_transfer_byte(raw[i]);
  }
  can_spi_deselect();
}

static void mcp_write8(uint16_t addr, uint8_t value) {
  can_spi_select();
  mcp_send_header(MCP_CMD_WRITE, addr);
  (void)can_spi_transfer_byte(value);
  can_spi_deselect();
}

static void mcp_zero_message_ram(void) {
  static const uint8_t zeros[16] = {0};
  for (uint16_t addr = MCP_RAM_START; addr < MCP_RAM_END; addr += sizeof(zeros)) {
    can_spi_select();
    mcp_send_header(MCP_CMD_WRITE, addr);
    for (size_t i = 0; i < sizeof(zeros); ++i) {
      (void)can_spi_transfer_byte(zeros[i]);
    }
    can_spi_deselect();
  }
}


static bool mcp_wait_mode(uint8_t mode, uint32_t timeout_ms) {
  absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
  while (!time_reached(deadline)) {
    uint32_t con = mcp_read32(MCP_REG_C1CON);
    if (((con >> MCP_CON_OPMOD_SHIFT) & 0x7u) == mode) {
      return true;
    }
    sleep_us(200);
  }
  return false;
}

static bool mcp_request_mode(uint8_t mode) {
  uint32_t reg = mcp_read32(MCP_REG_C1CON);
  reg &= ~(0x7u << MCP_CON_REQOP_SHIFT);
  reg |= ((uint32_t)mode << MCP_CON_REQOP_SHIFT);
  mcp_write32(MCP_REG_C1CON, reg);
  return mcp_wait_mode(mode, 20u);
}

static void mcp_configure_timing_500k(void) {
  // 500 kbps nominal timing for a 40 MHz CAN clock, keeping the controller in
  // passive classic-CAN observation mode.
  // Candidate from the original MCP2518FD driver branch.
  const uint32_t nbtcfg = (1u << 24) | (30u << 16) | (7u << 8) | 7u;
  // Keep the data-phase timing benign; CAN2.0 listen-only relies on nominal timing.
  const uint32_t dbtcfg = (15u << 24) | (15u << 16) | (15u << 8) | 0u;
  mcp_write32(MCP_REG_C1NBTCFG, nbtcfg);
  mcp_write32(MCP_REG_C1DBTCFG, dbtcfg);
  mcp_write32(MCP_REG_C1TDC, 0u);
}

static void mcp_configure_interrupts_only(void) {
  mcp_write32(MCP_REG_C1TEFCON, 0u);
  mcp_write32(MCP_REG_C1INT, MCP_INT_RXIE_BITS);
}

static bool mcp_configure_queues_only(void) {
  g_debug_queue_flags = 0u;

  mcp_write32(MCP_REG_C1TXQCON,
              (0u << 29) |
              (0u << 24) |
              (0u << 21) |
              (0u << 16) |
              MCP_FIFO_FRESET_BITS |
              MCP_FIFO_TXEN_BITS);
  g_debug_last_txqcon = mcp_read32(MCP_REG_C1TXQCON);
  g_debug_queue_flags |= (1u << 0);

  mcp_write32(MCP_REG_C1FIFOCON1,
              (0u << 29) |
              (0u << 24) |
              MCP_FIFO_RXTSEN_BITS |
              MCP_FIFO_FRESET_BITS);
  g_debug_last_fifocon1 = mcp_read32(MCP_REG_C1FIFOCON1);
  g_debug_queue_flags |= (1u << 1);

  return true;
}

static void mcp_configure_filters_only(void) {
  // Program filter object/mask only while the filter is explicitly disabled.
  mcp_write32(MCP_REG_C1FLTCON0, 0u);
  mcp_write32(MCP_REG_C1FLTOBJ0, 0u);
  mcp_write32(MCP_REG_C1MASK0, 0u);
  mcp_write32(MCP_REG_C1FLTCON0, (1u << 7) | 1u);
}

static uint8_t can_len_from_dlc(uint8_t dlc) {
  return dlc <= 8u ? dlc : 8u;
}

static bool parse_can_object(const uint8_t *obj, can_bus_frame_t *frame) {
  uint32_t r0 = (uint32_t)obj[0]
              | ((uint32_t)obj[1] << 8)
              | ((uint32_t)obj[2] << 16)
              | ((uint32_t)obj[3] << 24);
  uint32_t r1 = (uint32_t)obj[4]
              | ((uint32_t)obj[5] << 8)
              | ((uint32_t)obj[6] << 16)
              | ((uint32_t)obj[7] << 24);

  memset(frame, 0, sizeof(*frame));
  frame->extended = ((r1 >> 4) & 0x1u) != 0u;
  frame->rtr = ((r1 >> 5) & 0x1u) != 0u;
  frame->dlc = can_len_from_dlc((uint8_t)(r1 & 0x0fu));
  frame->timestamp_us = (uint32_t)obj[8]
                      | ((uint32_t)obj[9] << 8)
                      | ((uint32_t)obj[10] << 16)
                      | ((uint32_t)obj[11] << 24);

  if (frame->extended) {
    uint32_t sid = r0 & 0x7ffu;
    uint32_t eid = (r0 >> 11) & 0x3ffffu;
    frame->id = (sid << 18) | eid;
  } else {
    frame->id = r0 & 0x7ffu;
  }

  if (!frame->rtr) {
    memcpy(frame->data, &obj[12], frame->dlc);
  }
  return true;
}

static bool can_push_frame(const can_bus_frame_t *frame) {
  uint32_t head = g_can_head;
  uint32_t next = (head + 1u) % CAN_RX_RING_CAP;
  if (next == g_can_tail) {
    g_status.overflow_count++;
    return false;
  }
  g_can_frames[head] = *frame;
  g_can_head = next;
  g_status.rx_total++;
  return true;
}

static void can_bus_init_pins_once(void) {
  if (g_pins_initialized) {
    return;
  }

  gpio_init(CAN_SPI_CS_PIN);
  gpio_set_dir(CAN_SPI_CS_PIN, GPIO_OUT);
  gpio_put(CAN_SPI_CS_PIN, 1);

  gpio_init(CAN_SPI_SCK_PIN);
  gpio_set_dir(CAN_SPI_SCK_PIN, GPIO_OUT);
  gpio_put(CAN_SPI_SCK_PIN, 0);

  gpio_init(CAN_SPI_MOSI_PIN);
  gpio_set_dir(CAN_SPI_MOSI_PIN, GPIO_OUT);
  gpio_put(CAN_SPI_MOSI_PIN, 0);

  gpio_init(CAN_SPI_MISO_PIN);
  gpio_set_dir(CAN_SPI_MISO_PIN, GPIO_IN);
  gpio_pull_up(CAN_SPI_MISO_PIN);

  gpio_init(CAN_INT_PIN);
  gpio_set_dir(CAN_INT_PIN, GPIO_IN);
  gpio_pull_up(CAN_INT_PIN);

  g_pins_initialized = true;
}

void can_bus_init(void) {
  memset(&g_status, 0, sizeof(g_status));
  g_can_head = 0u;
  g_can_tail = 0u;
  g_last_read = 0u;
  g_mode_ok = false;
  g_timing_ok = false;
  g_can20_ok = false;
  g_int_ok = false;
  g_fifo_ok = false;
  g_filter_ok = false;
  g_poll_ok = false;
  g_ua_ok = false;
  g_obj_ok = false;
  g_uinc_ok = false;
  g_parse_ok = false;
  g_push_ok = false;
  g_debug_int_low_count = 0u;
  g_debug_fifo_nonempty_count = 0u;
  g_debug_last_fifosta = 0u;
  g_debug_last_trec = 0u;
  g_debug_last_con = 0u;
  g_debug_last_int = 0u;
  g_debug_last_bdiag0 = 0u;
  g_debug_last_bdiag1 = 0u;
  g_debug_con_after_reset = 0u;
  g_debug_con_after_config_req = 0u;
  g_debug_con_after_can20_req = 0u;
  g_debug_last_osc = 0u;
  g_debug_last_nbtcfg = 0u;
  g_debug_init_flags = 0u;
  g_debug_queue_flags = 0u;
  g_debug_last_txqcon = 0u;
  g_debug_last_fifocon1 = 0u;
  can_bus_init_pins_once();
}

void can_bus_poll(void) {
  if (!g_filter_ok) {
    return;
  }
  bool int_low = !gpio_get(CAN_INT_PIN);
  g_debug_last_con = mcp_read32(MCP_REG_C1CON);
  g_debug_last_int = mcp_read32(MCP_REG_C1INT);
  g_debug_last_bdiag0 = mcp_read32(MCP_REG_C1BDIAG0);
  g_debug_last_bdiag1 = mcp_read32(MCP_REG_C1BDIAG1);
  g_debug_last_trec = mcp_read32(MCP_REG_C1TREC);
  g_debug_last_osc = mcp_read32(MCP_REG_OSC);
  g_debug_last_nbtcfg = mcp_read32(MCP_REG_C1NBTCFG);
  g_debug_last_fifosta = mcp_read32(MCP_REG_C1FIFOSTA1);
  if (int_low) {
    g_debug_int_low_count++;
  }
  if ((g_debug_last_fifosta & MCP_FIFO_TFNRFNIF_BITS) != 0u) {
    uint32_t ua = mcp_read32(MCP_REG_C1FIFOUA1);
    uint8_t obj[20];
    can_bus_frame_t frame;
    g_debug_fifo_nonempty_count++;
    g_ua_ok = true;
    mcp_read_bytes((uint16_t)((MCP_RAM_START + ua) & 0x0fffu), obj, sizeof(obj));
    g_obj_ok = true;
    mcp_write8(MCP_REG_C1FIFOCON1 + 1u, 1u);
    g_uinc_ok = true;
    g_parse_ok = parse_can_object(obj, &frame);
    if (g_parse_ok) {
      g_push_ok = can_push_frame(&frame);
    }
  }
  g_poll_ok = true;
}

void can_bus_reset(void) {
  bool keep_started = g_status.started;
  uint32_t keep_bitrate = g_status.bitrate;
  memset(&g_status, 0, sizeof(g_status));
  g_status.started = keep_started;
  g_status.bitrate = keep_bitrate;
}

bool can_bus_set_bitrate(uint32_t bitrate) {
  can_bus_init_pins_once();
  g_debug_init_flags = 0u;
  g_status.started = false;
  if (bitrate != CAN_DEFAULT_BITRATE) {
    return false;
  }

  g_debug_init_flags |= (1u << 0);
  g_status.bitrate = bitrate;
  mcp_reset_device();
  g_debug_init_flags |= (1u << 1);
  g_last_read = mcp_read32(MCP_REG_C1CON);
  g_debug_con_after_reset = g_last_read;

  g_mode_ok = mcp_request_mode(MCP_MODE_CONFIG);
  g_debug_con_after_config_req = mcp_read32(MCP_REG_C1CON);
  if (!g_mode_ok) {
    goto done;
  }
  g_debug_init_flags |= (1u << 2);

  uint32_t con = mcp_read32(MCP_REG_C1CON);
  con |= MCP_CON_TXQEN_BITS;
  con &= ~(0x7u << MCP_CON_REQOP_SHIFT);
  con |= ((uint32_t)MCP_MODE_CONFIG << MCP_CON_REQOP_SHIFT);
  mcp_write32(MCP_REG_C1CON, con);
  g_debug_last_con = mcp_read32(MCP_REG_C1CON);

  mcp_zero_message_ram();
  mcp_configure_timing_500k();
  g_timing_ok = true;
  g_debug_init_flags |= (1u << 3);

  mcp_configure_interrupts_only();
  g_int_ok = true;
  g_debug_init_flags |= (1u << 4);

  g_fifo_ok = mcp_configure_queues_only();
  if (!g_fifo_ok) {
    goto done;
  }
  g_debug_init_flags |= (1u << 5);

  mcp_configure_filters_only();
  g_filter_ok = true;
  g_debug_init_flags |= (1u << 6);

  g_can20_ok = mcp_request_mode(MCP_MODE_LISTEN_ONLY);
  g_debug_con_after_can20_req = mcp_read32(MCP_REG_C1CON);
  if (g_can20_ok) {
    g_debug_init_flags |= (1u << 7);
  }

  g_debug_last_txqcon = mcp_read32(MCP_REG_C1TXQCON);
  g_debug_last_fifocon1 = mcp_read32(MCP_REG_C1FIFOCON1);
  if ((g_debug_last_txqcon & MCP_FIFO_FRESET_BITS) == 0u) {
    g_debug_queue_flags |= (1u << 2);
  }
  if ((g_debug_last_fifocon1 & MCP_FIFO_FRESET_BITS) == 0u) {
    g_debug_queue_flags |= (1u << 3);
  }

 done:
  g_debug_last_osc = mcp_read32(MCP_REG_OSC);
  g_debug_last_nbtcfg = mcp_read32(MCP_REG_C1NBTCFG);
  g_status.started = g_can20_ok;
  return true;
}

uint32_t can_bus_get_bitrate(void) {
  return g_status.bitrate;
}

bool can_bus_is_started(void) {
  return g_status.started;
}

bool can_bus_pop_frame(can_bus_frame_t *frame) {
  uint32_t tail = g_can_tail;
  if (tail == g_can_head) {
    return false;
  }
  if (frame) {
    *frame = g_can_frames[tail];
  }
  g_can_tail = (tail + 1u) % CAN_RX_RING_CAP;
  return true;
}

bool can_bus_send_frame(const can_bus_frame_t *frame) {
  (void)frame;
  return false;
}

void can_bus_get_status(can_bus_status_t *status) {
  if (status) {
    *status = g_status;
    status->rec = (uint8_t)(g_debug_last_trec & 0xffu);
    status->tec = (uint8_t)((g_debug_last_trec >> 8) & 0xffu);
    status->rx_total = g_status.rx_total;
    status->tx_total = g_status.tx_total;
    status->tx_attempt = g_status.tx_attempt;
    status->fwd_count = 0u;
    status->checksum_error_count = 0u;
    status->overflow_count = g_status.overflow_count;
    status->error_count = g_status.error_count;
    status->bus_off = (g_debug_last_trec & MCP_TREC_TXBO_BITS) != 0u;
  }
}
