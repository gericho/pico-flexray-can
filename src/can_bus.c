#include "can_bus.h"

#include <stdio.h>
#include <string.h>

#include "hardware/gpio.h"
#include "pico/stdlib.h"

#define CAN_DEFAULT_BITRATE 500000u
#define CAN_RX_RING_CAP 32u

#define CAN_SPI_CS_PIN 12u
#define CAN_SPI_MISO_PIN 13u
#define CAN_SPI_MOSI_PIN 14u
#define CAN_SPI_SCK_PIN 15u
#define CAN_INT_PIN 11u

#define MCP_CMD_RESET 0x0u
#define MCP_CMD_WRITE 0x2u
#define MCP_CMD_READ 0x3u

#define MCP_REG_C1CON 0x000u
#define MCP_REG_C1NBTCFG 0x004u
#define MCP_REG_C1DBTCFG 0x008u
#define MCP_REG_C1TDC 0x00Cu
#define MCP_REG_C1INT 0x01Cu
#define MCP_REG_C1TXREQ 0x030u
#define MCP_REG_C1TREC 0x034u
#define MCP_REG_C1TEFCON 0x040u
#define MCP_REG_C1TXQCON 0x050u
#define MCP_REG_C1TXQSTA 0x054u
#define MCP_REG_C1TXQUA 0x058u
#define MCP_REG_C1FIFOCON1 0x05Cu
#define MCP_REG_C1FIFOSTA1 0x060u
#define MCP_REG_C1FIFOUA1 0x064u
#define MCP_REG_C1FLTCON0 0x1D0u
#define MCP_REG_C1FLTOBJ0 0x1F0u
#define MCP_REG_C1MASK0 0x1F4u

#define MCP_RAM_START 0x400u
#define MCP_RAM_END   0xC00u

#define MCP_CON_REQOP_SHIFT 24u
#define MCP_CON_OPMOD_SHIFT 21u
#define MCP_CON_TXQEN_BITS (1u << 20)

#define MCP_FIFO_UINC_BITS (1u << 8)
#define MCP_FIFO_TXREQ_BITS (1u << 9)
#define MCP_FIFO_FRESET_BITS (1u << 10)
#define MCP_FIFO_TXEN_BITS (1u << 7)
#define MCP_FIFO_RXTSEN_BITS (1u << 5)
#define MCP_FIFO_TFNRFNIF_BITS (1u << 0)
#define MCP_FIFO_TXQNIF_BITS (1u << 0)

#define MCP_INT_RXIE_BITS (1u << 17)

#define MCP_TREC_TXBO_BITS (1u << 21)

#define MCP_MODE_CANFD 0u
#define MCP_MODE_SLEEP 1u
#define MCP_MODE_INT_LOOPBACK 2u
#define MCP_MODE_LISTEN_ONLY 3u
#define MCP_MODE_CONFIG 4u
#define MCP_MODE_EXT_LOOPBACK 5u
#define MCP_MODE_CAN20 6u

static volatile can_bus_frame_t can_frames[CAN_RX_RING_CAP];
static volatile uint32_t can_head;
static volatile uint32_t can_tail;

static uint32_t current_bitrate = 0u;
static uint32_t can_rx_total;
static uint32_t can_tx_total;
static uint32_t can_tx_attempt;
static uint32_t can_overflow_count;
static uint32_t can_error_count;
static bool can_started;
static bool can_io_initialized;
static uint32_t tx_sequence;
static uint32_t can_debug_c1con;
static uint32_t can_debug_c1txqcon;
static uint32_t can_debug_c1txqsta;
static uint32_t can_debug_c1txqua;
static uint8_t can_send_fail_reason;

static inline void can_spi_delay(void)
{
    __asm volatile("nop\nnop\nnop\nnop");
}

static void can_spi_select(void)
{
    gpio_put(CAN_SPI_CS_PIN, 0);
    can_spi_delay();
}

static void can_spi_deselect(void)
{
    can_spi_delay();
    gpio_put(CAN_SPI_CS_PIN, 1);
    can_spi_delay();
}

static uint8_t can_spi_transfer_byte(uint8_t tx)
{
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

static void mcp_send_header(uint8_t cmd, uint16_t addr)
{
    uint16_t header = (uint16_t)(((uint16_t)cmd << 12) | (addr & 0x0fffu));
    (void)can_spi_transfer_byte((uint8_t)(header >> 8));
    (void)can_spi_transfer_byte((uint8_t)(header & 0xffu));
}

static void mcp_reset_device(void)
{
    can_spi_select();
    mcp_send_header(MCP_CMD_RESET, 0u);
    can_spi_deselect();
    sleep_ms(2);
}

static void mcp_read_bytes(uint16_t addr, uint8_t *dst, size_t len)
{
    can_spi_select();
    mcp_send_header(MCP_CMD_READ, addr);
    for (size_t i = 0; i < len; ++i) {
        dst[i] = can_spi_transfer_byte(0xffu);
    }
    can_spi_deselect();
}

static void mcp_write_bytes(uint16_t addr, const uint8_t *src, size_t len)
{
    can_spi_select();
    mcp_send_header(MCP_CMD_WRITE, addr);
    for (size_t i = 0; i < len; ++i) {
        (void)can_spi_transfer_byte(src[i]);
    }
    can_spi_deselect();
}

static uint32_t mcp_read32(uint16_t addr)
{
    uint8_t raw[4];
    mcp_read_bytes(addr, raw, sizeof(raw));
    return (uint32_t)raw[0]
         | ((uint32_t)raw[1] << 8)
         | ((uint32_t)raw[2] << 16)
         | ((uint32_t)raw[3] << 24);
}

static void mcp_write32(uint16_t addr, uint32_t value)
{
    uint8_t raw[4];
    raw[0] = (uint8_t)(value & 0xffu);
    raw[1] = (uint8_t)((value >> 8) & 0xffu);
    raw[2] = (uint8_t)((value >> 16) & 0xffu);
    raw[3] = (uint8_t)((value >> 24) & 0xffu);
    mcp_write_bytes(addr, raw, sizeof(raw));
}

static void mcp_write8(uint16_t addr, uint8_t value)
{
    mcp_write_bytes(addr, &value, 1u);
}

static void mcp_update_bits(uint16_t addr, uint32_t mask, uint32_t value)
{
    uint32_t reg = mcp_read32(addr);
    reg &= ~mask;
    reg |= value & mask;
    mcp_write32(addr, reg);
}

static void mcp_zero_message_ram(void)
{
    static const uint8_t zeros[16] = {0};
    for (uint16_t addr = MCP_RAM_START; addr < MCP_RAM_END; addr += sizeof(zeros)) {
        mcp_write_bytes(addr, zeros, sizeof(zeros));
    }
}

static bool mcp_wait_mode(uint8_t mode, uint32_t timeout_ms)
{
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

static bool mcp_request_mode(uint8_t mode)
{
    mcp_update_bits(MCP_REG_C1CON, 0x7u << MCP_CON_REQOP_SHIFT, (uint32_t)mode << MCP_CON_REQOP_SHIFT);
    return mcp_wait_mode(mode, 20u);
}

static bool can_push_frame(const can_bus_frame_t *frame)
{
    uint32_t head = can_head;
    uint32_t next = (head + 1u) % CAN_RX_RING_CAP;

    if (next == can_tail) {
        can_overflow_count++;
        return false;
    }

    can_frames[head] = *frame;
    can_head = next;
    return true;
}

static void mcp_configure_timing_500k(void)
{
    const uint32_t nbtcfg =
        (1u << 24) |
        (30u << 16) |
        (7u << 8) |
        7u;
    const uint32_t dbtcfg =
        (15u << 24) |
        (15u << 16) |
        (15u << 8) |
        0u;

    mcp_write32(MCP_REG_C1NBTCFG, nbtcfg);
    mcp_write32(MCP_REG_C1DBTCFG, dbtcfg);
    mcp_write32(MCP_REG_C1TDC, 0u);
}

static bool mcp_init_controller(void)
{
    mcp_reset_device();

    if (!mcp_wait_mode(MCP_MODE_CONFIG, 20u)) {
        return false;
    }

    mcp_write32(MCP_REG_C1CON, ((uint32_t)MCP_MODE_CONFIG << MCP_CON_REQOP_SHIFT) | MCP_CON_TXQEN_BITS);
    mcp_zero_message_ram();
    mcp_configure_timing_500k();

    mcp_write32(MCP_REG_C1TEFCON, 0u);
    mcp_write32(MCP_REG_C1INT, MCP_INT_RXIE_BITS);

    mcp_write32(
        MCP_REG_C1TXQCON,
        (0u << 29) |
        (0u << 24) |
        (0u << 21) |
        (0u << 16) |
        MCP_FIFO_FRESET_BITS |
        MCP_FIFO_TXEN_BITS
    );

    mcp_write32(
        MCP_REG_C1FIFOCON1,
        (0u << 29) |
        (0u << 24) |
        MCP_FIFO_FRESET_BITS |
        MCP_FIFO_RXTSEN_BITS
    );

    mcp_write32(MCP_REG_C1FLTOBJ0, 0u);
    mcp_write32(MCP_REG_C1MASK0, 0u);
    mcp_write32(MCP_REG_C1FLTCON0, (1u << 7) | 1u);

    if (!mcp_request_mode(MCP_MODE_CAN20)) {
        return false;
    }

    return true;
}

static uint8_t can_len_from_dlc(uint8_t dlc)
{
    return dlc <= 8u ? dlc : 8u;
}

static void can_handle_rx_object(const uint8_t *obj)
{
    can_bus_frame_t frame = {0};
    uint32_t r0 = (uint32_t)obj[0]
                | ((uint32_t)obj[1] << 8)
                | ((uint32_t)obj[2] << 16)
                | ((uint32_t)obj[3] << 24);
    uint32_t r1 = (uint32_t)obj[4]
                | ((uint32_t)obj[5] << 8)
                | ((uint32_t)obj[6] << 16)
                | ((uint32_t)obj[7] << 24);
    uint32_t ts = (uint32_t)obj[8]
                | ((uint32_t)obj[9] << 8)
                | ((uint32_t)obj[10] << 16)
                | ((uint32_t)obj[11] << 24);

    frame.extended = ((r1 >> 4) & 0x1u) != 0u;
    frame.rtr = ((r1 >> 5) & 0x1u) != 0u;
    frame.dlc = can_len_from_dlc((uint8_t)(r1 & 0x0fu));
    frame.timestamp_us = ts;

    if (frame.extended) {
        uint32_t sid = r0 & 0x7ffu;
        uint32_t eid = (r0 >> 11) & 0x3ffffu;
        frame.id = (sid << 18) | eid;
    } else {
        frame.id = r0 & 0x7ffu;
    }

    if (!frame.rtr) {
        memcpy(frame.data, &obj[12], frame.dlc);
    }

    can_rx_total++;
    (void)can_push_frame(&frame);
}

static void can_bus_init_pins_once(void)
{
    if (can_io_initialized) {
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

    can_io_initialized = true;
}

void can_bus_init(void)
{
    can_bus_init_pins_once();

    can_head = 0u;
    can_tail = 0u;
    can_rx_total = 0u;
    can_tx_total = 0u;
    can_tx_attempt = 0u;
    can_overflow_count = 0u;
    can_error_count = 0u;
    tx_sequence = 0u;
    can_debug_c1con = 0u;
    can_debug_c1txqcon = 0u;
    can_debug_c1txqsta = 0u;
    can_debug_c1txqua = 0u;
    can_send_fail_reason = 0u;

    can_started = false;
    printf("CAN V1 idle until bitrate is configured over USB\n");
}

void can_bus_poll(void)
{
    if (!can_started) {
        return;
    }

    uint32_t processed = 0u;
    const uint32_t max_frames_per_poll = 4u;

    while (processed < max_frames_per_poll) {
        uint32_t fifo_status = mcp_read32(MCP_REG_C1FIFOSTA1);
        if ((fifo_status & MCP_FIFO_TFNRFNIF_BITS) == 0u) {
            break;
        }

        uint32_t fifo_ua = mcp_read32(MCP_REG_C1FIFOUA1);
        uint8_t obj[20];
        mcp_read_bytes((uint16_t)((MCP_RAM_START + fifo_ua) & 0x0fffu), obj, sizeof(obj));
        can_handle_rx_object(obj);
        mcp_write8(MCP_REG_C1FIFOCON1 + 1u, 1u);
        processed++;
    }
}

void can_bus_reset(void)
{
    can_head = 0u;
    can_tail = 0u;
    can_rx_total = 0u;
    can_tx_total = 0u;
    can_tx_attempt = 0u;
    can_overflow_count = 0u;
    can_error_count = 0u;
    tx_sequence = 0u;
    can_debug_c1con = 0u;
    can_debug_c1txqcon = 0u;
    can_debug_c1txqsta = 0u;
    can_debug_c1txqua = 0u;
    can_send_fail_reason = 0u;

    if (can_started) {
        can_started = mcp_init_controller();
    }
}

bool can_bus_set_bitrate(uint32_t bitrate)
{
    if (bitrate != CAN_DEFAULT_BITRATE) {
        return false;
    }

    can_bus_init_pins_once();
    current_bitrate = bitrate;
    can_started = mcp_init_controller();
    if (!can_started) {
        printf("CAN V1 init failed at %lu kbps\n", (unsigned long)(bitrate / 1000u));
    }
    return can_started;
}

uint32_t can_bus_get_bitrate(void)
{
    return current_bitrate;
}

bool can_bus_is_started(void)
{
    return can_started;
}

bool can_bus_pop_frame(can_bus_frame_t *frame)
{
    uint32_t tail = can_tail;

    if (tail == can_head) {
        return false;
    }

    if (frame) {
        *frame = can_frames[tail];
    }
    can_tail = (tail + 1u) % CAN_RX_RING_CAP;
    return true;
}

bool can_bus_send_frame(const can_bus_frame_t *frame)
{
    can_debug_c1con = mcp_read32(MCP_REG_C1CON);
    can_debug_c1txqcon = mcp_read32(MCP_REG_C1TXQCON);
    can_debug_c1txqsta = mcp_read32(MCP_REG_C1TXQSTA);
    can_debug_c1txqua = mcp_read32(MCP_REG_C1TXQUA);
    can_send_fail_reason = 0u;

    if (!can_started) {
        can_send_fail_reason = 1u;
        return false;
    }
    if (!frame) {
        can_send_fail_reason = 2u;
        return false;
    }
    if (frame->dlc > 8u) {
        can_send_fail_reason = 3u;
        return false;
    }
    if (((can_debug_c1con >> MCP_CON_OPMOD_SHIFT) & 0x7u) != MCP_MODE_CAN20) {
        can_send_fail_reason = 4u;
        return false;
    }
    uint32_t txq_ua = mcp_read32(MCP_REG_C1TXQUA);
    uint8_t obj[16] = {0};
    uint32_t t0;
    uint32_t t1;

    if (frame->extended) {
        uint32_t sid = (frame->id >> 18) & 0x7ffu;
        uint32_t eid = frame->id & 0x3ffffu;
        t0 = sid | (eid << 11);
    } else {
        t0 = frame->id & 0x7ffu;
    }

    t1 = ((tx_sequence++ & 0x7fffffu) << 9)
       | (frame->rtr ? (1u << 5) : 0u)
       | (frame->extended ? (1u << 4) : 0u)
       | (frame->dlc & 0x0fu);

    obj[0] = (uint8_t)(t0 & 0xffu);
    obj[1] = (uint8_t)((t0 >> 8) & 0xffu);
    obj[2] = (uint8_t)((t0 >> 16) & 0xffu);
    obj[3] = (uint8_t)((t0 >> 24) & 0xffu);
    obj[4] = (uint8_t)(t1 & 0xffu);
    obj[5] = (uint8_t)((t1 >> 8) & 0xffu);
    obj[6] = (uint8_t)((t1 >> 16) & 0xffu);
    obj[7] = (uint8_t)((t1 >> 24) & 0xffu);

    if (!frame->rtr && frame->dlc > 0u) {
        memcpy(&obj[8], frame->data, frame->dlc);
    }

    mcp_write_bytes((uint16_t)((MCP_RAM_START + txq_ua) & 0x0fffu), obj, sizeof(obj));
    mcp_write8(MCP_REG_C1TXQCON + 1u, (uint8_t)((MCP_FIFO_UINC_BITS | MCP_FIFO_TXREQ_BITS) >> 8));

    can_tx_attempt++;
    can_tx_total++;
    return true;
}

void can_bus_get_status(can_bus_status_t *status)
{
    if (!status) {
        return;
    }

    memset(status, 0, sizeof(*status));
    status->started = can_started;
    status->bitrate = current_bitrate;
    status->rx_total = can_rx_total;
    status->tx_total = can_tx_total;
    status->tx_attempt = can_tx_attempt;
    status->overflow_count = can_overflow_count;
    status->error_count = can_error_count;
    status->debug_c1con = can_debug_c1con;
    status->debug_c1txqcon = can_debug_c1txqcon;
    status->debug_c1txqsta = can_debug_c1txqsta;
    status->debug_c1txqua = can_debug_c1txqua;
    status->send_fail_reason = can_send_fail_reason;

    if (can_started) {
        uint32_t trec = mcp_read32(MCP_REG_C1TREC);
        status->rec = (uint8_t)(trec & 0xffu);
        status->tec = (uint8_t)((trec >> 8) & 0xffu);
        status->bus_off = (trec & MCP_TREC_TXBO_BITS) != 0u;
    }
}
