#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "pico/multicore.h"

#include "panda_usb.h"
#include "flexray_bss_streamer.h"
#include "flexray_forwarder_with_injector.h"


// --- Configuration ---

// -- Streamer Pins --
#define BGE_PIN 2
#define STBN_PIN 3

#define RELAY_FR_1_2 17
#define RELAY_FR_3_4 18

#define TXD_FR_1_PIN 28
#define TXEN_FR_1_PIN 27
#define RXD_FR_1_PIN 26

#define TXD_FR_2_PIN 4
#define TXEN_FR_2_PIN 5
#define RXD_FR_2_PIN 6

#define TXD_FR_3_PIN 10
#define TXEN_FR_3_PIN 9
#define RXD_FR_3_PIN 8

#define TXD_FR_4_PIN 16
#define TXEN_FR_4_PIN 22
#define RXD_FR_4_PIN 21

void core1_entry(void)
{
    setup_stream(pio0,
                 RXD_FR_1_PIN, TXEN_FR_2_PIN,
                 RXD_FR_2_PIN, TXEN_FR_1_PIN);

    while (1)
    {
        __wfi();
    }
}

void setup_pins(void)
{
    // disable transceiver
    gpio_init(BGE_PIN);
    gpio_set_dir(BGE_PIN, GPIO_OUT);
    gpio_put(BGE_PIN, 0);

    gpio_init(STBN_PIN);
    gpio_set_dir(STBN_PIN, GPIO_OUT);
    gpio_put(STBN_PIN, 0);

    gpio_pull_up(TXEN_FR_1_PIN);
    gpio_pull_up(TXEN_FR_2_PIN);
    gpio_pull_up(TXEN_FR_3_PIN);
    gpio_pull_up(TXEN_FR_4_PIN);

    // Keep all FlexRay TXD lines recessive before any TXEN streamer can enable
    // a transceiver. The PIO forwarder takes these pins over later.
    gpio_init(TXD_FR_1_PIN);
    gpio_set_dir(TXD_FR_1_PIN, GPIO_OUT);
    gpio_put(TXD_FR_1_PIN, 1);
    gpio_init(TXD_FR_2_PIN);
    gpio_set_dir(TXD_FR_2_PIN, GPIO_OUT);
    gpio_put(TXD_FR_2_PIN, 1);
    gpio_init(TXD_FR_3_PIN);
    gpio_set_dir(TXD_FR_3_PIN, GPIO_OUT);
    gpio_put(TXD_FR_3_PIN, 1);
    gpio_init(TXD_FR_4_PIN);
    gpio_set_dir(TXD_FR_4_PIN, GPIO_OUT);
    gpio_put(TXD_FR_4_PIN, 1);

    gpio_init(RXD_FR_1_PIN);
    gpio_set_dir(RXD_FR_1_PIN, GPIO_IN);
    gpio_init(RXD_FR_2_PIN);
    gpio_set_dir(RXD_FR_2_PIN, GPIO_IN);

    gpio_init(RXD_FR_3_PIN);
    gpio_set_dir(RXD_FR_3_PIN, GPIO_IN);
    gpio_init(RXD_FR_4_PIN);
    gpio_set_dir(RXD_FR_4_PIN, GPIO_IN);

    gpio_pull_up(RXD_FR_1_PIN);
    gpio_pull_up(RXD_FR_2_PIN);
    gpio_pull_up(RXD_FR_3_PIN);
    gpio_pull_up(RXD_FR_4_PIN);

    gpio_init(RELAY_FR_1_2);
    gpio_set_dir(RELAY_FR_1_2, GPIO_OUT);
    gpio_put(RELAY_FR_1_2, 1);
    sleep_ms(500);
    gpio_init(RELAY_FR_3_4);
    gpio_set_dir(RELAY_FR_3_4, GPIO_OUT);
    gpio_put(RELAY_FR_3_4, 0);

    // delay enabling pins to avoid glitch
    sleep_ms(100);

    // enable transceiver
    gpio_put(BGE_PIN, 1);
    gpio_put(STBN_PIN, 1);

}

int main(void)
{
    (void)set_sys_clock_khz(100000, true);
    panda_usb_init();
    notify_queue_init();
    setup_pins();

    // Forwarder must own TXD before core1 starts toggling TXEN.
    setup_forwarder_sas_only(pio2,
                             RXD_FR_1_PIN, TXD_FR_2_PIN,
                             RXD_FR_2_PIN, TXD_FR_1_PIN);

    multicore_launch_core1(core1_entry);

    while (true)
    {
        panda_usb_task();
        sleep_ms(1);
    }

    return 0;
}
