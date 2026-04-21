#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "pico/multicore.h"

#include "panda_usb.h"
#include "flexray_bss_streamer.h"
#include "flexray_forwarder_with_injector.h"
#include "flexray_board_pins.h"


// --- Configuration ---

#define FLEXRAY_MODE_SAS_ONLY 1
#define FLEXRAY_MODE_EPS_ONLY 2
#define FLEXRAY_MODE_DUAL     3

#ifndef FLEXRAY_FORWARD_MODE
#define FLEXRAY_FORWARD_MODE FLEXRAY_MODE_DUAL
#endif

void core1_entry(void)
{
#if FLEXRAY_FORWARD_MODE == FLEXRAY_MODE_SAS_ONLY || FLEXRAY_FORWARD_MODE == FLEXRAY_MODE_DUAL
    setup_stream(pio0,
                 RXD_FR_1_PIN, TXEN_FR_2_PIN,
                 RXD_FR_2_PIN, TXEN_FR_1_PIN);
#endif

#if FLEXRAY_FORWARD_MODE == FLEXRAY_MODE_EPS_ONLY || FLEXRAY_FORWARD_MODE == FLEXRAY_MODE_DUAL
    setup_stream_fr34(pio1,
                      RXD_FR_3_PIN, TXEN_FR_4_PIN,
                      RXD_FR_4_PIN, TXEN_FR_3_PIN);
#endif

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

#if FLEXRAY_FORWARD_MODE != FLEXRAY_MODE_EPS_ONLY
    gpio_pull_up(TXEN_FR_1_PIN);
    gpio_pull_up(TXEN_FR_2_PIN);
#endif
#if FLEXRAY_FORWARD_MODE != FLEXRAY_MODE_SAS_ONLY
    gpio_pull_up(TXEN_FR_3_PIN);
    gpio_pull_up(TXEN_FR_4_PIN);
#endif

    // Keep all FlexRay TXD lines recessive before any TXEN streamer can enable
    // a transceiver. The PIO forwarder takes these pins over later.
#if FLEXRAY_FORWARD_MODE != FLEXRAY_MODE_EPS_ONLY
    gpio_init(TXD_FR_1_PIN);
    gpio_set_dir(TXD_FR_1_PIN, GPIO_OUT);
    gpio_put(TXD_FR_1_PIN, 1);
    gpio_init(TXD_FR_2_PIN);
    gpio_set_dir(TXD_FR_2_PIN, GPIO_OUT);
    gpio_put(TXD_FR_2_PIN, 1);
#else
    gpio_init(TXD_FR_1_PIN);
    gpio_set_dir(TXD_FR_1_PIN, GPIO_IN);
    gpio_disable_pulls(TXD_FR_1_PIN);
    gpio_init(TXD_FR_2_PIN);
    gpio_set_dir(TXD_FR_2_PIN, GPIO_IN);
    gpio_disable_pulls(TXD_FR_2_PIN);
    gpio_init(TXEN_FR_1_PIN);
    gpio_set_dir(TXEN_FR_1_PIN, GPIO_IN);
    gpio_disable_pulls(TXEN_FR_1_PIN);
    gpio_init(TXEN_FR_2_PIN);
    gpio_set_dir(TXEN_FR_2_PIN, GPIO_IN);
    gpio_disable_pulls(TXEN_FR_2_PIN);
#endif
#if FLEXRAY_FORWARD_MODE != FLEXRAY_MODE_SAS_ONLY
    gpio_init(TXD_FR_3_PIN);
    gpio_set_dir(TXD_FR_3_PIN, GPIO_OUT);
    gpio_put(TXD_FR_3_PIN, 1);
    gpio_init(TXD_FR_4_PIN);
    gpio_set_dir(TXD_FR_4_PIN, GPIO_OUT);
    gpio_put(TXD_FR_4_PIN, 1);
#else
    gpio_init(TXD_FR_3_PIN);
    gpio_set_dir(TXD_FR_3_PIN, GPIO_IN);
    gpio_disable_pulls(TXD_FR_3_PIN);
    gpio_init(TXD_FR_4_PIN);
    gpio_set_dir(TXD_FR_4_PIN, GPIO_IN);
    gpio_disable_pulls(TXD_FR_4_PIN);
    gpio_init(TXEN_FR_3_PIN);
    gpio_set_dir(TXEN_FR_3_PIN, GPIO_IN);
    gpio_disable_pulls(TXEN_FR_3_PIN);
    gpio_init(TXEN_FR_4_PIN);
    gpio_set_dir(TXEN_FR_4_PIN, GPIO_IN);
    gpio_disable_pulls(TXEN_FR_4_PIN);
#endif

#if FLEXRAY_FORWARD_MODE != FLEXRAY_MODE_EPS_ONLY
    gpio_init(RXD_FR_1_PIN);
    gpio_set_dir(RXD_FR_1_PIN, GPIO_IN);
    gpio_init(RXD_FR_2_PIN);
    gpio_set_dir(RXD_FR_2_PIN, GPIO_IN);
    gpio_pull_up(RXD_FR_1_PIN);
    gpio_pull_up(RXD_FR_2_PIN);
#else
    gpio_init(RXD_FR_1_PIN);
    gpio_set_dir(RXD_FR_1_PIN, GPIO_IN);
    gpio_disable_pulls(RXD_FR_1_PIN);
    gpio_init(RXD_FR_2_PIN);
    gpio_set_dir(RXD_FR_2_PIN, GPIO_IN);
    gpio_disable_pulls(RXD_FR_2_PIN);
#endif

#if FLEXRAY_FORWARD_MODE != FLEXRAY_MODE_SAS_ONLY
    gpio_init(RXD_FR_3_PIN);
    gpio_set_dir(RXD_FR_3_PIN, GPIO_IN);
    gpio_init(RXD_FR_4_PIN);
    gpio_set_dir(RXD_FR_4_PIN, GPIO_IN);
    gpio_pull_up(RXD_FR_3_PIN);
    gpio_pull_up(RXD_FR_4_PIN);
#else
    gpio_init(RXD_FR_3_PIN);
    gpio_set_dir(RXD_FR_3_PIN, GPIO_IN);
    gpio_disable_pulls(RXD_FR_3_PIN);
    gpio_init(RXD_FR_4_PIN);
    gpio_set_dir(RXD_FR_4_PIN, GPIO_IN);
    gpio_disable_pulls(RXD_FR_4_PIN);
#endif

    gpio_init(RELAY_FR_1_2);
    gpio_set_dir(RELAY_FR_1_2, GPIO_OUT);
#if FLEXRAY_FORWARD_MODE == FLEXRAY_MODE_EPS_ONLY
    gpio_put(RELAY_FR_1_2, 0);
#else
    gpio_put(RELAY_FR_1_2, 0);
#endif
    sleep_ms(500);
    gpio_init(RELAY_FR_3_4);
    gpio_set_dir(RELAY_FR_3_4, GPIO_OUT);
#if FLEXRAY_FORWARD_MODE == FLEXRAY_MODE_SAS_ONLY
    gpio_put(RELAY_FR_3_4, 1);
#else
    gpio_put(RELAY_FR_3_4, 1);
#endif

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
#if FLEXRAY_FORWARD_MODE == FLEXRAY_MODE_SAS_ONLY
    setup_forwarder_sas_only(pio2,
                             RXD_FR_1_PIN, TXD_FR_2_PIN,
                             RXD_FR_2_PIN, TXD_FR_1_PIN);
#elif FLEXRAY_FORWARD_MODE == FLEXRAY_MODE_EPS_ONLY
    setup_forwarder_eps_only(pio2,
                             RXD_FR_3_PIN, TXD_FR_4_PIN,
                             RXD_FR_4_PIN, TXD_FR_3_PIN);
#else
    setup_forwarder_with_injector(pio2,
                                  RXD_FR_1_PIN, TXD_FR_2_PIN,
                                  RXD_FR_2_PIN, TXD_FR_1_PIN,
                                  RXD_FR_3_PIN, TXD_FR_4_PIN,
                                  RXD_FR_4_PIN, TXD_FR_3_PIN);
#endif

    multicore_launch_core1(core1_entry);

    while (true)
    {
        uint32_t encoded;
        while (notify_queue_pop(&encoded)) {
            streamer_process_notify(encoded);
        }
        panda_usb_task();
        sleep_ms(1);
    }

    return 0;
}
