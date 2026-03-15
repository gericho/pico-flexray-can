# pico-flexray-can

Firmware for the Czok V1 board (`pico2_w`) with:

- FlexRay MITM forwarding for BMW i3 reverse engineering
- MCP2518FD-based classical CAN RX on `CN13`
- USB streaming for both FlexRay and CAN

This repo is the working V1 branch, not a generic multi-board firmware.

## Current Status

What works:

- FlexRay MITM forwarding is stable
- FlexRay USB streaming is stable
- MCP2518FD bring-up is stable at `500 kbps`
- CAN RX from the external bus is working
- CAN is exported over a second USB endpoint and can be logged by a patched `pandad`

What is intentionally not implemented:

- CAN TX to the vehicle bus
- generic CAN-FD support
- a stock upstream host stack without `pandad` changes

## Hardware

Target board:

- Czok V1
- `PICO_BOARD=pico2_w`

Relevant buses:

- FlexRay MITM:
  - `src 0` = ECU side
  - `src 1` = vehicle side
- CAN:
  - external MCP2518FD + TCAN1042 path
  - connector: `CN13`
  - intended use: BMW i3 `PT-CAN`
  - bitrate: `500 kbit/s`

Known V1 CAN path:

- controller: `MCP2518FD`
- transceiver: `TCAN1042`
- oscillator: `40 MHz`

## Connector Labels

Use the board silkscreen labels as follows:

- `CN4 / Flexray2`
  - connect to `SAS`
  - this is the FlexRay `ECU side`
  - logged as `src 0`

- `CN3 / Flexray1`
  - connect to the vehicle side / `BDC`
  - this is the FlexRay `vehicle side`
  - logged as `src 1`

- `CN13 / CAN2`
  - connect to BMW i3 `PT-CAN`
  - this is the external CAN path through `MCP2518FD + TCAN1042`
  - logged as `src 2` when the host `pandad` patch is present

In short:

- `CN4 / Flexray2 -> SAS / ECU side`
- `CN3 / Flexray1 -> BDC / vehicle side`
- `CN13 / CAN2 -> PT-CAN`

## USB Layout

The firmware enumerates as one USB device with two vendor IN streams:

- `0x81`: FlexRay stream
- `0x82`: Panda-style CAN packet stream

The firmware also supports software entry into BOOTSEL through the Panda vendor request:

- `PANDA_ENTER_BOOTLOADER_MODE = 0xD1`

This calls `reset_usb_boot(0, 0)` in firmware.

## Host Requirement

To log CAN and FlexRay together, the host must read both USB endpoints.

The matching host change is in `selfdrive/pandad`:

- `0x81` continues to carry the FlexRay stream
- `0x82` is read as Panda-style CAN packets

Without that host-side patch, CAN RX will work in the firmware but will not appear in routes.

## Key Firmware Notes

The final CAN RX fix was not in the bring-up itself, but in the MCP2518FD RX handling:

- read the RX object from `0x400 + FIFOUA`
- advance the RX FIFO with an `8-bit` write to `C1FIFOCON1 + 1`

This is the change that turned the CAN USB stream from malformed/repeated frames into valid Panda-style CAN packets.

## Build

Build output is always expected here:

`/home/gericho/pico-flexray-can/build-v1can/pico_flexray.uf2`

Typical rebuild:

```bash
cmake --build /home/gericho/pico-flexray-can/build-v1can --target clean
cmake --build /home/gericho/pico-flexray-can/build-v1can -j4
```

## Flash

Manual flash:

1. Put the Pico into BOOTSEL
2. Copy:
   - `build-v1can/pico_flexray.uf2`
   - to the `RP2350` mass-storage device

Software BOOTSEL is preferred when the board is mounted in the car.

Important operational note:

- after flashing, a full RP power cycle may be required before judging FlexRay forwarding behavior

## Verification

Firmware-only checks:

- `GET_CAN_HEALTH_STATS` should show:
  - `canSpeed = 500`
  - `totalRxCnt > 0`

End-to-end host checks:

- routes should contain:
  - `src 0` and `src 1` for FlexRay
  - `src 2` for CAN

## Branch Notes

Important recent commits:

- `e963a99` `Stabilize MCP2518FD CAN RX bring-up`
- `13cb6bd` `Fix MCP2518FD RX object address and FIFO advance`

## Credits

- CzokNorris for the V1 board and original reverse-engineering work
- dynm for the original `pico-flexray` firmware base
- Pierre Molinaro / `ACAN2517` for a useful MCP2517/2518 RX reference during debugging
