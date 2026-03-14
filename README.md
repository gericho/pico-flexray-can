# pico-flexray-can

Firmware branch for the Czok V1 board with simultaneous FlexRay and CAN logging.

This branch keeps the original FlexRay MITM path for the V1 hardware and adds support for the onboard MCP2518FD CAN controller and TCAN1042 transceiver. The goal is to let the board bridge and stream FlexRay while also exposing a 500 kbps classical CAN bus for BMW i3 PT-CAN style use cases.

## Summary of changes

- Targets the Czok V1 hardware, not the newer V2 layout.
- Keeps the FlexRay MITM forwarding and injection behavior used by the original V1 firmware.
- Adds support for the external MCP2518FD CAN controller connected to the RP2040/RP2350 over SPI.
- Streams CAN and FlexRay at the same time over USB.
- Uses a Panda-compatible CAN stream for CAN traffic.
- Keeps the original FlexRay source markers:
  - `src 0` = FlexRay ECU side
  - `src 1` = FlexRay vehicle side
- Moves CAN to a separate source ID:
  - `src 2` = external CAN bus

## USB behavior

The firmware enumerates as a single USB device with two vendor interfaces:

- Interface 0: Panda-style CAN RX/TX
- Interface 1: FlexRay RX/TX stream

This is intended to work with a patched host stack that can consume both interfaces from the same device.

## Hardware notes

For the V1 board:

- SAS side is the ECU side
- BDC side is the vehicle side
- The active FlexRay MITM pair used by this firmware is:
  - `CN4 / Flexray2` for SAS / ECU
  - `CN3 / Flexray1` for BDC / VEHICLE
- External CAN is exposed on `CN13 / CAN2`

## Build output

The firmware builds as the normal `pico_flexray` target and produces a UF2 image such as:

`build-v1can/pico_flexray.uf2`

## Credits

- CzokNorris: this project builds on CzokNorris's FlexRay reverse-engineering work and the V1 board design. Board reference: `https://oshwlab.com/czoknorris/v1board`
- Dynm: FlexRay firmware foundation and related pico-flexray work. Repository: `https://github.com/dynm/pico-flexray`
