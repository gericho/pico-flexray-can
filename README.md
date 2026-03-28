# pico-flexray-can

## Branch Summary

Branch:

- `Czok-V1-CAN-mimic`

Purpose:

- keep a dedicated firmware branch for BMW i3 injector-style work
- separate future i3-specific FlexRay mimic/injection changes from the stable `Czok-V1-CAN` base

What has been done on this branch so far:

- branch created from `Czok-V1-CAN`
- firmware rebuilt from this branch
- firmware flashed successfully to the RP2350/Pico in BOOTSEL mode

Why this branch exists:

- the current injector path in firmware is still hardcoded to the legacy SP2018-style rule set
- upcoming work needs a safe branch to move toward BMW i3-specific `long` and `lateral` mimic rules
- this avoids mixing experimental injector changes with the known-good base branch

Current state:

- firmware injector rules are now being moved away from the old single SP2018 rule
- first i3-specific mimic targets are:
  - `54`
  - `59`
  - `72`
  - `96`
- this branch remains the intended working area for validating those injector changes before they are considered for the base branch

Firmware for the Czok V1 board (`pico2_w`) with:

- single-channel FlexRay MITM forwarding for BMW i3 reverse engineering
- MCP2518FD-based classical CAN on `CN13`
- USB streaming for both FlexRay and CAN
- USB host->CAN injection support for controlled replay/debug

This repo is the working V1 branch, not a generic multi-board firmware.

## Current Status

What works:

- single-channel FlexRay MITM forwarding is stable
- FlexRay USB streaming is stable
- MCP2518FD bring-up is stable at `500 kbps`
- CAN RX from the external bus is working
- CAN TX from the host to the external bus is working
- CAN is exported over a second USB interface/endpoint pair and can be logged by a patched `pandad`
- host tools can replay CAN and FlexRay in parallel through the same Pico device

Current limitation:

- FlexRay forwarding is single-channel only for now
- the vehicle-side acceptance gate for BMW i3 lateral replay is still under investigation
- replaying valid raw frames is not yet sufficient, by itself, to guarantee real steering response on the vehicle

What is intentionally not implemented:

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

The CAN path also exposes a host->device vendor OUT endpoint:

- `0x04`: Panda-style CAN packet write endpoint

Practical meaning:

- FlexRay host transport remains on interface `0`
- CAN host transport uses interface `1`
- the same Pico can now be used for:
  - FlexRay streaming
  - CAN logging
  - host-driven CAN replay/injection

The firmware also supports software entry into BOOTSEL through the Panda vendor request:

- `PANDA_ENTER_BOOTLOADER_MODE = 0xD1`

This calls `reset_usb_boot(0, 0)` in firmware.

## Host Requirement

To log CAN and FlexRay together, the host must read both USB endpoints.

The matching host change is in `selfdrive/pandad`:

- `0x81` continues to carry the FlexRay stream
- `0x82` is read as Panda-style CAN packets

GitHub location of the matching `pandad`:

- repository: matching `pandad` fork with Pico dual-endpoint support
- branch: `Czok-V1-can`
- path inside the host tree:
  - `selfdrive/pandad`

Without that host-side patch, CAN RX will work in the firmware but will not appear in routes.

## Key Firmware Notes

The current working CAN path depends on linking the real MCP2518FD driver, not the old stub:

- normal builds must use `src/can_bus.c`
- `LINK_PICO_CAN_STUB=OFF` is the expected setting for the V1 CAN firmware

Important MCP2518FD RX handling details:

- read the RX object from `0x400 + FIFOUA`
- advance the RX FIFO with an `8-bit` write to `C1FIFOCON1 + 1`
- poll RX FIFO state directly, rather than relying only on the interrupt pin level

Important MCP2518FD TX handling details:

- write TX objects into message RAM at `0x400 + TXQUA`
- trigger TX with an `8-bit` write to `C1TXQCON + 1`
- keep the controller in classic CAN mode (`CAN20`) for the BMW i3 PT-CAN path

These changes are what turned the V1 CAN path into a real bidirectional USB<->CAN transport instead of a partial RX-only bring-up.

## Build

Build output is always expected here:

`build-v1can/pico_flexray.uf2`

Typical rebuild:

```bash
cmake --build build-v1can --target clean
cmake --build build-v1can -j4
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
  - `totalTxCnt > 0` after a host-driven CAN transmit test

End-to-end host checks:

- routes should contain:
  - `src 0` and `src 1` for FlexRay
  - `src 2` for CAN

Direct host transport checks:

- host CAN RX probe should show:
  - live reads on endpoint `0x82`
  - `totalRxCnt` increasing
- host CAN TX probe should show:
  - `totalTxCnt` increasing
  - `txFail = 0` in the exposed debug counters

## Branch Notes

Important recent commits:

- `e963a99` `Stabilize MCP2518FD CAN RX bring-up`
- `13cb6bd` `Fix MCP2518FD RX object address and FIFO advance`
- `b5e8929` `can: wire real MCP2518 driver and USB host transport`

## Credits

- CzokNorris for the V1 board and original reverse-engineering work
- dynm for the original `pico-flexray` firmware base
- Pierre Molinaro / `ACAN2517` for a useful MCP2517/2518 RX reference during debugging
