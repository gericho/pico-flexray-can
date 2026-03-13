[![Ask DeepWiki](https://deepwiki.com/badge.svg)](https://deepwiki.com/dynm/pico-flexray)

### pico-flexray-can

## Credits

- Czok for the new V2 board hardware work and board bring-up details.
- dynm for the original [`pico-flexray`](https://github.com/dynm/pico-flexray) firmware repository this project builds on.

## Experimental Status

- Added OLED display support.
- Added firmware-driven watchdog heartbeat on `GPIO20`.
- Set `FR1/FR2` relay control to `GPIO25` and drive it high at boot.
- Set `FR3/FR4` relay control to `GPIO24` and drive it high at boot.
- Proper CAN support is still incomplete.
- This firmware should still be considered experimental and currently `BROKEN`.

## Experimental CAN RX-Only

Experimental Classical CAN RX-only @ 500 kbps.

- Added an experimental CAN RX-only software sniffer for the Czok V2 board.
- The current implementation is fixed at `500 kbps`.
- Current board mapping used by the sniffer:
- `CAN RX` on `GPIO46`
- `CAN STB` on `GPIO45`
- `CAN relay` on `GPIO41`
- The sniffer uses `core1` for software sampling and frame decoding.
- It attempts to decode standard and extended CAN frames, including bit de-stuffing and CRC validation.
- Decoded frames are forwarded to `core0` and printed on UART for early bring-up and validation.
- OLED status now includes basic CAN RX counters.
- This is an early bring-up implementation and has not yet been validated on real hardware.
- It should be considered experimental and may still miss frames or decode invalid data.

A Raspberry Pi Pico-based FlexRay man-in-the-middle (MITM) bridge that forwards frames between ECU and vehicle transceivers, with optional test replay output and a Panda-compatible USB interface.

- Core features:
  - Continuous, bidirectional FlexRay frame forwarding (vehicle ↔ ECU)
  - Optional replay/test output via a dedicated GPIO
  - USB interface is Panda-compatible
  - FlexRay MITM Done

### Build and flash

```bash
git clone https://github.com/dynm/pico-flexray/
cd pico-flexray
```

Option 1: Visual Studio Code
1. Install the [Raspberry Pi Pico extension](https://marketplace.visualstudio.com/items?itemName=raspberry-pi.raspberry-pi-pico)
2. Open this repo, do not enable RISC-V instructions
3. Click the Pico extension tab on the left panel
4. Click "Switch Board" and select your Pico board
5. Hold BOOT, plug USB, then you can release BOOT
6. Click "Run Project (USB)"
7. Done!

Option 2: Command line
Prerequisites:
- Raspberry Pi Pico SDK 2.1.x (env var `PICO_SDK_PATH` or the VS Code Pico extension auto-setup)
- `picotool` for flashing, or UF2 drag-and-drop

Configure and build (default board is set in `CMakeLists.txt` to `pico2`):

```bash
cd pico-flexray
mkdir build && cd build
ninja -C build
```

Artifacts are produced in `build/` (e.g., `pico_flexray.uf2`, `pico_flexray.elf`).

Flash to device:
- UF2: Hold BOOT, plug USB, then copy `build/pico_flexray.uf2` to the RPI-RP2 mass storage device.
- Picotool: put the board in BOOTSEL or use reset-to-boot, then:

```bash
picotool load -f build/pico_flexray.uf2
```

Run-time:
- USB enumerates as a vendor-specific device (no CDC serial). Use UART for logs.
- On boot, the app prints pin assignments and status, enables transceivers, and starts forwarding.

### Adjusting pins or board

If you use a different board or wiring, update the GPIO defines at the top of `src/main.c` and/or modify set(PICO_BOARD pico2 CACHE STRING "Board type") in CMakeLists.txt. Rebuild and reflash.

### Streaming with Cabana

To visualize FlexRay data using Cabana:

1. Clone the OpenPilot repository and switch to the FlexRay-enabled branch:
   ```bash
   git clone https://github.com/dynm/openpilot
   cd openpilot
   git checkout cabana-flexray
   ```

2. Set up the environment:
   ```bash
   ./tools/op.sh setup
   ```

3. Build Cabana:
   ```bash
   source .venv/bin/activate
   scons -j$(nproc) tools/cabana/cabana
   ```

4. Launch Cabana:
   ```bash
   ./tools/cabana/cabana
   ```

5. Testing:
   If you want to develop without transceivers and are using only a single Pico board,
   connect REPLAY_TX to RXD_FROM_ECU or RXD_FROM_VEHICLE with a jumper wire.
   You will then see frames in Cabana.
