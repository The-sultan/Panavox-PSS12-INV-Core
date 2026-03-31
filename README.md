# Panavox PSS-12 INV Core

A hardware-agnostic C++ driver library for controlling the **Panavox PSS-12 INV** split inverter air conditioner via its W950 WiFi module UART connector.

## What it does

This library handles the full serial protocol between a microcontroller and the AC indoor unit:

- Builds and sends command frames (power, mode, temperature, fan speed, swing, presets)
- Sends periodic status requests and parses the 150-byte status response frames
- Exposes decoded state via a callback (`onStatusUpdate`) and a clean `DeviceStatus` struct
- Manages the TX state machine (queueing, inter-message gaps, ACK handling)

The library takes a `Stream&` reference, making it compatible with any Arduino-based platform (ESP32, ESP8266, etc.) or any environment that provides a `Stream` interface.

## Hardware

Connect to the indoor unit's W950 WiFi module connector via UART at **9600 baud, 8N1**.

Note: the W950 connector TX/RX lines may require signal inversion depending on your hardware. On the reference ESP32-S3 implementation, GPIO TX uses software+hardware inversion (net: normal) and GPIO RX uses hardware inversion via a 2N2222 transistor.

## Usage (PlatformIO)

Add to your `platformio.ini`:

```ini
lib_deps =
    panavox-core=https://github.com/The-sultan/Panavox-PSS12-INV-Core.git
```

Basic example:

```cpp
#include <PanavoxAC.h>

PanavoxAC ac(Serial1);

void setup() {
    Serial1.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
    ac.onStatusUpdate([](const DeviceStatus& s) {
        // s.power, s.mode, s.target_temp_c, s.current_temp_c, etc.
    });
    ac.begin();
}

void loop() {
    ac.loop();
}
```

## ESPHome

This library is used as the protocol core by the [Panavox PSS-12 INV ESPHome component](#) *(link coming soon)*, which exposes a full `climate` entity in Home Assistant.

## Protocol

The serial protocol is documented in [`docs/Panavox_PSS-12_INV_Protocol_Specification.docx.pdf`](docs/Panavox_PSS-12_INV_Protocol_Specification.docx.pdf).

This is not an official manufacturer document. It was reverse-engineered by analysing the source code of the open-source ESPHome component for Hisense/Aircon International compatible units:

> https://github.com/pslawinski/esphome_airconintl

The protocol used by the Panavox PSS-12 INV via its W950 WiFi module connector appears to be compatible with that family of units. The specification was further refined through empirical testing on a physical unit, correcting several inaccuracies found in the original codebase and documenting behaviors not previously described.

## License

MIT
