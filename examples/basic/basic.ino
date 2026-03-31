#include <Arduino.h>
#include "PanavoxAC.h"

// W950 connector UART pins (ESP32-S3)
// TX: GPIO16, software-inverted + 2N2222 = net non-inverted to AC
// RX: GPIO17, hardware-inverted by level shifter, no software inversion
#define AC_TX_PIN 16
#define AC_RX_PIN 17

PanavoxAC ac(Serial2);

void setup() {
    Serial.begin(115200);

    Serial2.begin(9600, SERIAL_8N1, AC_RX_PIN, AC_TX_PIN, /*invert_tx=*/true);

    ac.onStatusUpdate([](const DeviceStatus& s) {
        Serial.printf("[STATUS] power=%d mode=%d fan=%d temp=%.1f/%.1f°C comp=%dHz\n",
            s.power,
            static_cast<int>(s.mode),
            static_cast<int>(s.fan_speed),
            s.target_temp_c,
            s.current_temp_c,
            s.compressor_freq);
    });

    ac.onError([](AcError err) {
        switch (err) {
        case AcError::TIMEOUT:           Serial.println("[ERROR] ACK timeout"); break;
        case AcError::CHECKSUM_MISMATCH: Serial.println("[ERROR] Checksum mismatch"); break;
        case AcError::INVALID_FRAME:     Serial.println("[ERROR] Invalid frame"); break;
        }
    });

    ac.begin();
}

void loop() {
    ac.loop();

    // Example: after 10 seconds, set mode to COOL at 22°C, fan AUTO
    static bool demo_done = false;
    if (!demo_done && millis() > 10000) {
        demo_done = true;
        ac.setPower(true);
        ac.setMode(AcMode::COOL);
        ac.setTargetTemp(22.0f);
        ac.setFanSpeed(FanSpeed::AUTO);
        ac.setSwing(SwingMode::VERTICAL);
    }
}
