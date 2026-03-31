#pragma once
#include <cstdint>
#include <cstddef>
#include "panavox_types.h"
#include "panavox_protocol.h"

// Raw packed struct that maps directly onto the AC status response payload.
// Byte offsets are relative to byte 16 of the response frame (start of payload).
// Must remain packed with no padding.
#pragma pack(push, 1)
struct DeviceStatusRaw {
    uint8_t wind_status;              // 0:  current fan speed
    uint8_t sleep_status;             // 1
    uint8_t direction_run_mode;       // 2:  [mode:4 | run:2 | direction:2]
    uint8_t indoor_temp_setting;      // 3:  target temp (Fahrenheit-based encoding)
    uint8_t indoor_temp_status;       // 4:  current indoor temp (Fahrenheit-based encoding)
    uint8_t indoor_pipe_temp;         // 5:  indoor coil temp (Fahrenheit-based encoding)
    int8_t  indoor_humidity_setting;  // 6:  humidity setpoint (%)
    int8_t  indoor_humidity_status;   // 7:  current humidity (%)
    uint8_t somatosensory_temp;       // 8:  feels-like temp
    uint8_t somatosensory_comp;       // 9:  [ctrl:3 | compensation:5]
    uint8_t fahrenheit_comp;          // 10: [fahrenheit:3 | compensation:5]
    uint8_t timer;                    // 11
    uint8_t hour;                     // 12
    uint8_t minute;                   // 13
    uint8_t poweron_hour;             // 14
    uint8_t poweron_minute;           // 15
    uint8_t poweroff_hour;            // 16
    uint8_t poweroff_minute;          // 17
    uint8_t wind_door_drying;         // 18: [wind_door:4 | drying:4]
    uint8_t feature_flags;            // 19: [up_down:1|left_right:1|nature:1|heat:1|low_power:1|low_elec:1|efficient:1|dual_freq:1]
    uint8_t feature_flags2;           // 20
    uint8_t led_filter_flags;         // 21
    uint8_t system_flags;             // 22
    uint8_t error_flags;              // 23
    uint8_t comm_flags;               // 24
    uint8_t compressor_freq;          // 25: current compressor Hz
    uint8_t compressor_freq_setting;  // 26: target compressor Hz
    uint8_t compressor_freq_send;     // 27: last sent Hz command
    int8_t  outdoor_temperature;      // 28: outdoor ambient (°C, signed)
    int8_t  outdoor_condenser_temp;   // 29: outdoor coil (°C, signed)
    int8_t  compressor_exhaust_temp;  // 30: compressor exhaust (°C, signed)
    int8_t  target_exhaust_temp;      // 31: target exhaust (°C, signed)
    uint8_t expand_threshold;         // 32: expansion valve threshold
    uint8_t voltage_current[12];      // 33-44: UAB, UBC, UCA, IAB, IBC, ICA, bus V, IUV
    uint8_t outdoor_flags;            // 45: [wind_machine:3 | outdoor_machine:1 | four_way:1]
};
#pragma pack(pop)

// Decoded, human-readable device state.
// This is what PanavoxAC exposes via the onStatusUpdate callback.
struct DeviceStatus {
    // Control state
    bool     power             = false;
    AcMode   mode              = AcMode::COOL;
    FanSpeed fan_speed         = FanSpeed::FAN_AUTO;
    float    target_temp_c     = 24.0f;
    float    current_temp_c    = 24.0f;
    bool     swing_vertical    = false;
    bool     swing_horizontal  = false;
    Preset   preset            = Preset::NONE;

    // Sensor data
    float    outdoor_temp_c       = 0.0f;
    float    condenser_temp_c     = 0.0f;
    float    exhaust_temp_c       = 0.0f;
    float    pipe_temp_c          = 0.0f;
    int8_t   humidity_setting     = 0;
    int8_t   humidity_status      = 0;
    uint8_t  compressor_freq      = 0;
    uint8_t  compressor_freq_setting = 0;
    bool     compressor_running   = false;

    // Diagnostic
    uint8_t  error_flags          = 0;
    uint8_t  comm_flags           = 0;
};

// Decode a raw status payload into a DeviceStatus.
// raw must point to at least sizeof(DeviceStatusRaw) bytes.
// Returns false if the decoded temperatures fail sanity checks.
bool decodeStatus(const uint8_t* raw, size_t len, DeviceStatus& out);

// Temperature bytes in the status response are direct Celsius (integer degrees).
inline float rawTempToCelsius(uint8_t raw) {
    return static_cast<float>(raw);
}

// Encode a Celsius temperature to command byte: byte = (2 * T) + 1
// T must be in [TEMP_CMD_MIN_C, TEMP_CMD_MAX_C].
inline uint8_t celsiusToTempByte(float celsius) {
    return static_cast<uint8_t>(2.0f * celsius + 1.0f);
}

// Map wind_status raw byte to FanSpeed enum.
FanSpeed decodeWindStatus(uint8_t wind_status);
