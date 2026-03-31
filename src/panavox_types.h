#pragma once
#include <cstdint>

enum class AcMode : uint8_t {
    FAN_ONLY = 0,
    HEAT     = 1,
    COOL     = 2,
    DRY      = 3,
};

enum class FanSpeed : uint8_t {
    FAN_AUTO   = 0,
    FAN_QUIET  = 1,
    FAN_LOW    = 2,
    FAN_MEDIUM = 3,
    FAN_HIGH   = 4,
};

enum class SwingMode : uint8_t {
    OFF        = 0,
    VERTICAL   = 1,
    HORIZONTAL = 2,
    BOTH       = 3,
};

enum class Preset : uint8_t {
    NONE  = 0,
    TURBO = 1,
    ECO   = 2,
};

enum class AcError : uint8_t {
    TIMEOUT,           // No response received within ACK_TIMEOUT_MS
    CHECKSUM_MISMATCH, // Frame checksum did not match computed value
    INVALID_FRAME,     // Frame size or structure invalid
};
