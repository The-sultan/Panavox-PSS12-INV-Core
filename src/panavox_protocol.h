#pragma once
#include <cstdint>

// Protocol constants for the Panavox PSS-12 INV W950 serial interface.
// All values derived from spec v1.0. No logic here — only constants.
namespace PanavoxProtocol {

    // --- Framing ---
    constexpr uint8_t FRAME_ESCAPE = 0xF4;
    constexpr uint8_t FRAME_START2 = 0xF5;  // F4 F5 = start delimiter
    constexpr uint8_t FRAME_END2   = 0xFB;  // F4 FB = end delimiter

    // --- Header fixed bytes ---
    constexpr uint8_t DIR_CTRL_TO_AC = 0x00;
    constexpr uint8_t DIR_AC_TO_CTRL = 0x01;
    constexpr uint8_t FIXED_ID       = 0x40;

    // Source/destination address bytes
    constexpr uint8_t ADDR_CTRL_HI = 0x01;
    constexpr uint8_t ADDR_CTRL_LO = 0x01;
    constexpr uint8_t ADDR_AC_HI   = 0xFE;
    constexpr uint8_t ADDR_AC_LO   = 0x01;

    // --- Message types (byte 13 in frame) ---
    constexpr uint8_t MSG_COMMAND    = 0x65;
    constexpr uint8_t MSG_STATUS_REQ = 0x66;
    constexpr uint8_t MSG_STATUS_RSP = 0x66;

    // --- Payload lengths (byte 4 in frame) ---
    constexpr uint8_t LEN_COMMAND    = 0x29;  // 41 bytes
    constexpr uint8_t LEN_STATUS_REQ = 0x0C;  // 12 bytes
    constexpr uint8_t LEN_STATUS_RSP = 0x8D;  // 141 bytes

    // --- Frame sizes (pre-stuffing / logical sizes) ---
    constexpr uint8_t  CMD_FRAME_SIZE    = 50;
    constexpr uint8_t  STATUS_REQ_SIZE   = 21;
    constexpr uint16_t STATUS_RSP_SIZE   = 150;

    // --- Payload parameters ---
    constexpr uint8_t HEADER_SIZE   = 16;  // bytes before payload in command frame
    constexpr uint8_t PAYLOAD_SIZE  = 30;  // command payload bytes

    // --- Timing (milliseconds) ---
    constexpr uint32_t INTER_MSG_GAP_MS  = 100;
    constexpr uint32_t POST_SEND_IDLE_MS = 1500;
    constexpr uint32_t ACK_TIMEOUT_MS    = 5000;
    constexpr uint32_t POLL_INTERVAL_MS  = 5000;

    // --- Control class values (payload byte P7 = frame byte 23) ---
    constexpr uint8_t CTRL_NORMAL    = 0x04;
    constexpr uint8_t CTRL_CELSIUS   = 0x05;
    constexpr uint8_t CTRL_FAHRENHEIT = 0x07;

    // --- Fan speed command codes (payload byte P0 = frame byte 16) ---
    constexpr uint8_t FAN_CMD_AUTO   = 0x01;
    constexpr uint8_t FAN_CMD_QUIET  = 0x03;
    constexpr uint8_t FAN_CMD_LOW    = 0x05;
    constexpr uint8_t FAN_CMD_MEDIUM = 0x07;
    constexpr uint8_t FAN_CMD_HIGH   = 0x09;

    // --- Mode bitfield values (payload byte P2 = frame byte 18) ---
    constexpr uint8_t MODE_BITS_COOL     = 0x50;
    constexpr uint8_t MODE_BITS_HEAT     = 0x30;
    constexpr uint8_t MODE_BITS_FAN_ONLY = 0x10;
    constexpr uint8_t MODE_BITS_DRY      = 0x70;
    constexpr uint8_t MODE_BITS_POWER_ON = 0x0C;

    // --- Status response: mode_status values (bits 7-4 of status byte 2) ---
    constexpr uint8_t STATUS_MODE_FAN_ONLY = 0;
    constexpr uint8_t STATUS_MODE_HEAT     = 1;
    constexpr uint8_t STATUS_MODE_COOL     = 2;
    constexpr uint8_t STATUS_MODE_DRY      = 3;

    // --- Status response: wind_status values ---
    constexpr uint8_t STATUS_FAN_AUTO   = 0;
    constexpr uint8_t STATUS_FAN_QUIET  = 2;
    constexpr uint8_t STATUS_FAN_LOW    = 10;
    constexpr uint8_t STATUS_FAN_MEDIUM = 14;
    constexpr uint8_t STATUS_FAN_HIGH   = 18;

    // --- Temperature bounds (Celsius) ---
    constexpr float TEMP_CMD_MIN_C      = 16.0f;
    constexpr float TEMP_CMD_MAX_C      = 32.0f;
    constexpr float TEMP_SANITY_TGT_MIN = 7.0f;
    constexpr float TEMP_SANITY_TGT_MAX = 33.0f;
    constexpr float TEMP_SANITY_CUR_MIN = 1.0f;
    constexpr float TEMP_SANITY_CUR_MAX = 49.0f;

    // Default temperatures included in mode frames (Celsius)
    constexpr float TEMP_DEFAULT_COOL = 26.5f;
    constexpr float TEMP_DEFAULT_HEAT = 23.5f;
    constexpr float TEMP_DEFAULT_FAN  = 25.5f;
    constexpr float TEMP_DEFAULT_DRY  = 25.5f;

} // namespace PanavoxProtocol
