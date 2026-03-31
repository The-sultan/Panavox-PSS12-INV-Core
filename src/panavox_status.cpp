#include "panavox_status.h"
#include <cstring>

using namespace PanavoxProtocol;

FanSpeed decodeWindStatus(uint8_t ws) {
    switch (ws) {
    case STATUS_FAN_QUIET:  return FanSpeed::FAN_QUIET;
    case STATUS_FAN_LOW:    return FanSpeed::FAN_LOW;
    case STATUS_FAN_MEDIUM: return FanSpeed::FAN_MEDIUM;
    case STATUS_FAN_HIGH:   return FanSpeed::FAN_HIGH;
    default:                return FanSpeed::FAN_AUTO;
    }
}

bool decodeStatus(const uint8_t* raw, size_t len, DeviceStatus& out) {
    if (len < sizeof(DeviceStatusRaw)) return false;

    DeviceStatusRaw s;
    memcpy(&s, raw, sizeof(DeviceStatusRaw));

    // --- Power and mode ---
    uint8_t run_status  = (s.direction_run_mode >> 2) & 0x03;
    uint8_t mode_status = (s.direction_run_mode >> 4) & 0x0F;

    out.power = (run_status != 0);

    switch (mode_status) {
    case STATUS_MODE_FAN_ONLY: out.mode = AcMode::FAN_ONLY; break;
    case STATUS_MODE_HEAT:     out.mode = AcMode::HEAT;     break;
    case STATUS_MODE_COOL:     out.mode = AcMode::COOL;     break;
    case STATUS_MODE_DRY:      out.mode = AcMode::DRY;      break;
    default:                   out.mode = AcMode::COOL;     break;
    }

    // --- Fan speed ---
    out.fan_speed = decodeWindStatus(s.wind_status);

    // --- Temperatures (raw byte = direct Celsius integer, with sanity bounds) ---
    float tgt = rawTempToCelsius(s.indoor_temp_setting);
    if (tgt > TEMP_SANITY_TGT_MIN && tgt < TEMP_SANITY_TGT_MAX) {
        out.target_temp_c = tgt;
    }

    float cur = rawTempToCelsius(s.indoor_temp_status);
    if (cur > TEMP_SANITY_CUR_MIN && cur < TEMP_SANITY_CUR_MAX) {
        out.current_temp_c = cur;
    }

    float pipe = rawTempToCelsius(s.indoor_pipe_temp);
    if (pipe > TEMP_SANITY_CUR_MIN && pipe < TEMP_SANITY_CUR_MAX) {
        out.pipe_temp_c = pipe;
    }

    // --- Outdoor / compressor temperatures (raw int8 already in °C) ---
    out.outdoor_temp_c    = static_cast<float>(s.outdoor_temperature);
    out.condenser_temp_c  = static_cast<float>(s.outdoor_condenser_temp);
    out.exhaust_temp_c    = static_cast<float>(s.compressor_exhaust_temp);

    // --- Swing (feature_flags byte, offset 19) ---
    out.swing_vertical   = (s.feature_flags & (1 << 7)) != 0;
    out.swing_horizontal = (s.feature_flags & (1 << 6)) != 0;

    // --- Preset ---
    // Note: turbo/eco flags are not clearly documented in the status response.
    // Preserve the preset from desired_state; PanavoxAC handles this.
    // We only reset preset to NONE if power is off.
    if (!out.power) out.preset = Preset::NONE;

    // --- Humidity ---
    out.humidity_setting = s.indoor_humidity_setting;
    out.humidity_status  = s.indoor_humidity_status;

    // --- Compressor ---
    out.compressor_freq         = s.compressor_freq;
    out.compressor_freq_setting = s.compressor_freq_setting;
    out.compressor_running      = (s.compressor_freq > 0);

    // --- Diagnostics ---
    out.error_flags = s.error_flags;
    out.comm_flags  = s.comm_flags;

    return true;
}
