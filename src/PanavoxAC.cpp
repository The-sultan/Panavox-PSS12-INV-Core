#include "PanavoxAC.h"
#include <cstring>
#include <algorithm>

using namespace PanavoxProtocol;

// ---- Constructor / begin / loop ----

PanavoxAC::PanavoxAC(Stream& serial) : _serial(serial) {
    _parser.onFrame([this](const uint8_t* frame, size_t len) {
        handleFrame(frame, len);
    });
    // Advance the TX state machine as soon as F4 F5 is detected — the AC sends
    // the frame header+body immediately but delays the checksum+F4FB by up to 10s
    // (its internal sampling cycle). We don't need to wait for F4 FB to unblock the queue.
    _parser.onFrameStart([this]() {
        if (_commState == CommState::WAITING_ACK) {
            if (!_queue.empty()) _queue.pop();
            _commState = CommState::IDLE;
            _lastPollTime = millis();
        }
    });
    _parser.onError([this](FrameParser::FrameError err) {
        if (!_errorCb) return;
        switch (err) {
        case FrameParser::FrameError::TOO_SHORT:          _errorCb(AcError::INVALID_FRAME);     break;
        case FrameParser::FrameError::SIZE_MISMATCH:      _errorCb(AcError::INVALID_FRAME);     break;
        case FrameParser::FrameError::CHECKSUM_MISMATCH:  _errorCb(AcError::CHECKSUM_MISMATCH); break;
        }
    });
}

void PanavoxAC::begin() {
    // Flush any stale bytes in the RX buffer
    while (_serial.available()) _serial.read();
    _parser.reset();
    _commState   = CommState::IDLE;
    _initialized = false;
    _lastSendTime = 0;
    _lastPollTime = 0;

    // Immediately send a status request to sync state with the AC
    enqueue(buildStatusRequestFrame());
}

void PanavoxAC::loop() {
    // Feed all available RX bytes to the frame parser
    while (_serial.available()) {
        _parser.feed(static_cast<uint8_t>(_serial.read()));
    }

    // Drive the TX state machine
    processTx();

    // Schedule periodic status polling.
    // Runs unconditionally so the initial sync is retried if the first request times out,
    // and so HA reflects the real AC state immediately after every flash/reboot.
    uint32_t now = millis();
    if (_queue.empty()
        && _commState == CommState::IDLE
        && (now - _lastPollTime) >= POLL_INTERVAL_MS)
    {
        enqueue(buildStatusRequestFrame());
        _lastPollTime = now;
    }
}

// ---- Control setters ----

void PanavoxAC::setPower(bool on) {
    if (on == _desired.power) return;
    _desired.power = on;

    if (on) {
        enqueueFullStateOnPowerOn();
    } else {
        uint8_t p[PAYLOAD_SIZE] = {};
        payloadPowerOff(p);
        enqueue(buildCommandFrame(p));
    }
}

void PanavoxAC::setMode(AcMode mode) {
    _desired.mode = mode;

    if (_desired.power) {
        uint8_t p[PAYLOAD_SIZE] = {};
        payloadMode(mode, _desired.target_temp, _desired.fan_speed, p);
        enqueue(buildCommandFrame(p));
    }
}

void PanavoxAC::setTargetTemp(float celsius) {
    celsius = std::max(TEMP_CMD_MIN_C, std::min(TEMP_CMD_MAX_C, celsius));
    _desired.target_temp = celsius;

    if (_desired.power) {
        uint8_t p[PAYLOAD_SIZE] = {};
        payloadTemp(celsius, p);
        enqueue(buildCommandFrame(p));
    }
}

void PanavoxAC::setFanSpeed(FanSpeed speed) {
    _desired.fan_speed = speed;

    if (_desired.power) {
        uint8_t p[PAYLOAD_SIZE] = {};
        payloadFan(speed, p);
        enqueue(buildCommandFrame(p));
    }
}

void PanavoxAC::setSwing(SwingMode swing) {
    _desired.swing = swing;

    if (_desired.power) {
        bool v = (swing == SwingMode::VERTICAL || swing == SwingMode::BOTH);
        bool h = (swing == SwingMode::HORIZONTAL || swing == SwingMode::BOTH);

        uint8_t pv[PAYLOAD_SIZE] = {};
        payloadSwingVertical(v, pv);
        enqueue(buildCommandFrame(pv));

        uint8_t ph[PAYLOAD_SIZE] = {};
        payloadSwingHorizontal(h, ph);
        enqueue(buildCommandFrame(ph));
    }
}

void PanavoxAC::setPreset(Preset preset) {
    Preset prev = _desired.preset;
    _desired.preset = preset;

    if (!_desired.power) return;

    uint8_t p[PAYLOAD_SIZE] = {};

    // Clear previous preset if needed
    if (prev == Preset::TURBO && preset != Preset::TURBO) {
        payloadPresetTurbo(false, p);
        enqueue(buildCommandFrame(p));
        memset(p, 0, PAYLOAD_SIZE);
    }
    if (prev == Preset::ECO && preset != Preset::ECO) {
        payloadPresetEco(false, p);
        enqueue(buildCommandFrame(p));
        memset(p, 0, PAYLOAD_SIZE);
    }

    // Activate new preset
    if (preset == Preset::TURBO) {
        payloadPresetTurbo(true, p);
        enqueue(buildCommandFrame(p));
    } else if (preset == Preset::ECO) {
        payloadPresetEco(true, p);
        enqueue(buildCommandFrame(p));
    }
}

void PanavoxAC::setDisplay(bool on) {
    if (_desired.display == on) return;
    _desired.display = on;

    uint8_t p[PAYLOAD_SIZE] = {};
    payloadDisplay(on, p);
    enqueue(buildCommandFrame(p));
}

// ---- Internal TX state machine ----

void PanavoxAC::enqueue(std::vector<uint8_t> frame) {
    _queue.push(std::move(frame));
}

void PanavoxAC::processTx() {
    uint32_t now = millis();

    // Every sent frame expects a full status response (spec §3.6).
    // Stay in WAITING_ACK until the response arrives or we time out.
    if (_commState == CommState::WAITING_ACK) {
        if ((now - _lastSendTime) >= ACK_TIMEOUT_MS) {
            if (!_queue.empty()) _queue.pop();
            _commState = CommState::IDLE;
            _lastPollTime = now;  // prevent immediate re-poll while AC may still be mid-frame
            if (_errorCb) _errorCb(AcError::TIMEOUT);
        }
        return;
    }

    if (_queue.empty()) return;

    // Enforce post-send idle gap between consecutive frames
    uint32_t elapsed = now - _lastSendTime;
    uint32_t required = (_lastSendTime == 0) ? 0 : POST_SEND_IDLE_MS;
    if (elapsed < required) return;
    if (elapsed < INTER_MSG_GAP_MS) return;

    const std::vector<uint8_t>& frame = _queue.front();
    _serial.write(frame.data(), frame.size());
    _lastSendTime = now;
    _commState = CommState::WAITING_ACK;
}

// ---- RX frame handling ----

void PanavoxAC::handleFrame(const uint8_t* frame, size_t len) {
    // We only care about status response frames (direction AC→controller, type 0x66)
    if (len < 14) return;
    if (frame[2] != DIR_AC_TO_CTRL) return;
    if (frame[13] != MSG_STATUS_RSP) return;

    // Response received — pop the pending frame from the queue and go IDLE
    if (_commState == CommState::WAITING_ACK) {
        if (!_queue.empty()) _queue.pop();
        _commState = CommState::IDLE;
    }

    // Reset the poll timer on any received status (requested or spontaneous)
    // so we don't immediately poll again when the AC is already sending data.
    _lastPollTime = millis();

    processStatusResponse(frame, len);
}

void PanavoxAC::processStatusResponse(const uint8_t* frame, size_t len) {
    // Payload starts at byte 16; we need at least sizeof(DeviceStatusRaw) bytes
    if (len < HEADER_SIZE + sizeof(DeviceStatusRaw)) return;

    const uint8_t* payload = frame + HEADER_SIZE;
    size_t payload_len = len - HEADER_SIZE - 4; // subtract header + checksum(2) + footer(2)

    DeviceStatus decoded = _status; // preserve fields not decoded from status (e.g. preset)

    if (!decodeStatus(payload, payload_len, decoded)) return;

    // Preserve preset from desired_state (status response has no clear preset flags)
    decoded.preset = _desired.preset;

    _status = decoded;

    // On first successful response, sync desired_state to AC state
    if (!_initialized) {
        _initialized = true;
        _desired.power       = _status.power;
        _desired.mode        = _status.mode;
        _desired.target_temp = _status.target_temp_c;
        _desired.fan_speed   = _status.fan_speed;  // already a FanSpeed enum
        _desired.swing       = (_status.swing_vertical && _status.swing_horizontal) ? SwingMode::BOTH
                             : _status.swing_vertical   ? SwingMode::VERTICAL
                             : _status.swing_horizontal ? SwingMode::HORIZONTAL
                                                        : SwingMode::OFF;
        _desired.preset      = Preset::NONE;
    }

    if (_statusCb) _statusCb(_status);
}

// ---- Payload builders ----
// All functions write into a pre-zeroed 30-byte array.
// Byte positions are payload-relative (P_n = frame byte 16+n).

void PanavoxAC::payloadPowerOn(uint8_t out[]) const {
    // P2=0x0C, P7=0x04
    out[2] = 0x0C;
    out[7] = CTRL_NORMAL;
}

void PanavoxAC::payloadPowerOff(uint8_t out[]) const {
    // From raw frame: P1=01, P2=04, P7=04, P8=01, P13=01, P16=01, P17=55, P19=01
    out[1]  = 0x01;
    out[2]  = 0x04;
    out[7]  = CTRL_NORMAL;
    out[8]  = 0x01;
    out[13] = 0x01;
    out[16] = 0x01;
    out[17] = 0x55;
    out[19] = 0x01;
}

void PanavoxAC::payloadMode(AcMode mode, float temp_c, FanSpeed fan, uint8_t out[]) const {
    // Mode commands include "change confirmation" flags at P15, P16, P17, P19.
    // P0: fan code (for FAN_ONLY mode the fan speed is encoded here; others use 0x01)
    // P1: 0x01 (active)
    // P2: mode bitfield
    // P3: temperature (2*T+1)
    // P7: control class
    // P15,P16,P17,P19: change confirmation flags (per captured frames)

    static const uint8_t fanCmd[] = {
        FAN_CMD_AUTO, FAN_CMD_QUIET, FAN_CMD_LOW, FAN_CMD_MEDIUM, FAN_CMD_HIGH
    };

    // For FAN_ONLY, the fan speed goes into P0; for all others P0 = 0x01
    if (mode == AcMode::FAN_ONLY) {
        out[0] = fanCmd[static_cast<uint8_t>(fan) - static_cast<uint8_t>(FanSpeed::FAN_AUTO)];
    } else {
        out[0] = 0x01;
    }
    out[1] = 0x01;

    switch (mode) {
    case AcMode::COOL:     out[2] = MODE_BITS_COOL;     break;
    case AcMode::HEAT:     out[2] = MODE_BITS_HEAT;     break;
    case AcMode::FAN_ONLY: out[2] = MODE_BITS_FAN_ONLY; break;
    case AcMode::DRY:      out[2] = MODE_BITS_DRY;      break;
    }

    temp_c = std::max(TEMP_CMD_MIN_C, std::min(TEMP_CMD_MAX_C, temp_c));
    out[3] = celsiusToTempByte(temp_c);
    out[7] = CTRL_NORMAL;

    // Change confirmation flags (observed in all captured mode frames)
    out[15] = 0x10;
    out[16] = 0x01;
    out[17] = 0x04;
    out[19] = 0x10;
}

void PanavoxAC::payloadTemp(float celsius, uint8_t out[]) const {
    // Temperature-only: minimal payload — only P3 and P7
    celsius = std::max(TEMP_CMD_MIN_C, std::min(TEMP_CMD_MAX_C, celsius));
    out[3] = celsiusToTempByte(celsius);
    out[7] = CTRL_NORMAL;
}

void PanavoxAC::payloadFan(FanSpeed speed, uint8_t out[]) const {
    // P0: fan code, P1: 0x01, P7: ctrl, P17: 0x04, P19: 0x10 (0x30 for QUIET)
    static const uint8_t fanCmd[] = {
        FAN_CMD_AUTO, FAN_CMD_QUIET, FAN_CMD_LOW, FAN_CMD_MEDIUM, FAN_CMD_HIGH
    };
    out[0]  = fanCmd[static_cast<uint8_t>(speed)];
    out[1]  = 0x01;
    out[7]  = CTRL_NORMAL;
    out[17] = 0x04;
    out[19] = (speed == FanSpeed::FAN_QUIET) ? 0x30 : 0x10;
}

void PanavoxAC::payloadSwingVertical(bool on, uint8_t out[]) const {
    // P7=0x04, P15=0x01, P16: 0xC0=on, 0x40=off
    out[7]  = CTRL_NORMAL;
    out[15] = 0x01;
    out[16] = on ? 0xC0 : 0x40;
}

void PanavoxAC::payloadSwingHorizontal(bool on, uint8_t out[]) const {
    // P7=0x04, P16: 0x30=on, 0x10=off, P21=0x14
    out[7]  = CTRL_NORMAL;
    out[16] = on ? 0x30 : 0x10;
    out[21] = 0x14;
}

void PanavoxAC::payloadPresetTurbo(bool on, uint8_t out[]) const {
    // P0=0x01, P1=0x01, P7=0x04, P17: 0x5C=on / 0x54=off, P19=0x10
    out[0]  = 0x01;
    out[1]  = 0x01;
    out[7]  = CTRL_NORMAL;
    out[17] = on ? 0x5C : 0x54;
    out[19] = 0x10;
}

void PanavoxAC::payloadPresetEco(bool on, uint8_t out[]) const {
    // P7=0x04, P17: 0x34=on / 0x14=off, P19=0x00
    out[7]  = CTRL_NORMAL;
    out[17] = on ? 0x34 : 0x14;
    // P19 = 0x00 (already zero)
}

void PanavoxAC::payloadDisplay(bool on, uint8_t out[]) const {
    // P7=0x04, P20: 0xC0=on / 0x40=off
    out[7]  = CTRL_NORMAL;
    out[20] = on ? 0xC0 : 0x40;
}

// ---- Full state restore on power-on ----

void PanavoxAC::payloadPowerOnWithFullState(AcMode mode, float temp_c, FanSpeed fan,
                                             bool swing_v, bool swing_h, uint8_t out[]) const {
    // Combines power-on + mode + fan + temperature + swing into a single frame.
    //
    // P2: MODE_BITS use bits 4-6 (0x10/0x30/0x50/0x70),
    //     MODE_BITS_POWER_ON uses bits 2-3 (0x0C) — no overlap, safe to OR.
    // P16: vertical swing uses bits 7-6, horizontal swing uses bits 5-4 — independent.
    // P15: mode-change flag (0x10) | swing-change flag (0x01) = 0x11.

    static const uint8_t fanCmd[] = {
        FAN_CMD_AUTO, FAN_CMD_QUIET, FAN_CMD_LOW, FAN_CMD_MEDIUM, FAN_CMD_HIGH
    };

    out[0] = fanCmd[static_cast<uint8_t>(fan)];
    out[1] = 0x01;

    switch (mode) {
    case AcMode::COOL:     out[2] = MODE_BITS_COOL     | MODE_BITS_POWER_ON; break;
    case AcMode::HEAT:     out[2] = MODE_BITS_HEAT     | MODE_BITS_POWER_ON; break;
    case AcMode::FAN_ONLY: out[2] = MODE_BITS_FAN_ONLY | MODE_BITS_POWER_ON; break;
    case AcMode::DRY:      out[2] = MODE_BITS_DRY      | MODE_BITS_POWER_ON; break;
    }

    temp_c = std::max(TEMP_CMD_MIN_C, std::min(TEMP_CMD_MAX_C, temp_c));
    out[3] = celsiusToTempByte(temp_c);
    out[7] = CTRL_NORMAL;

    out[15] = 0x11;  // mode-change flag (0x10) | swing-change flag (0x01)
    out[16] = (swing_v ? 0xC0 : 0x40) | (swing_h ? 0x30 : 0x10);
    out[17] = 0x04;
    out[19] = (fan == FanSpeed::FAN_QUIET) ? 0x30 : 0x10;
    out[21] = 0x14;  // horizontal swing change flag
}

void PanavoxAC::enqueueFullStateOnPowerOn() {
    bool v = (_desired.swing == SwingMode::VERTICAL || _desired.swing == SwingMode::BOTH);
    bool h = (_desired.swing == SwingMode::HORIZONTAL || _desired.swing == SwingMode::BOTH);

    uint8_t p[PAYLOAD_SIZE] = {};
    payloadPowerOnWithFullState(_desired.mode, _desired.target_temp, _desired.fan_speed, v, h, p);
    enqueue(buildCommandFrame(p));

    // Preset requires its own frame (separate control byte, can't be merged)
    if (_desired.preset == Preset::TURBO) {
        memset(p, 0, PAYLOAD_SIZE);
        payloadPresetTurbo(true, p);
        enqueue(buildCommandFrame(p));
    } else if (_desired.preset == Preset::ECO) {
        memset(p, 0, PAYLOAD_SIZE);
        payloadPresetEco(true, p);
        enqueue(buildCommandFrame(p));
    }
}
