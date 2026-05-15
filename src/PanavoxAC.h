#pragma once
#include <Arduino.h>
#include <functional>
#include <queue>
#include <vector>
#include "panavox_types.h"
#include "panavox_status.h"
#include "panavox_frame.h"

// Desired state: what we have asked the AC to be.
// Used to re-send the full state on power-on and to track pending changes.
struct AcDesiredState {
    bool      power        = false;
    AcMode    mode         = AcMode::COOL;
    float     target_temp  = 24.0f;   // Celsius
    FanSpeed  fan_speed    = FanSpeed::FAN_AUTO;
    SwingMode swing        = SwingMode::OFF;
    Preset    preset       = Preset::NONE;
    bool      display      = true;
};

class PanavoxAC {
public:
    explicit PanavoxAC(Stream& serial);

    // Call once from setup().
    void begin();

    // Call every loop() iteration. Drives the TX state machine and reads RX bytes.
    void loop();

    // --- Control ---
    // All setters update desired_state and queue the necessary frames.
    void setPower(bool on);
    void setMode(AcMode mode);
    void setTargetTemp(float celsius);
    void setFanSpeed(FanSpeed speed);
    void setSwing(SwingMode swing);
    void setPreset(Preset preset);
    void setDisplay(bool on);

    // --- Callbacks ---
    void onStatusUpdate(std::function<void(const DeviceStatus&)> cb) { _statusCb = cb; }
    void onError(std::function<void(AcError)> cb)                    { _errorCb  = cb; }
    void onPolarityMismatch(std::function<void()> cb)                { _polarityMismatchCb = cb; }
    void onWiringIssue(std::function<void()> cb)                     { _wiringIssueCb      = cb; }

    // --- Accessors ---
    const DeviceStatus&    getStatus()       const { return _status; }
    const AcDesiredState&  getDesiredState() const { return _desired; }

private:
    enum class CommState { IDLE, WAITING_ACK };

    Stream&   _serial;
    CommState _commState  = CommState::IDLE;

    FrameParser _parser;
    std::queue<std::vector<uint8_t>> _queue;

    uint32_t _lastSendTime  = 0;  // millis() when last frame was sent
    uint32_t _lastPollTime  = 0;  // millis() when last status request was sent
    bool     _initialized   = false; // true after first valid status response

    bool     _pendingTempCmd  = false; // temperature command waiting for debounce
    uint32_t _tempCmdDue      = 0;    // millis() when pending temp command should fire
    bool     _parserInFrame   = false; // true while the parser is inside a valid F4F5 frame

    // --- Diagnostic detection ---
    static constexpr size_t   DIAG_BUF_LIMIT        = 64;
    static constexpr size_t   DIAG_TRIGGER_THRESHOLD = 20;
    static constexpr uint32_t DIAG_WARN_INTERVAL_MS  = 30000;

    std::vector<uint8_t> _diagBuf;
    bool     _polarityWarnReady  = true;
    bool     _wiringWarnReady    = true;
    uint32_t _lastPolarityWarnMs = 0;
    uint32_t _lastWiringWarnMs   = 0;

    DeviceStatus   _status;
    AcDesiredState _desired;

    std::function<void(const DeviceStatus&)> _statusCb;
    std::function<void(AcError)>             _errorCb;
    std::function<void()>                    _polarityMismatchCb;
    std::function<void()>                    _wiringIssueCb;

    // Internal: queue a frame. Every frame expects a status response from the AC.
    void enqueue(std::vector<uint8_t> frame);

    // Internal: send the next frame from the queue if timing allows
    void processTx();

    // Internal: handle a complete validated frame from the parser
    void handleFrame(const uint8_t* frame, size_t len);

    // Internal: parse status payload and update _status, then sync desired_state
    void processStatusResponse(const uint8_t* frame, size_t len);

    // --- Payload builders (return a 30-byte payload array via out[]) ---
    void payloadPowerOn(uint8_t out[]) const;
    void payloadPowerOff(uint8_t out[]) const;
    void payloadMode(AcMode mode, float temp_c, FanSpeed fan, uint8_t out[]) const;
    void payloadTemp(float celsius, uint8_t out[]) const;
    void payloadFan(FanSpeed speed, uint8_t out[]) const;
    void payloadSwingVertical(bool on, uint8_t out[]) const;
    void payloadSwingHorizontal(bool on, uint8_t out[]) const;
    void payloadPresetTurbo(bool on, uint8_t out[]) const;
    void payloadPresetEco(bool on, uint8_t out[]) const;
    void payloadDisplay(bool on, uint8_t out[]) const;

    // Single-frame payload: power-on + mode + temp + fan + swing combined.
    // MODE_BITS (bits 4-6) and MODE_BITS_POWER_ON (bits 2-3) share P2 without overlap.
    void payloadPowerOnWithFullState(AcMode mode, float temp_c, FanSpeed fan,
                                     bool swing_v, bool swing_h, uint8_t out[]) const;

    // Enqueue all frames required to bring the AC to the desired state on power-on
    void enqueueFullStateOnPowerOn();

    // Inspect _diagBuf for polarity-mismatch signature or unexpected bytes,
    // fire the appropriate rate-limited callback, then clear the buffer.
    void checkDiagnostics();
};
