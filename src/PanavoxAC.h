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

    DeviceStatus   _status;
    AcDesiredState _desired;

    std::function<void(const DeviceStatus&)> _statusCb;
    std::function<void(AcError)>             _errorCb;

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
};
