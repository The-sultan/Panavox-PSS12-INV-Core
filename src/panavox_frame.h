#pragma once
#include <cstdint>
#include <vector>
#include <functional>
#include "panavox_protocol.h"

// Build a 50-byte command wire frame from a 30-byte payload.
// Applies byte stuffing and computes the 16-bit checksum.
std::vector<uint8_t> buildCommandFrame(const uint8_t payload[PanavoxProtocol::PAYLOAD_SIZE]);

// Returns the constant 21-byte status request frame.
std::vector<uint8_t> buildStatusRequestFrame();

// FrameParser: accumulates incoming bytes, handles F4-byte stuffing,
// validates the checksum, and fires a callback for each valid frame.
class FrameParser {
public:
    enum class FrameError { TOO_SHORT, SIZE_MISMATCH, CHECKSUM_MISMATCH };

    // Called with the complete logical (unstuffed) frame including delimiters.
    using FrameCallback = std::function<void(const uint8_t* frame, size_t len)>;
    using ErrorCallback = std::function<void(FrameError)>;
    using FrameStartCallback = std::function<void()>;

    void onFrame(FrameCallback cb)       { _frameCb      = cb; }
    void onError(ErrorCallback cb)       { _errorCb      = cb; }
    void onFrameStart(FrameStartCallback cb) { _frameStartCb = cb; }

    // Feed one byte from the UART receive buffer.
    void feed(uint8_t byte);

    // Reset the parser to the initial SEARCHING state.
    void reset();

private:
    enum class State { SEARCHING, SEEN_F4, IN_FRAME, IN_FRAME_SEEN_F4 };

    State _state = State::SEARCHING;
    std::vector<uint8_t> _buf;  // accumulates the logical (unstuffed) frame
    FrameCallback      _frameCb;
    ErrorCallback      _errorCb;
    FrameStartCallback _frameStartCb;

    void dispatch();
};
