#include "panavox_frame.h"
#include <cstring>

using namespace PanavoxProtocol;

// ---- Command frame header (bytes 0-15 of the logical frame) ----
static const uint8_t CMD_HEADER[HEADER_SIZE] = {
    FRAME_ESCAPE, FRAME_START2,  // [0-1]  F4 F5 — frame start
    DIR_CTRL_TO_AC,              // [2]    00    — direction
    FIXED_ID,                    // [3]    40    — fixed identifier
    LEN_COMMAND,                 // [4]    29    — payload length
    0x00, 0x00,                  // [5-6]  reserved
    ADDR_CTRL_HI, ADDR_CTRL_LO, // [7-8]  01 01 — source: controller
    ADDR_AC_HI,   ADDR_AC_LO,   // [9-10] FE 01 — destination: AC
    0x00, 0x00,                  // [11-12] reserved
    MSG_COMMAND,                 // [13]   65    — message type: command
    0x00, 0x00,                  // [14-15] reserved
};

// Constant status request frame (pre-built, no payload bytes to stuff).
static const uint8_t STATUS_REQ_BYTES[STATUS_REQ_SIZE] = {
    0xF4, 0xF5, 0x00, 0x40, 0x0C, 0x00, 0x00,
    0x01, 0x01, 0xFE, 0x01, 0x00, 0x00, 0x66,
    0x00, 0x00, 0x00, 0x01, 0xB3, 0xF4, 0xFB
};

// Append a byte to the output, doubling it if it equals FRAME_ESCAPE (0xF4).
static inline void appendStuffed(std::vector<uint8_t>& out, uint8_t b) {
    out.push_back(b);
    if (b == FRAME_ESCAPE) out.push_back(FRAME_ESCAPE);
}

std::vector<uint8_t> buildCommandFrame(const uint8_t payload[PAYLOAD_SIZE]) {
    // Step 1: Assemble the logical (pre-stuffing) frame.
    //   [0-1]   F4 F5      — start delimiter
    //   [2-15]  header     — direction, id, length, addresses, msg type
    //   [16-45] payload    — 30 command bytes
    //   [46-47] checksum   — 16-bit big-endian
    //   [48-49] F4 FB      — end delimiter
    uint8_t logical[CMD_FRAME_SIZE];
    memcpy(logical, CMD_HEADER, HEADER_SIZE);
    memcpy(&logical[HEADER_SIZE], payload, PAYLOAD_SIZE);

    // Step 2: Compute checksum over logical[2..45] (spec §3.3).
    // msg_size = LEN_COMMAND + 9 = 50, so checksum range = [2 .. 50-5] = [2..45].
    uint16_t cs = 0;
    for (uint8_t i = 2; i <= 45; i++) cs += logical[i];
    logical[46] = static_cast<uint8_t>(cs >> 8);
    logical[47] = static_cast<uint8_t>(cs & 0xFF);
    logical[48] = FRAME_ESCAPE;
    logical[49] = FRAME_END2;

    // Step 3: Build wire frame.
    //   Start delimiter is emitted as-is (never stuffed).
    //   Content bytes [2..47] (header content + payload + checksum) are stuffed.
    //   End delimiter is emitted as-is.
    std::vector<uint8_t> wire;
    wire.reserve(CMD_FRAME_SIZE + 4);  // small slack for stuffed bytes
    wire.push_back(FRAME_ESCAPE);
    wire.push_back(FRAME_START2);
    for (uint8_t i = 2; i <= 47; i++) appendStuffed(wire, logical[i]);
    wire.push_back(FRAME_ESCAPE);
    wire.push_back(FRAME_END2);
    return wire;
}

std::vector<uint8_t> buildStatusRequestFrame() {
    return std::vector<uint8_t>(STATUS_REQ_BYTES, STATUS_REQ_BYTES + STATUS_REQ_SIZE);
}

// ---- FrameParser ----

void FrameParser::reset() {
    _state = State::SEARCHING;
    _buf.clear();
}

void FrameParser::feed(uint8_t b) {
    switch (_state) {

    case State::SEARCHING:
        if (b == FRAME_ESCAPE) _state = State::SEEN_F4;
        break;

    case State::SEEN_F4:
        if (b == FRAME_START2) {
            // Valid frame start: F4 F5
            _buf.clear();
            _buf.push_back(FRAME_ESCAPE);
            _buf.push_back(FRAME_START2);
            _state = State::IN_FRAME;
            if (_frameStartCb) _frameStartCb();
        } else if (b == FRAME_ESCAPE) {
            // Two consecutive F4 outside a frame — could be noise, keep waiting
            // Stay in SEEN_F4 so the second F4 can be the start of a real F4 F5
        } else {
            _state = State::SEARCHING;
        }
        break;

    case State::IN_FRAME:
        if (b == FRAME_ESCAPE) {
            _state = State::IN_FRAME_SEEN_F4;
        } else {
            _buf.push_back(b);
        }
        break;

    case State::IN_FRAME_SEEN_F4:
        if (b == FRAME_ESCAPE) {
            // Stuffed F4: F4 F4 → single F4 data byte
            _buf.push_back(FRAME_ESCAPE);
            _state = State::IN_FRAME;
        } else if (b == FRAME_END2) {
            // End of frame: F4 FB
            _buf.push_back(FRAME_ESCAPE);
            _buf.push_back(FRAME_END2);
            dispatch();
            _state = State::SEARCHING;
            _buf.clear();
        } else if (b == FRAME_START2) {
            // Unexpected new frame start — discard current and restart
            _buf.clear();
            _buf.push_back(FRAME_ESCAPE);
            _buf.push_back(FRAME_START2);
            _state = State::IN_FRAME;
        } else {
            // Invalid sequence — resync
            _buf.clear();
            _state = State::SEARCHING;
        }
        break;
    }
}

void FrameParser::dispatch() {
    const size_t len = _buf.size();

    if (len < 9) {
        if (_errorCb) _errorCb(FrameError::TOO_SHORT);
        return;
    }

    // Length byte is at index 4; logical frame size = length + 9 (spec §3.3)
    uint8_t length_byte = _buf[4];
    size_t expected_size = static_cast<size_t>(length_byte) + 9;

    if (len != expected_size) {
        if (_errorCb) _errorCb(FrameError::SIZE_MISMATCH);
        return;
    }

    // Checksum covers buf[2 .. expected_size-5] (spec §3.3)
    uint16_t cs = 0;
    for (size_t i = 2; i <= expected_size - 5; i++) cs += _buf[i];

    uint16_t frame_cs = (static_cast<uint16_t>(_buf[len - 4]) << 8)
                       | static_cast<uint16_t>(_buf[len - 3]);

    if (cs != frame_cs) {
        if (_errorCb) _errorCb(FrameError::CHECKSUM_MISMATCH);
        return;
    }

    if (_frameCb) _frameCb(_buf.data(), len);
}
