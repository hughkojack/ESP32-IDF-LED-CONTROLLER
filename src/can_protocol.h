#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

// NOTE: This project uses event-based input semantics on CAN (CLICK/HOLD/etc).

#include <cstdint>
#include <cstring>
#include <string>

// CAN protocol helpers for this project.
//
// IMPORTANT:
// - The bus can carry frames with flags (EFF/RTR/ERR) in the top bits of can_id.
// - This project’s protocol uses **standard 11-bit data frames** only.

// --- Device ID ---
// Define the unique ID for this controller on the CAN bus
#define CONTROLLER_NODE_ID 0

// --- CAN ID Structure (11-bit) ---

// Message Types (using the top 4 bits of the ID)
enum CanMessageType : uint8_t {
    // Input events from wall switches / nodes.
    INPUT_EVENT      = 0x1,
    // Legacy name kept because parts of the codebase still use this symbol.
    LIGHTING_COMMAND = INPUT_EVENT,
    SENSOR_DATA      = 0x2, // A message containing sensor readings
    NODE_CONFIG      = 0x3, // Hub -> node: configuration (set node ID, find-me, etc.)
    NODE_STATE_FEEDBACK = 0x4, // Hub -> node: per-button brightness 0-100 or 0xFF (payload 4 bytes)
    HEARTBEAT        = 0x8  // Node -> hub: announce/heartbeat (node_id, type, input_count)
};

// Unconfigured node ID (127): hub detects new nodes; valid configured IDs 1..126
#define NODE_ID_UNCONFIGURED 127

// Config sub-commands (payload byte 0 when NODE_CONFIG, hub -> node)
#define CMD_SET_NODE_ID         0x01
#define CMD_SET_INPUT_CFG       0x02  // payload after cmd: [input_index][input_id][mode][gpio][active_high?]. DLC 4 legacy (idx,id,mode); DLC 5 +gpio; DLC 6 +active_high (0=active low default, 1=active high)
#define CMD_SET_INPUT_COUNT     0x03
#define CMD_SET_TIMING          0x04
#define CMD_FIND_ME             0x05
#define CMD_SET_FIND_ME_OUTPUT  0x06
#define CMD_SET_INPUT_LABEL     0x07
#define CMD_SET_DATETIME        0x08  // payload: Unix timestamp (uint32_t LE, bytes 1-4); hub sends every hour
#define CMD_REBOOT              0x09  // no payload; node restarts
#define CMD_SET_CAN_LINK_INDICATOR 0x0A  // payload: [gpio]; GPIO 0-48 for CAN link LED (solid=good link, flash=bad/no link), 0xFF=disable
#define CMD_SET_TIMEZONE          0x0B  // payload: multi-frame like CMD_SET_INPUT_LABEL: byte 1 = total_len (first) or 0xFF (cont.), bytes 2-7 = TZ string (6 chars/frame); hub sends before CMD_SET_DATETIME so LCD can use localtime()
#define CMD_SET_NIGHT_LIGHT       0x0C  // payload: [enabled 0/1, brightness 0-100]; mechanical WS2812 night light

// Node types
#define NODE_TYPE_LCD        1
#define NODE_TYPE_MECHANICAL 2

// --- Payload Structure (8 Bytes) ---

// Command Types (for Byte 0 of a LIGHTING_COMMAND payload)
enum CanCommandType : uint8_t {
    SET_BRIGHTNESS = 0x10,
    SET_STATE      = 0x11
};


// --- Helper Functions to Build and Parse CAN IDs ---

// Linux-style CAN flags that may be present in `can_frame.can_id`.
static constexpr uint32_t CAN_ID_EFF_FLAG = 0x80000000UL; // extended frame format
static constexpr uint32_t CAN_ID_RTR_FLAG = 0x40000000UL; // remote transmission request
static constexpr uint32_t CAN_ID_ERR_FLAG = 0x20000000UL; // error frame
static constexpr uint32_t CAN_ID_SFF_MASK = 0x000007FFUL; // 11-bit identifier mask

// --- Input Event Codes (event-based; detected by node firmware) ---
// Payload:
//   byte0 = input/button number (1..255)
//   byte1 = event code (below)
//   For EVT_DIM: byte2 = brightness 0..100 (optional bytes 3–4 = fade_ms).
enum CanInputEventCode : uint8_t {
    EVT_CLICK        = 0x01,
    EVT_HOLD         = 0x02,
    EVT_DOUBLE_CLICK = 0x03,

    // Extra useful events (optional for node firmware):
    EVT_TRIPLE_CLICK = 0x04,
    EVT_LONG_HOLD    = 0x05,
    EVT_HOLD_REPEAT  = 0x06, // e.g. send periodically while held for dimming/step actions
    EVT_DIM          = 0x07, // byte2 = brightness 0..100; match same binding as CLICK for (node_id, button)
};

inline bool isEventActionString(const std::string& s) {
    return s == "CLICK" || s == "HOLD" || s == "DOUBLE_CLICK" ||
           s == "TRIPLE_CLICK" || s == "LONG_HOLD" || s == "HOLD_REPEAT" || s == "DIM";
}

// Decode the "trigger action" string used for binding matching.
// Returns a pointer to a string literal (stable for lifetime of program).
inline const char* decodeEventAction(uint8_t evt) {
    switch (evt) {
        case EVT_CLICK:        return "CLICK";
        case EVT_HOLD:         return "HOLD";
        case EVT_DOUBLE_CLICK: return "DOUBLE_CLICK";
        case EVT_TRIPLE_CLICK: return "TRIPLE_CLICK";
        case EVT_LONG_HOLD:    return "LONG_HOLD";
        case EVT_HOLD_REPEAT:  return "HOLD_REPEAT";
        case EVT_DIM:          return "DIM";
        default:               return "UNKNOWN";
    }
}

/**
 * Creates an 11-bit CAN ID from a message type and node ID.
 * @param msgType The purpose of the message.
 * @param nodeId The ID of the device sending the message.
 * @return The formatted 11-bit CAN ID.
 */
inline uint16_t createCanId(CanMessageType msgType, uint8_t nodeId) {
    // 4 bits for message type, 7 bits for node ID
    return ( (uint16_t)msgType << 7 ) | (nodeId & 0x7F);
}

/**
 * True if `can_id` represents a standard 11-bit DATA frame (no EFF/RTR/ERR).
 */
inline bool isStandardDataFrame(uint32_t canId) {
    return (canId & (CAN_ID_EFF_FLAG | CAN_ID_RTR_FLAG | CAN_ID_ERR_FLAG)) == 0;
}

/**
 * Extract the standard 11-bit ID portion from `can_id` (flags masked out).
 * Only meaningful if `isStandardDataFrame(canId)` is true.
 */
inline uint16_t getStandardId(uint32_t canId) {
    return (uint16_t)(canId & CAN_ID_SFF_MASK);
}

/**
 * Extracts the Message Type from a CAN ID.
 * @param canId The 11-bit CAN ID received from the bus.
 * @return The CanMessageType enum.
 */
inline CanMessageType getMessageType(uint32_t canId) {
    const uint16_t sid = getStandardId(canId);
    return (CanMessageType)((sid >> 7) & 0x0F);
}

/**
 * Extracts the Source Node ID from a CAN ID.
 * @param canId The 11-bit CAN ID received from the bus.
 * @return The ID of the node that sent the message.
 */
inline uint8_t getNodeId(uint32_t canId) {
    const uint16_t sid = getStandardId(canId);
    return (uint8_t)(sid & 0x7F);
}

#endif // CAN_PROTOCOL_H