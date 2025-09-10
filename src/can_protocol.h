#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#include <Arduino.h>

// --- Device ID ---
// Define the unique ID for this controller on the CAN bus
#define CONTROLLER_NODE_ID 0

// --- CAN ID Structure (11-bit) ---

// Message Types (using the top 4 bits of the ID)
enum CanMessageType : uint8_t {
    LIGHTING_COMMAND = 0x1, // A command to control lights
    SENSOR_DATA      = 0x2, // A message containing sensor readings
    HEARTBEAT        = 0x8  // A status message from a node
};

// --- Payload Structure (8 Bytes) ---

// Command Types (for Byte 0 of a LIGHTING_COMMAND payload)
enum CanCommandType : uint8_t {
    SET_BRIGHTNESS = 0x10,
    SET_STATE      = 0x11
};


// --- Helper Functions to Build and Parse CAN IDs ---

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
 * Extracts the Message Type from a CAN ID.
 * @param canId The 11-bit CAN ID received from the bus.
 * @return The CanMessageType enum.
 */
inline CanMessageType getMessageType(uint32_t canId) {
    return (CanMessageType)((canId >> 7) & 0x0F);
}

/**
 * Extracts the Source Node ID from a CAN ID.
 * @param canId The 11-bit CAN ID received from the bus.
 * @return The ID of the node that sent the message.
 */
inline uint8_t getNodeId(uint32_t canId) {
    return (uint8_t)(canId & 0x7F);
}

#endif // CAN_PROTOCOL_H