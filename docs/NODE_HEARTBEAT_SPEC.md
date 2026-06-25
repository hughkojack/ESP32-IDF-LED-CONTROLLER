# Node HEARTBEAT Message Specification

This document specifies the HEARTBEAT message format that nodes (like SwitchUI) must send for the hub to discover and track them.

## Purpose

HEARTBEAT messages allow the hub to:
- Discover new nodes on the CAN bus
- Track which nodes are online/offline
- Display node information in the web UI
- Configure unconfigured nodes

## Message Format

### CAN ID Structure

**11-bit Standard CAN ID:**
```
CAN_ID = (MessageType << 7) | (NodeID & 0x7F)
```

Where:
- `MessageType = 0x8` (HEARTBEAT)
- `NodeID = 127` for unconfigured nodes, or `1-126` for configured nodes

**Examples:**
- Unconfigured node: `CAN_ID = (0x8 << 7) | 127 = 0x3FF`
- Configured node ID 5: `CAN_ID = (0x8 << 7) | 5 = 0x405`

### Payload Format

**Minimum DLC:** 2 bytes

| Byte | Field | Description | Values |
|------|-------|-------------|--------|
| 0 | node_type | Node hardware type | `1` = LCD, `2` = Mechanical |
| 1 | input_count | Number of inputs/buttons | `1-255` |
| 2–3 | fw_version | Firmware version (uint16 LE) | e.g. `0x0100` = v1.0; `0` if unknown |
| 4 | ota_capable | CAN OTA support | `1` = node accepts FIRMWARE messages; `0` = USB-only |

**Bytes 5–7:** Reserved; send zero or omit (DLC may be 2, 5, or 8).

Hub ignores bytes 5–7. Nodes with DLC &lt; 5 are treated as legacy (no version / not OTA-capable).

## Implementation Requirements

### For Unconfigured Nodes (First Boot)

1. **Node ID:** Use `127` (NODE_ID_UNCONFIGURED) until configured by hub
2. **Send HEARTBEAT periodically:** Recommended every 10-15 seconds
3. **Payload:**
   - Byte 0: Set to `1` (LCD) or `2` (Mechanical) based on hardware
   - Byte 1: Number of input buttons/channels (e.g., `4` for 4-button switch)

### For Configured Nodes

1. **Node ID:** Use the ID assigned by hub (1-126)
2. **Send HEARTBEAT periodically:** Recommended every 10-15 seconds
3. **Payload:** Same format as unconfigured nodes

### Example Code (Pseudocode)

```c
// For unconfigured node
uint8_t node_id = 127;  // NODE_ID_UNCONFIGURED
uint8_t node_type = 1;  // NODE_TYPE_LCD (or 2 for mechanical)
uint8_t input_count = 4; // Number of buttons

// Build CAN ID
uint16_t can_id = (0x8 << 7) | node_id;  // 0x3FF for unconfigured

// Build payload (extended heartbeat for OTA-capable mechanical nodes)
uint8_t payload[8] = {0};
payload[0] = node_type;
payload[1] = input_count;
payload[2] = (uint8_t)(FW_VERSION & 0xFF);
payload[3] = (uint8_t)(FW_VERSION >> 8);
payload[4] = OTA_CAPABLE ? 1 : 0;

// Send CAN frame
can_frame frame;
frame.can_id = can_id;
frame.can_dlc = 5;
memcpy(frame.data, payload, 5);
send_can_frame(&frame);
```

## Hub Behavior

The hub will:
1. **Detect new nodes** when it receives a HEARTBEAT with node_id = 127
2. **Track node status** (online if HEARTBEAT received within last 30 seconds)
3. **Display nodes** in the web UI "Nodes" tab
4. **Allow configuration** of unconfigured nodes via web UI

## Timing Recommendations

- **Initial HEARTBEAT:** Send within 1-2 seconds after boot
- **Periodic HEARTBEAT:** Send every 10-15 seconds while running
- **After configuration:** Continue sending HEARTBEAT with new node ID

## Node Type Values

- `NODE_TYPE_LCD = 1`: For nodes with LCD display (e.g., touchscreen switches)
- `NODE_TYPE_MECHANICAL = 2`: For nodes with physical buttons/switches

## Notes

- The hub accepts HEARTBEAT messages with DLC >= 2
- If DLC < 2, the hub will still update the node's timestamp but won't extract type/input_count
- Nodes should continue sending HEARTBEAT even after being configured
- The hub tracks "last seen" timestamp to determine online/offline status
