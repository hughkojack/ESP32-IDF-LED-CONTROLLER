#ifndef MCP2515_DEFS_H
#define MCP2515_DEFS_H

// MCP2515 Commands
#define MCP_WRITE       0x02
#define MCP_READ        0x03
#define MCP_BITMOD      0x05
#define MCP_LOAD_TXB0   0x40
#define MCP_LOAD_TXB1   0x42
#define MCP_LOAD_TXB2   0x44
#define MCP_RTS_TXB0    0x81
#define MCP_RTS_TXB1    0x82
#define MCP_RTS_TXB2    0x84
#define MCP_READ_RXB0   0x90
#define MCP_READ_RXB1   0x94
#define MCP_READ_STATUS 0xA0
#define MCP_RX_STATUS   0xB0
#define MCP_RESET       0xC0

// MCP2515 Registers
#define REG_CANCTRL     0x0F
#define REG_CANSTAT     0x0E
#define REG_CNF1        0x2A
#define REG_CNF2        0x29
#define REG_CNF3        0x28
#define REG_CANINTE     0x2B
#define REG_CANINTF     0x2C
#define REG_RXB0CTRL    0x60
#define REG_RXB1CTRL    0x70

// Interrupt flags
#define RX0IE           0x01
#define RX1IE           0x02
#define RX0IF           0x01
#define RX1IF           0x02

// Bitrate options
#define CAN_500KBPS     0
#define CAN_250KBPS     1
#define CAN_125KBPS     2
#define CAN_100KBPS     3
#define CAN_50KBPS      4
#define CAN_25KBPS      5

// Clock options
#define MCP_8MHZ        0
#define MCP_16MHZ       1

#endif