#ifndef MCP2515_DEFS_H
#define MCP2515_DEFS_H

// Bitrates (minimal set for demo)
#define CAN_125KBPS 0
#define CAN_250KBPS 1
#define CAN_500KBPS 2
#define CAN_1000KBPS 3

// Oscillator
#define MCP_8MHZ  0
#define MCP_16MHZ 1

// Registers / Commands (subset)
#define REG_CANCTRL  0x0F
#define REG_CANINTE  0x2B
#define REG_CANINTF  0x2C
#define REG_CNF1     0x2A
#define REG_CNF2     0x29
#define REG_CNF3     0x28
#define REG_RXB0CTRL 0x60
#define REG_RXB1CTRL 0x70

#define RX0IE 0x01
#define RX1IE 0x02
#define RX0IF 0x01
#define RX1IF 0x02

#define MCP_RESET      0xC0
#define MCP_READ       0x03
#define MCP_WRITE      0x02
#define MCP_BITMOD     0x05
#define MCP_RX_STATUS  0xB0
#define MCP_READ_RXB0  0x90
#define MCP_READ_RXB1  0x94
#define MCP_RTS_TXB0   0x81

#endif
