#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

// --- Select your board ---
// Uncomment the board you are compiling for
#define BOARD_RACK32
//#define BOARD_OLIMEX_POE


#if defined(BOARD_OLIMEX_POE)
    #define ETH_MDC_GPIO      23
    #define ETH_MDIO_GPIO     18
    #define ETH_PHY_ADDR      0
    #define ETH_RST_GPIO     -1
    #define ETH_PHY_PWR_GPIO  12
    #define I2C_MASTER_SCL_IO 16
    #define I2C_MASTER_SDA_IO 13

    #define CAN_MISO_GPIO  33
    #define CAN_MOSI_GPIO  32
    #define CAN_CLK_GPIO   4
    #define CAN_INT_GPIO   35
    #define CAN_CS_GPIO    5
#elif defined(BOARD_RACK32)
    // --- Rack32 GPIOs ---
    #define W5500_HOST        SPI2_HOST
    #define W5500_MOSI_GPIO   23
    #define W5500_MISO_GPIO   19
    #define W5500_SCLK_GPIO   18
    #define W5500_CS_GPIO     26
    #define W5500_INT_GPIO    34
    #define W5500_RST_GPIO    13
    #define I2C_MASTER_SCL_IO 22
    #define I2C_MASTER_SDA_IO 21

    #define CAN_MISO_GPIO  33 // J11 pin 4 (IO33_SDA2)
    #define CAN_MOSI_GPIO  32 // J11 pin 5 (IO32_SCL2)
    #define CAN_CLK_GPIO   17 // J11 pin 7 (IO17_TX2) 
    #define CAN_INT_GPIO   16 // J11 pin 8 (IO16_RX2)
    #define CAN_CS_GPIO    25 // J1 pin 8 (IO25_TFT_CS)
#endif

// --- Common Pin Definitions ---
#define I2C_MASTER_NUM    I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000
#define CAN_HOST          SPI3_HOST

#endif // HARDWARE_CONFIG_H