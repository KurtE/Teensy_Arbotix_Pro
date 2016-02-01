#ifndef _GLOBALS_H_
#define _GLOBALS_H_

//==================================================================
// Defines to include or not include options
//==================================================================
#define USE_LSM9DS1

//==================================================================
// Defines 
//==================================================================
#define HWSERIAL Serial1
#define DBGSerial Serial3

// Using #define instead of enum as some may be tested for by #ifdef
// Define Teeny IO pins
#define   AX_BUS_POWER_PIN  2
#define   NEOPIXEL_PIN      6
#define   LED_PIN           13
#define   LED2_PIN          24    // New on V2

#define   GPIO_1_PIN        4
#define   GPIO_2_PIN        5
#define   GPIO_3_PIN        0
#define   GPIO_4_PIN        8
#define   GPIO_5_PIN        8
#define   GPIO_6_PIN        10
  
#define   VOLTAGE_ANALOG_PIN  10   // Was 0 on V1
#define   BUTTONS_ANALOG_PIN  12   // Two buttons resistor ladder... 

#define     USART_TIMEOUT_MIN   8   //  x 20us
#define     SEND_TIMEOUT_MIN    0   //  x 20us
#define     RECEIVE_TIMEOUT_MIN 10  //  x 20us
//default values, can be modified with write_data and are saved in EEPROM
#define     USART_TIMEOUT       50   //  x 20us
#define     SEND_TIMEOUT        4    //  x 20us
#define   RECEIVE_TIMEOUT       100  //  x 20us

// Define our voltage divider
// Warning: values directly like 10000 and 40200 will overflow math, so reduce. 
//  example 100 and 402... 
#define   VOLTAGE_DIVIDER_RES1  100 // 10K
#define   VOLTAGE_DIVIDER_RES2  402 // 40.2K
#define   LOW_VOLTAGE_SHUTOFF_DEFAULT 90  // 9 volts

// AX-Bus Read pass though mode
#define AX_PASSTHROUGH      1
#define AX_DIVERT           2

//Dynamixel device Control table
#define AX_ID_DEVICE        200    // Default ID
#define AX_ID_BROADCAST     0xfe
#define MODEL_NUMBER_L      0x041 // 'A' ...
#define MODEL_NUMBER_H      0x54  // 'T' arbitrary model number that should not collide with the ones from Robotis
#define FIRMWARE_VERSION    0x05  // Firmware version, needs to be updated with every new release
#define RETURN_LEVEL         2


//extern uint8_t regs[REG_TABLE_SIZE];
#define REG_TABLE_SIZE      (CM730_ADC15_H+1)

// Define which IDs will saved to and restored from EEPROM
#define REG_EEPROM_FIRST    CM730_ID
#define REG_EEPROM_LAST     CM730_STATUS_RETURN_LEVEL

#define AX_CMD_SYNC_READ      0x84
#define SYNC_READ_START_ADDR  5
#define SYNC_READ_LENGTH      6

#define AX_BUFFER_SIZE              128
#define AX_SYNC_READ_MAX_DEVICES    120
#define AX_MAX_RETURN_PACKET_SIZE   235

enum {AX_SEARCH_FIRST_FF = 0, AX_SEARCH_SECOND_FF, PACKET_ID, PACKET_LENGTH,
      PACKET_INSTRUCTION, AX_SEARCH_RESET, AX_SEARCH_BOOTLOAD, AX_GET_PARAMETERS,
      AX_SEARCH_READ, AX_SEARCH_PING, AX_PASS_TO_SERVOS
     };

//==================================================================
// Registers - CM730(ish)
//==================================================================
// CM-730 address table
enum{
    CM730_MODEL_NUMBER_L              = 0,
    CM730_MODEL_NUMBER_H              = 1,
    CM730_FIRMWARE_VERSION            = 2,
    CM730_ID                          = 3,
    CM730_BAUD_RATE                   = 4,
    CM730_RETURN_DELAY_TIME           = 5,
    TA_DOWN_LIMIT_VOLTAGE              = 12,
    CM730_STATUS_RETURN_LEVEL         = 16,
    CM730_DXL_POWER                   = 24,
    CM730_LED_PANEL                   = 25, // Teensy D13 low bit. D12? for 2nd bit. 
    CM730_LED_HEAD_L                  = 26,
    CM730_LED_HEAD_H                  = 27,
//    CM730_LED_EYE_L                   = 28,
//    CM730_LED_EYE_H                   = 29,
    CM730_BUTTON                      = 30, // Teensy Arbotix D10 (Also uses GPIO7) 
    TA_GPIO_MODE                      = 31, // Teensy - GPIO pins Input or Output 
    TA_GPIO_1                         = 32, // GPI1 - D4
    TA_GPIO_2                         = 33, // GPI2 - D5
    TA_GPIO_3                         = 34, // GPI3 - D0
    TA_GPIO_4                         = 35, // GPI4 - D6
    TA_GPIO_5                         = 36, // GPI5 - D8
    TA_GPIO_6                         = 37, // GPI6 - D9

    CM730_GYRO_Z_L                    = 38,
    CM730_GYRO_Z_H                    = 39,
    CM730_GYRO_Y_L                    = 40,
    CM730_GYRO_Y_H                    = 41,
    CM730_GYRO_X_L                    = 42,
    CM730_GYRO_X_H                    = 43,
    CM730_ACCEL_X_L                   = 44,
    CM730_ACCEL_X_H                   = 45,
    CM730_ACCEL_Y_L                   = 46,
    CM730_ACCEL_Y_H                   = 47,
    CM730_ACCEL_Z_L                   = 48,
    CM730_ACCEL_Z_H                   = 49,
    CM730_VOLTAGE                     = 50, // A0
    CM730_ADC1_L                      = 51, // Not Implemented
    CM730_ADC1_H                      = 52, 
    CM730_ADC2_L                      = 53, // A1
    CM730_ADC2_H                      = 54,
    CM730_ADC3_L                      = 55, // A2
    CM730_ADC3_H                      = 56,
    CM730_ADC4_L                      = 57, // A3
    CM730_ADC4_H                      = 58,
    CM730_ADC5_L                      = 59, // A6
    CM730_ADC5_H                      = 60,
    CM730_ADC6_L                      = 61, // A7
    CM730_ADC6_H                      = 62,
    CM730_ADC7_L                      = 63, // A8
    CM730_ADC7_H                      = 64,
    CM730_ADC8_L                      = 65, // A9
    CM730_ADC8_H                      = 66,
    CM730_ADC9_H                      = 67, // A14 (CM730 - this is mic2)
    CM730_ADC9_L                      = 68,
    CM730_ADC10_L                     = 69, // A15
    CM730_ADC10_H                     = 70,
    CM730_ADC11_L                     = 71, // A16
    CM730_ADC11_H                     = 72,
    CM730_ADC12_L                     = 73, // A17
    CM730_ADC12_H                     = 74,
    CM730_ADC13_L                     = 75, // A18
    CM730_ADC13_H                     = 76,
    CM730_ADC14_L                     = 77, // A19
    CM730_ADC14_H                     = 78,
    CM730_ADC15_L                     = 79, // A20
    CM730_ADC15_H                     = 80
};

#if 0
// Arbotix Pro stuff - Only showing those things that have been added.

// Arbotix-Pro added. 
//#define -     80
#define P_BUZZER_DATA0        81
#define P_BUZZER_DATA1        82
#endif
// 83-89 used maybe on pro...
//==================================================================
// EEPROM and Min/Max
//==================================================================
extern uint8_t g_controller_registers[REG_TABLE_SIZE];

extern uint8_t g_passthrough_mode;
extern unsigned long last_message_time;
extern uint8_t ax_state;

extern uint8_t rxbyte[AX_SYNC_READ_MAX_DEVICES + 8]; // buffer where currently processed data are stored when looking for a Dynamixel packet, with enough space for longest possible sync read request
extern uint8_t rxbyte_count;   // number of used bytes in rxbyte buffer

//==================================================================
// function definitions
//==================================================================
extern void InitalizeRegisterTable(void);
extern void axStatusPacket(uint8_t err, uint8_t* data, uint8_t count_bytes);
extern void LocalRegistersRead(uint8_t register_id, uint8_t count_bytes);
extern void CheckBatteryVoltage(void);
extern void LocalRegistersWrite(uint8_t register_id, uint8_t* data, uint8_t count_bytes);
extern void sync_read(uint8_t id, uint8_t* params, uint8_t nb_params);
extern void setAXtoTX(bool fTX);
extern void MaybeFlushUSBOutputData(void);
extern void FlushUSBInputQueue(void);

extern bool ProcessInputFromUSB(void);
extern bool ProcessInputFromAXBuss(void);

//==================================================================
// Optional include neopixel stuff...
//==================================================================
#if NEOPIXEL_PIN
#include <Adafruit_NeoPixel.h>
extern Adafruit_NeoPixel strip;
#endif





#endif
