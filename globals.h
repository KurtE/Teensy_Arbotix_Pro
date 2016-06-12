#ifndef _GLOBALS_H_
#define _GLOBALS_H_

//==================================================================
// Defines to include or not include options
//==================================================================
//#define USE_LSM9DS1

//#define V03_BOARD
//#define V04_BOARD

//==================================================================
// Defines
//==================================================================
//#define HWSerial_TXPIN    1       // hack when we turn off TX pin turns to normal IO, try to set high...
#define HWSERIAL Serial1
#define DBGSerial Serial2
#define USE_DEBUG_IOPINS


// Some options may depend on if we are communicating using USB or not.
#define PCSerial Serial   // Default to USB
#define PCSerial_USB    // Is the PCSerial going to USB?
#define RECIVE_CHAR_LOOP_COUNT 20
//#define BUFFER_TO_PC


#define SERVO_DIRECTION_PIN -1
#define   AX_BUS_POWER_PIN  2
#define   SOUND_PIN         3
#define   GPIO_1_PIN        4
#define   GPIO_2_PIN        5
#define   GPIO_3_PIN        0
#define   GPIO_4_PIN        8
#define   GPIO_5_PIN        8
#define   GPIO_6_PIN        10

#ifdef V03_BOARD
//V0.3
#define SERVO_DIRECTION_PIN 2
#define   AX_BUS_POWER_PIN  3
#define   SOUND_PIN         4

#define   GPIO_1_PIN        32
#define   GPIO_2_PIN        33
#define   GPIO_3_PIN        5
#define   GPIO_4_PIN        8
#define   GPIO_5_PIN        11
#define   GPIO_6_PIN        12
#endif

#define DEBUG_PIN_USB_INPUT           A0
#define DEBUG_PIN_SEND_STATUS_PACKET  A1
#define DEBUG_PIN_AX_INPUT            A2
#define DEBUG_PIN_BACKGROUND          A3
#define DEBUG_PIN_AX_TO_HOST          A6
#define DEBUG_PIN_FLUSH_TO_HOST       A7

#ifdef BUFFER_TO_PC
extern uint8_t g_abToPCBuffer[256]; // way more than enough buffer space
extern uint8_t g_abToPCCnt;
#endif


#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif
#define   LED_PIN           LED_BUILTIN
#define   LED2_PIN          24    // New on V2
#define   NEOPIXEL_PIN      6


#define   VOLTAGE_ANALOG_PIN  10   // Was 0 on V1
#define   BUTTONS_ANALOG_PIN  12   // Two buttons resistor ladder...


#ifdef USE_DEBUG_IOPINS
#define debug_digitalWrite(pin, state)  digitalWrite((pin), (state))
#define debug_digitalToggle(pin)  digitalWrite((pin), !digitalRead((pin)))
#else
#define debug_digitalWrite(pin, state)
#define debug_digitalToggle(pin)
#endif
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
#define   VOLTAGE_DIVIDER_RES1  50 // 10K
#define   VOLTAGE_DIVIDER_RES2  201 // 40.2K
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
#define REG_TABLE_SIZE      (TDSC_GM_SERVO_31_GOAL_POS_H+1)

// Define which IDs will saved to and restored from EEPROM
#define REG_EEPROM_FIRST    TDSC_ID
#define REG_EEPROM_LAST     TDSC_STATUS_RETURN_LEVEL

// Added commands
#define AX_CMD_SYNC_READ      0x84    // USBTOAX
#define AX_CMD_BULK_READ     0x92     // BULKREAD Arbotix Pro and others

#define SYNC_READ_START_ADDR  5
#define SYNC_READ_LENGTH      6

#define AX_BUFFER_SIZE              128
#define AX_SYNC_READ_MAX_DEVICES    120
#define AX_MAX_RETURN_PACKET_SIZE   235
#define TDSC_MAX_GM_SIZE            32
enum {AX_SEARCH_FIRST_FF = 0, AX_SEARCH_SECOND_FF, PACKET_ID, PACKET_LENGTH,
      PACKET_INSTRUCTION, AX_SEARCH_RESET, AX_SEARCH_BOOTLOAD, AX_GET_PARAMETERS,
      AX_SEARCH_READ, AX_SEARCH_PING, AX_PASS_TO_SERVOS
     };

//==================================================================
// Registers - CM730(ish)
//==================================================================
// CM-730 address table
enum {
  TDSC_MODEL_NUMBER_L              = 0,
  TDSC_MODEL_NUMBER_H              = 1,
  TDSC_FIRMWARE_VERSION            = 2,
  TDSC_ID                          = 3,
  TDSC_BAUD_RATE                   = 4,
  TDSC_RETURN_DELAY_TIME           = 5,
  TA_DOWN_LIMIT_VOLTAGE              = 12,
  TDSC_STATUS_RETURN_LEVEL         = 16,
  TDSC_DXL_POWER                   = 24,
  TDSC_LED_PANEL                   = 25, // Teensy D13 low bit. D12? for 2nd bit.
  TDSC_LED_HEAD_L                  = 26,
  TDSC_LED_HEAD_H                  = 27,
  //    TDSC_LED_EYE_L                   = 28,
  //    TDSC_LED_EYE_H                   = 29,
  TDSC_BUTTON                      = 30, // Teensy Arbotix D10 (Also uses GPIO7)
  TDSC_GPIO_MODE                      = 31, // Teensy - GPIO pins Input or Output
  TDSC_GPIO_1                         = 32, // GPI1 - D4
  TDSC_GPIO_2                         = 33, // GPI2 - D5
  TDSC_GPIO_3                         = 34, // GPI3 - D0
  TDSC_GPIO_4                         = 35, // GPI4 - D6
  TDSC_GPIO_5                         = 36, // GPI5 - D8
  TDSC_GPIO_6                         = 37, // GPI6 - D9

  TDSC_GYRO_Z_L                    = 38,
  TDSC_GYRO_Z_H                    = 39,
  TDSC_GYRO_Y_L                    = 40,
  TDSC_GYRO_Y_H                    = 41,
  TDSC_GYRO_X_L                    = 42,
  TDSC_GYRO_X_H                    = 43,
  TDSC_ACCEL_X_L                   = 44,
  TDSC_ACCEL_X_H                   = 45,
  TDSC_ACCEL_Y_L                   = 46,
  TDSC_ACCEL_Y_H                   = 47,
  TDSC_ACCEL_Z_L                   = 48,
  TDSC_ACCEL_Z_H                   = 49,
  TDSC_VOLTAGE                     = 50, // A0
  TDSC_ADC1_L                      = 51, // Not Implemented
  TDSC_ADC1_H                      = 52,
  TDSC_ADC2_L                      = 53, // A1
  TDSC_ADC2_H                      = 54,
  TDSC_ADC3_L                      = 55, // A2
  TDSC_ADC3_H                      = 56,
  TDSC_ADC4_L                      = 57, // A3
  TDSC_ADC4_H                      = 58,
  TDSC_ADC5_L                      = 59, // A6
  TDSC_ADC5_H                      = 60,
  TDSC_ADC6_L                      = 61, // A7
  TDSC_ADC6_H                      = 62,
  TDSC_ADC7_L                      = 63, // A8
  TDSC_ADC7_H                      = 64,
  TDSC_ADC8_L                      = 65, // A9
  TDSC_ADC8_H                      = 66,
  TDSC_ADC9_H                      = 67, // A14 (CM730 - this is mic2)
  TDSC_ADC9_L                      = 68,
  TDSC_ADC10_L                     = 69, // A15
  TDSC_ADC10_H                     = 70,
  TDSC_ADC11_L                     = 71, // A16
  TDSC_ADC11_H                     = 72,
  TDSC_ADC12_L                     = 73, // A17
  TDSC_ADC12_H                     = 74,
  TDSC_ADC13_L                     = 75, // A18
  TDSC_ADC13_H                     = 76,
  TDSC_ADC14_L                     = 77, // A19
  TDSC_ADC14_H                     = 78,
  TDSC_ADC15_L                     = 79, // A20
  TDSC_ADC15_H                     = 80,

  // Extensions for Group moves
  // Two counts - First is count of the servos(TDSC_GM_SERVO_CNT) that are part of group moves, like the 18 leg servos on hexapod
  //   Second TDSC_GM_SERVO_CNT_TOTAL - allows for independant timed move servos like Pan servo on Hexapod.
  TDSC_GM_SERVO_CNT                = 96,  // How many slots are used when user updates TDSC_GM_MOVE_COMMAND
  TDSC_GM_SERVO_CNT_TOTAL,                // Total count - allows ones outside of group move.
  TDSC_GM_FRAME_TIME_MS,                  // Frame time in mS
  TDSC_GM_IO_PIN_MOVE_ACTIVE,             // option to use io pin to show that GM is active
  TDSC_GM_IO_PIN_MOVE_INTERPOLATE,        // Like above but shows when interpolation is happening.
  TDSC_GM_SERVO_0_ID,
  TDSC_GM_SERVO_1_ID,
  // ... Through Servo 32
  TDSC_GM_SERVO_31_ID               = (TDSC_GM_SERVO_0_ID + TDSC_MAX_GM_SIZE - 1),
  TDSC_GM_SERVO_CNT_MOVING,
  TDSC_GM_MOVE_COMMAND,              // Move Status options
  TDSC_GM_MOVE_TIME_L,              // Low move time
  TDSC_GM_MOVE_TIME_H,              // high move time
  TDSC_GM_SERVO_0_GOAL_POS_L,       // Low goal position for first servo
  TDSC_GM_SERVO_0_GOAL_POS_H,       // Low goal position for first servo
  // ... Through servo 32
  TDSC_GM_SERVO_31_GOAL_POS_L = ((TDSC_GM_SERVO_0_GOAL_POS_L) + (TDSC_MAX_GM_SIZE - 1) * 2),    // Low goal position for first servo
  TDSC_GM_SERVO_31_GOAL_POS_H,         // Low goal position for first servo

  // Single Servo Timed move Command
  TDSC_TM_SLOT,                     // Which slot
  TDSC_TM_MOVE_TIME_L,
  TDSC_TM_MOVE_TIME_H,
  TDSC_TM_GOAL_POS_L,
  TDSC_TM_GOAL_POS_H
};

// Define Group Move Command Options, which are a set of bits
//
#define TDSC_GM_CMD_NEW     0x01    // New move
#define TDSC_GM_CMD_CHAIN   0x02    //  If set allow active one to complete first
#define TDSC_GM_CMD_READ    0x04    //  Hint should we first get positions
#define TDSC_GM_CMD_ABORT   0x80    // Abort active and pending
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
extern uint8_t ax_tohost_state;

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
extern void MaybeFlushDataBackToHost(void);
extern void FlushUSBInputQueue(void);
extern void UpdateHardwareAfterLocalWrite(uint8_t register_id, uint8_t count_bytes);
extern void CheckHardwareForLocalReadRequest(uint8_t register_id, uint8_t count_bytes);

extern bool ProcessInputFromUSB(void);
extern bool ProcessInputFromAXBuss(void);
extern void ProcessGroupMoveCommand(void);

extern void PoseInterpolateStepTask(void);

//==================================================================
// inline functions
//==================================================================
//-----------------------------------------------------------------------------
// setAXtoTX - Set the Tx buffer to input or output.
//-----------------------------------------------------------------------------
extern bool g_AX_IS_TX;

inline void  setAXtoTX()
{
  if (!g_AX_IS_TX) {
    g_AX_IS_TX = true;
#ifdef DBGSerial
    //    DBGSerial.println("TX");
#endif
    setTX(0);
  }
}

inline void  setAXtoRX()
{
  if (g_AX_IS_TX) {
    g_AX_IS_TX = false;
#ifdef DBGSerial
    //    DBGSerial.println("RX");
#endif
    setRX(0);
  }
}

//==================================================================
// Optional include neopixel stuff...
//==================================================================
#if NEOPIXEL_PIN
#include <Adafruit_NeoPixel.h>
extern Adafruit_NeoPixel strip;
#endif





#endif
