//=============================================================================
// File: LocalRegisters.cpp
// Description: Definitions and code to handle the logical registers associated
//      with the actual controller
//=============================================================================

//=============================================================================
// Header Files
//=============================================================================
#include <EEPROM.h>
#include <ax12Serial.h>
#include <BioloidSerial.h>
#include "globals.h"

//-----------------------------------------------------------------------------
// Define Global variables
//-----------------------------------------------------------------------------
//                                                    0             1              2                   3            4             5              6              7
uint8_t g_controller_registers[REG_TABLE_SIZE] = {MODEL_NUMBER_L, MODEL_NUMBER_H, FIRMWARE_VERSION, AX_ID_DEVICE, USART_TIMEOUT, SEND_TIMEOUT, RECEIVE_TIMEOUT, 0,
                                                  //                                                      8  9  0  1  2                            3  4  5  6
                                                  0, 0, 0, 0, LOW_VOLTAGE_SHUTOFF_DEFAULT, 0, 0, 0, RETURN_LEVEL
                                                 };
uint8_t g_controller_gpio_mode_save = 0;

// Maps which Teensy digital pins map the logical GPIO pins.
const uint8_t g_controller_gpio_PIN_mapping_table[] =
{GPIO_1_PIN, GPIO_2_PIN, GPIO_3_PIN, GPIO_4_PIN, GPIO_5_PIN, GPIO_6_PIN};

// Maps which Teensy analog pins map to the logical Analog pins - Warning Arduino has wierd pin numbering for analog
// 0-13 are Analog 0-13 14-23 are again A0-A9, ...
// For debug will map 0 into microphone 1 as to debug voltage code
const uint8_t g_controller_analog_PIN_mapping_table[] =
{ 0, 1, 2, 3, 6, 7, 8, 9, A14, A15, A16, A17, A18, A19, A20 };




const uint8_t g_controller_registers_ranges[][2] =
{
  {1, 0},   //MODEL_NUMBER_L        0
  {1, 0},   //MODEL_NUMBER_H        1
  {1, 0},   //VERSION               2
  {0, 253}, //ID                    3
  {1, 254}, //BAUD_RATE             4
  {0, 254}, //Return Delay time     5
  {0, 0xff}, {0, 0xff},  {0, 0xff},  {0, 0xff}, {0, 0xff}, {0, 0xff}, // 6-11
  {0, 250}, //DOWN_LIMIT_VOLTAGE   12
  {50, 250}, //UP_LIMIT_VOLTAGE 13
  {0, 0xff},  {0, 0xff},      // 14-15
  {0, 2}, //RETURN_LEVEL          16

  // Not saved to eeprom...
  {1, 0}, {1, 0},  {1, 0},  {1, 0}, {1, 0}, {1, 0}, {1, 0}, // 17-23

  //RAM area
  {0, 1}, //  P_DYNAMIXEL_POWER 24
  {0, 7}, //  P_LED_PANNEL  25
  {0, 0xff}, //  P_LED_HEAD  26
  {0, 127}, //  - 27
  {0, 0xff}, //  P_LED_EYE 28
  {0, 127}, //  - 29
  {0, 1}, //  P_BUTTON  30 Also D7
  {0, 0xff},  //  - 31 digital mode
  {0, 1}, //  - 32 D1
  {0, 1}, //  - 33 D2
  {0, 1}, //  - 34 D3
  {0, 1}, //  - 35 D4
  {0, 1}, //  - 36 D5
  {0, 1}, //  - 37 D6
  {1, 0}, //  P_GYRO_Z  38
  {1, 0}, //  - 39
  {1, 0}, //  P_GYRO_Y  40
  {1, 0}, //  - 41
  {1, 0}, //  P_GYRO_X  42
  {1, 0}, //  - 43
  {1, 0}, //  P_ACC_X 44
  {1, 0}, //  - 45
  {1, 0}, //  P_ACC_Y 46
  {1, 0}, //  - 47
  {1, 0}, //  P_ACC_Z 48
  {1, 0}, //  - 49
  {1, 0}, //  P_ADC0_VOLTAGE  50
  {1, 0}, //  P_ADC1_MIC1   51
  {1, 0}, //  -       52
  {1, 0}, //  P_ADC2   53
  {1, 0}, //  -       54
  {1, 0}, //  P_ADC3      55
  {1, 0}, //  -       56
  {1, 0}, //  P_ADC4      57
  {1, 0}, //  -       58
  {1, 0}, //  P_ADC5      59
  {1, 0}, //  -       60
  {1, 0}, //  P_ADC6      61
  {1, 0}, //  -       62
  {1, 0}, //  P_ADC7      63
  {1, 0}, //  -       64
  {1, 0}, //  P_ADC8      65
  {1, 0}, //  -       66
  {1, 0}, //  P_ADC9      67
  {1, 0}, //  -       68
  {1, 0}, //  P_ADC10     69
  {1, 0}, //  -       70
  {1, 0}, //  P_ADC11     71
  {1, 0}, //  -       72
  {1, 0}, //  P_ADC12     73
  {1, 0}, //  -       74
  {1, 0}, //  P_ADC13     75
  {1, 0}, //  -       76
  {1, 0}, //  P_ADC14     77
  {1, 0}, //  -       78
  {1, 0}, //  P_ADC15     79
  {1, 0}, //  -       80
  // Setup the  Interpolation area...
  {1, 0},{1, 0},{1, 0},{1, 0},{1, 0},{1, 0},{1, 0},{1, 0},{1, 0},{1, 0}, // 81-90
  {1, 0},{1, 0},{1, 0},{1, 0},{1, 0},                                    // 91-5
  {0, 32},    // TDSC_GM_SERVO_CNT 
  {0, 32},    // TDSC_GM_SERVO_CNT_TOTAL,                // Total count - allows ones outside of group move.
  {5, 50},    // TDSC_GM_FRAME_TIME_MS,                  // Frame time in mS
  {0, 0xff},   // TDSC_GM_IO_PIN_MOVE_ACTIVE,             // option to use io pin to show that GM is active
  {0, 0xff},   // TDSC_GM_IO_PIN_MOVE_INTERPOLATE,        // Like above but shows when interpolation is happening.
  {0, 0xff},   {0, 0xff}, {0, 0xff},   {0, 0xff},   {0, 0xff}, {0, 0xff},   {0, 0xff},   {0, 0xff},  // TDSC_GM_SERVO_0_ID - 31
  {0, 0xff},   {0, 0xff}, {0, 0xff},   {0, 0xff},   {0, 0xff}, {0, 0xff},   {0, 0xff},   {0, 0xff},       
  {0, 0xff},   {0, 0xff}, {0, 0xff},   {0, 0xff},   {0, 0xff}, {0, 0xff},   {0, 0xff},   {0, 0xff},       
  {0, 0xff},   {0, 0xff}, {0, 0xff},   {0, 0xff},   {0, 0xff}, {0, 0xff},   {0, 0xff},   {0, 0xff},       
  {0, 0xff},   // TDSC_GM_SERVO_CNT_MOVING,
  {0, 0xff},   // TDSC_GM_MOVE_COMMAND,              // Move Status options
  {0, 0xff},   // TDSC_GM_MOVE_TIME_L,              // Low move time
  {0, 0xff},   // TDSC_GM_MOVE_TIME_H,              // high move time
  {0, 0xff},{0, 0xff}, {0, 0xff},{0, 0xff}, {0, 0xff},{0, 0xff}, {0, 0xff},{0, 0xff},         // TDSC_GM_SERVO_0_GOAL_POS_L H ,
  {0, 0xff},{0, 0xff}, {0, 0xff},{0, 0xff}, {0, 0xff},{0, 0xff}, {0, 0xff},{0, 0xff},         //
  {0, 0xff},{0, 0xff}, {0, 0xff},{0, 0xff}, {0, 0xff},{0, 0xff}, {0, 0xff},{0, 0xff},         //
  {0, 0xff},{0, 0xff}, {0, 0xff},{0, 0xff}, {0, 0xff},{0, 0xff}, {0, 0xff},{0, 0xff},         //
  {0, 0xff},{0, 0xff}, {0, 0xff},{0, 0xff}, {0, 0xff},{0, 0xff}, {0, 0xff},{0, 0xff},         //
  {0, 0xff},{0, 0xff}, {0, 0xff},{0, 0xff}, {0, 0xff},{0, 0xff}, {0, 0xff},{0, 0xff},         //
  {0, 0xff},{0, 0xff}, {0, 0xff},{0, 0xff}, {0, 0xff},{0, 0xff}, {0, 0xff},{0, 0xff},         //
  {0, 0xff},{0, 0xff}, {0, 0xff},{0, 0xff}, {0, 0xff},{0, 0xff}, {0, 0xff},{0, 0xff},         //
  // Single Servo Timed move Command
  {0, 31},      // TDSC_TM_SLOT,                     // Which slot
  {0, 0xff},   //TDSC_TM_MOVE_TIME_L,
  {0, 0xff},   //TDSC_TM_MOVE_TIME_H,
  {0, 0xff},   //TDSC_TM_GOAL_POS_L,
  {0, 0xff}   //TDSC_TM_GOAL_POS_H
};


//-----------------------------------------------------------------------------
// Forward reference
//-----------------------------------------------------------------------------

// helper functions for read
extern void CheckHardwareForLocalReadRequest(uint8_t register_id, uint8_t count_bytes);

// Helper functions for write
extern uint8_t ValidateWriteData(uint8_t register_id, uint8_t* data, uint8_t count_bytes);
extern void SaveEEPromSectionsLocalRegisters(void);
extern void UpdateHardwareAfterLocalWrite(uint8_t register_id, uint8_t count_bytes);



//-----------------------------------------------------------------------------
// LocalRegistersRead
//-----------------------------------------------------------------------------
void LocalRegistersRead(uint8_t register_id, uint8_t count_bytes)
{
#ifdef DBGSerial
  DBGSerial.printf("LR: %d %d\n\r", register_id, count_bytes);
#endif

  // Several ranges of logical registers to process.
  uint16_t top = (uint16_t)register_id + count_bytes;
  if ( count_bytes == 0  || (top >= REG_TABLE_SIZE))
  {
    axStatusPacket( ERR_RANGE, NULL, 0 );
    return;
  }

  // See if we need to do any preprocesing.
  CheckHardwareForLocalReadRequest(register_id, count_bytes);

  axStatusPacket(ERR_NONE, g_controller_registers + register_id, count_bytes);
}


//-----------------------------------------------------------------------------
// LocalRegistersWrite: Update the local registers
//-----------------------------------------------------------------------------
void LocalRegistersWrite(uint8_t register_id, uint8_t* data, uint8_t count_bytes)
{
#ifdef DBGSerial
  DBGSerial.printf("LW: %d %d %x\n\r", register_id, count_bytes, *data);
#endif
  if ( ! ValidateWriteData(register_id, data, count_bytes) ) {
    axStatusPacket( ERR_RANGE, NULL, 0 );
  } else {
    memcpy(g_controller_registers + register_id, data, count_bytes);

    // If at least some of the registers set is in the EEPROM area, save updates
    if (register_id <= TDSC_STATUS_RETURN_LEVEL)
      SaveEEPromSectionsLocalRegisters();
    axStatusPacket(ERR_NONE, NULL, 0 );

    // Check to see if we need to do anything to the hardware in response to the
    // changes
    UpdateHardwareAfterLocalWrite(register_id, count_bytes);

  }
}


//--------------------------------------------------------------------
// CheckBatteryVoltage - We will call this from main loop.
//    It does some averaging of voltage reads to get a better consisten
//    voltage.  It will also try to detec when the voltage goes to low
//    either due to the battery is getting low or turned off.  Likewise
//    may detect when battery is turned on...
//--------------------------------------------------------------------

// Warning may need to increase sizes if we go beyond 10bit analog reads
#define MAX_ANALOG_DELTA 50
uint16_t  g_awVoltages[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint16_t  g_wVoltageSum = 0;
uint8_t   g_iVoltages = 0;

void CheckBatteryVoltage(void)
{
  // Get the current voltage
  uint16_t cur_analog = analogRead(VOLTAGE_ANALOG_PIN);

  if (abs((int)cur_analog - (int)(g_wVoltageSum / 8)) > MAX_ANALOG_DELTA)
  {
    // Lets read 8 times and reset sum...
    g_wVoltageSum = 0;
    for (g_iVoltages = 0; g_iVoltages < 8; g_iVoltages++)
    {
      g_awVoltages[g_iVoltages] = analogRead(VOLTAGE_ANALOG_PIN);
      g_wVoltageSum += g_awVoltages[g_iVoltages];
    }
  }
  else
  {
    g_iVoltages++;
    g_iVoltages &= 0x7;  // setup index to our array...
    g_wVoltageSum -= g_awVoltages[g_iVoltages];
    g_awVoltages[g_iVoltages] = cur_analog;
    g_wVoltageSum += cur_analog;
  }
  // Warning - using resistor values like 10000 and 40200 will overflow 32 bit math, but simply need right ratio. so wuse 100 and 402
  g_controller_registers[TDSC_VOLTAGE] = (uint32_t)(g_wVoltageSum * 33 * (VOLTAGE_DIVIDER_RES1 + VOLTAGE_DIVIDER_RES2))
                                         / (uint32_t)(1024 * 8 * VOLTAGE_DIVIDER_RES1);

  // Check to see if voltage went low and our servos are on and a low voltage value is set.
  if (g_controller_registers[TDSC_DXL_POWER] && g_controller_registers[TA_DOWN_LIMIT_VOLTAGE]
      && (g_controller_registers[TDSC_VOLTAGE] <  g_controller_registers[TA_DOWN_LIMIT_VOLTAGE] ))
  {
    // Power is too low to run servos so shut them off
    g_controller_registers[TDSC_DXL_POWER] = 0;  // Turn it logically off.
    UpdateHardwareAfterLocalWrite(TDSC_DXL_POWER, 1);  // use the update function to do the real work.
  }
}


//-----------------------------------------------------------------------------
// CheckHardwareForLocalReadRequest - Some of this will change later to probably
//        some background tasks...
//-----------------------------------------------------------------------------
void CheckHardwareForLocalReadRequest(uint8_t register_id, uint8_t count_bytes)
{
  while (count_bytes)
  {
    // Digital Input Range.
    if (register_id == TDSC_BUTTON)
    {
      uint16_t analog_value = analogRead(BUTTONS_ANALOG_PIN);

      // quick and dirty analog code to set which if any button may be pressed.
      // Try to handle both
      g_controller_registers[TDSC_BUTTON] = (analog_value > 540) ? 3 :
                                            (analog_value > 450) ? 1 : (analog_value > 150) ? 2 : 0;
    }

    // Digital Input Range.
    else if ((register_id >= TDSC_GPIO_1) && (register_id <= TDSC_GPIO_6))
    {
      g_controller_registers[register_id] = digitalRead(g_controller_gpio_PIN_mapping_table[register_id - TDSC_GPIO_1]);
    }

    // IMU Range ?
    // Our battery voltage - is done in background

    // Analog input range
    else if ((register_id >= TDSC_ADC1_L) && (register_id <= TDSC_ADC15_H))
    {
#ifndef USE_DEBUG_IOPINS  // Right now using these pins for debug stuff
      // Get which Analog input we should be getting.
      uint8_t i = (register_id - TDSC_ADC1_L) / 2;
      uint16_t analog_value = analogRead(g_controller_analog_PIN_mapping_table[i]);
      g_controller_registers[TDSC_ADC1_L + i * 2] = analog_value & 0xff;
      g_controller_registers[TDSC_ADC1_L + i * 2 + 1] = (analog_value >> 8) & 0xff;
      register_id++;
      if (!count_bytes--)
        break;
#endif
    }
    register_id++;
    count_bytes--;
  }

}

//-----------------------------------------------------------------------------
// ValidateWriteData: is this a valid range of registers to update?
//-----------------------------------------------------------------------------
uint8_t ValidateWriteData(uint8_t register_id, uint8_t* data, uint8_t count_bytes)
{
  uint16_t top = (uint16_t)register_id + count_bytes;
  if (count_bytes == 0  || ( top >= REG_TABLE_SIZE)) {
#ifdef DBGSerial
    DBGSerial.printf("VWDE: cb:%d, top:%d >= %d\n\r", count_bytes, top, REG_TABLE_SIZE);
#endif
    return false;
  }
  // Check that the value written are acceptable
  for (uint8_t i = 0 ; i < count_bytes; i++ ) {
    uint8_t val = data[i];
    if ((val < g_controller_registers_ranges[register_id + i][0] ) ||
        (val > g_controller_registers_ranges[register_id + i][1] ))
    {
#ifdef DBGSerial
    DBGSerial.printf("VWDE: R:%d %d (%d %d)\n\r",  register_id + i, val, g_controller_registers_ranges[register_id + i][0],
          g_controller_registers_ranges[register_id + i][1]);
#endif
      return false;
    }
  }
  return true;
}

//-----------------------------------------------------------------------------
// UpdateHardwareAfterLocalWrite
//-----------------------------------------------------------------------------
void UpdateHardwareAfterLocalWrite(uint8_t register_id, uint8_t count_bytes)
{
  uint8_t i;
  uint8_t mask;
  while (count_bytes)
  {
    switch (register_id)
    {
      case TDSC_DXL_POWER:
        digitalWriteFast(AX_BUS_POWER_PIN, g_controller_registers[TDSC_DXL_POWER]);
        break;

      // Handle Group move command.
      case TDSC_GM_MOVE_COMMAND:
        ProcessGroupMoveCommand();
        break;

      case TDSC_GM_IO_PIN_MOVE_ACTIVE:
      case TDSC_GM_IO_PIN_MOVE_INTERPOLATE:
        if (g_controller_registers[register_id] != 0xff)
          pinMode(g_controller_registers[register_id], OUTPUT);
        break;


      case TDSC_LED_PANEL:
        digitalWriteFast(LED_PIN, g_controller_registers[TDSC_LED_PANEL] & 1);
#ifdef LED2_PIN
        digitalWriteFast(LED2_PIN, (g_controller_registers[TDSC_LED_PANEL] & 2) ? HIGH : LOW);
#endif
        break;
#ifdef NEOPIXEL_PIN
      case TDSC_LED_HEAD_L:
        if (count_bytes > 1)
          break;  // process on the high byte
      case TDSC_LED_HEAD_H:
        {
          uint16_t rgb = (((uint16_t)(g_controller_registers[TDSC_LED_HEAD_H])) << 8) + (uint16_t)(g_controller_registers[TDSC_LED_HEAD_L]);
          uint8_t r = (rgb  << 3) & 0xf8;
          uint8_t g = (rgb >> 2) & 0xf8;
          uint8_t b = (rgb >> 7) & 0xf8;
          strip.setPixelColor(0, r, g, b );
          strip.show(); // Initialize all pixels to 'off'
        }
        break;
#endif
      case TDSC_GPIO_MODE:
        // Set the GPIO pins modes.
        if (g_controller_gpio_mode_save != g_controller_registers[TDSC_GPIO_MODE])
        {
          mask = 0x1;
          for (i = 0; i < sizeof(g_controller_gpio_PIN_mapping_table); i++)
          {
            if ((g_controller_gpio_mode_save & mask) != (g_controller_registers[TDSC_GPIO_MODE] & mask))
              pinMode(g_controller_gpio_PIN_mapping_table[i], (g_controller_registers[TDSC_GPIO_MODE] & mask) ? OUTPUT : INPUT);
            mask <<= 1;
          }
          g_controller_gpio_mode_save = g_controller_registers[TDSC_GPIO_MODE];
        }
        break;
      case TDSC_GPIO_1:
      case TDSC_GPIO_2:
      case TDSC_GPIO_3:
      case TDSC_GPIO_4:
      case TDSC_GPIO_5:
      case TDSC_GPIO_6:
        i = register_id - TDSC_GPIO_1;
        mask = 1 << i;
        if ((g_controller_registers[TDSC_GPIO_MODE] & mask))
          digitalWrite(g_controller_gpio_PIN_mapping_table[i], g_controller_registers[register_id]);
        else
          pinMode(g_controller_gpio_PIN_mapping_table[i], g_controller_registers[register_id] ? INPUT_PULLUP : INPUT);
        break;


    }
    register_id++;
    count_bytes--;
  }
}


//-----------------------------------------------------------------------------
// InitializeRegisterTable()
//-----------------------------------------------------------------------------
void InitalizeRegisterTable(void)
{
  uint8_t saved_reg_values[TDSC_STATUS_RETURN_LEVEL + 1];
  uint8_t checksum = 0;

  // First check to see if valid version is stored in EEPROM...
  if (EEPROM.read(1) != FIRMWARE_VERSION)
    return;

  // Now read in the bytes from the EEPROM
  for (int i = TDSC_FIRMWARE_VERSION; i <= TDSC_STATUS_RETURN_LEVEL; i++)
  {
    uint8_t ch = EEPROM.read(1 + i - TDSC_FIRMWARE_VERSION);
    checksum += ch;
    saved_reg_values[i] = ch;
  }

  // Now see if the checksum matches
  if (EEPROM.read(0) == checksum)
  {
    // Valid, so copy values into the working table
    for (int i = TDSC_FIRMWARE_VERSION; i <= TDSC_STATUS_RETURN_LEVEL; i++)
      g_controller_registers[i] = saved_reg_values[i];
  }
  // Do minimal initialize of group move area.
  g_controller_registers[TDSC_GM_SERVO_CNT] = 0;
  g_controller_registers[TDSC_GM_SERVO_CNT_TOTAL] = 0;
  g_controller_registers[TDSC_GM_FRAME_TIME_MS] = 20;         // default 20ms
  g_controller_registers[TDSC_GM_IO_PIN_MOVE_ACTIVE] = 0xff;
  g_controller_registers[TDSC_GM_IO_PIN_MOVE_INTERPOLATE] = 0xff;
  for (int i=0; i < TDSC_MAX_GM_SIZE; i++)
    g_controller_registers[TDSC_GM_SERVO_0_ID + i] = i; // default to servos 1,2,3...

  g_controller_registers[TDSC_GM_SERVO_CNT_MOVING] = 0;
  g_controller_registers[TDSC_GM_MOVE_COMMAND] = 0;              // Move Status options

  // Single Servo Timed move Command
  g_controller_registers[TDSC_TM_SLOT] = 0xff;

}

//-----------------------------------------------------------------------------
// SaveEEPromSectionsLocalRegisters - Save updated registers out to the Teensy EEProm.
//-----------------------------------------------------------------------------
void SaveEEPromSectionsLocalRegisters(void)
{
  // Prety stupid here. simply loop and write out data.  Will also keep a checksum...
  uint8_t checksum = 0;
  for (int i = TDSC_FIRMWARE_VERSION; i <= TDSC_STATUS_RETURN_LEVEL; i++)
  {
    EEPROM.write(1 + i - TDSC_FIRMWARE_VERSION, g_controller_registers[i]);
    checksum += g_controller_registers[i];
  }
  // Lets write the Checksum
  EEPROM.write(0, checksum);
}



