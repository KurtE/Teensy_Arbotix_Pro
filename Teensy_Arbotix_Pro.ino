//=============================================================================
//Project Teensy Arbotix Pro
//Description: To have an Teensy 3.1/3.2, semi emulate an Arbotix Pro or 
//      Robotis CM730
//=============================================================================

//=============================================================================
// Header Files
//=============================================================================

#include <EEPROM.h>

#include <ax12Serial.h>
#include <BioloidSerial.h>
#include "globals.h"
#include "imu.h"

#if NEOPIXEL_PIN
#include <Adafruit_NeoPixel.h>
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, NEOPIXEL_PIN, NEO_RGB + NEO_KHZ800);
#endif

//=============================================================================
//[CONSTANTS]
//=============================================================================

//=============================================================================
//[Globals]
//=============================================================================
uint8_t ax_state = AX_SEARCH_FIRST_FF; // current state of the Dynamixel packet parser state machine

uint8_t rxbyte[AX_SYNC_READ_MAX_DEVICES + 8]; // buffer where currently processed data are stored when looking for a Dynamixel packet, with enough space for longest possible sync read request
uint8_t rxbyte_count = 0;   // number of used bytes in rxbyte buffer
unsigned long last_message_time;
uint8_t g_passthrough_mode;

//unsigned long baud = 1000000;
unsigned long baud = 2000000;
IntervalTimer interval_timer_background_;


//-----------------------------------------------------------------------------
// setup - Main Arduino setup function
//-----------------------------------------------------------------------------
void setup()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWriteFast(LED_PIN, LOW);
#ifdef LED2_PIN
  pinMode(LED2_PIN, OUTPUT);
  digitalWriteFast(LED2_PIN, LOW);
#endif
  // Temporary Debug stuff
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(32, OUTPUT);
  pinMode(5, OUTPUT);
#ifdef DBGSerial
  delay(2000);
  DBGSerial.begin(115200);
  delay(1000);
  DBGSerial.println("Teensy Arbotix Pro Start");
#endif  
#ifdef NEOPIXEL_PIN
  strip.begin();
  strip.setPixelColor(0, 0x80, 0, 0 );
  strip.show(); // Initialize all pixels to 'off'
  delay(250);
  strip.setPixelColor(0, 0, 0x80, 0 );
  strip.show(); // Initialize all pixels to 'off'
  delay(250);
  strip.setPixelColor(0, 0, 0, 0x80 );
  strip.show(); // Initialize all pixels to 'off'
  
#endif

  pinMode(AX_BUS_POWER_PIN, OUTPUT);
  digitalWrite(AX_BUS_POWER_PIN, LOW);    // Start off with Servo power off.
  
  PCSerial.begin(baud);	// USB, communication to PC or Mac
  ax12Init(1000000, &HWSERIAL, SERVO_DIRECTION_PIN);
  
  setAXtoTX(false);
  InitalizeRegisterTable(); 

  // clear out USB Input queue
  FlushUSBInputQueue();

  // Make sure some of the output state matches our registers
  UpdateHardwareAfterLocalWrite(CM730_LED_PANEL, CM730_LED_HEAD_H-CM730_LED_PANEL+1);


  // Start up our background task
  interval_timer_background_.begin(BackgroundTimerInterrupt, 100000);  // setup for 100 times per second...


#ifdef USE_LSM9DS1
  // Try to startup imu
  g_imu.begin();
#endif   
}

//-----------------------------------------------------------------------------
// loop - Main Arduino loop function. 
//-----------------------------------------------------------------------------
void loop()
{

  digitalWriteFast(11,  HIGH);
  bool did_something = ProcessInputFromUSB();
  digitalWriteFast(11,  LOW);
//  yield();  // Give a chance for other things to happen

  // Call off to process any input that we may have received from the AXBuss
  digitalWriteFast(32,  HIGH);
  did_something |= ProcessInputFromAXBuss();
  digitalWriteFast(32,  LOW);
//  yield();

  // If we did not process any data input from USB or from AX Buss, maybe we should flush anything we have 
  // pending to go back to main processor
#if 0
  if (!did_something) 
  {
    MaybeFlushUSBOutputData();
    digitalWriteFast(5,  HIGH);
    CheckBatteryVoltage();
    digitalWriteFast(5,  LOW);
  }
#endif  
}


void BackgroundTimerInterrupt()
{
    digitalWriteFast(5,  HIGH);
    CheckBatteryVoltage();
    digitalWriteFast(5,  LOW);
}




