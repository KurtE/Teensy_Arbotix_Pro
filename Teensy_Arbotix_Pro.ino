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

unsigned long baud = 1000000;


//-----------------------------------------------------------------------------
// setup - Main Arduino setup function
//-----------------------------------------------------------------------------
void setup()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
#ifdef LED2_PIN
  pinMode(LED2_PIN, OUTPUT);
  digitalWrite(LED2_PIN, LOW);
#endif
  // Temporary Debug stuff
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
#ifdef DBGSerial
  delay(2000);
  DBGSerial.begin(115200);
  delay(1000);
  DBGSerial.println("Teensy Arbotix Pro Start");
#endif  
#ifdef NEOPIXEL_PIN
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
#endif

  pinMode(AX_BUS_POWER_PIN, OUTPUT);
  digitalWrite(AX_BUS_POWER_PIN, LOW);    // Start off with Servo power off.
  
  Serial.begin(baud);	// USB, communication to PC or Mac
  ax12Init(1000000, &HWSERIAL);
  setAXtoTX(false);
  InitalizeRegisterTable(); 

  // clear out USB Input queue
  FlushUSBInputQueue();

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

  digitalWrite(11,  HIGH);
  bool did_something = ProcessInputFromUSB();
  digitalWrite(11,  LOW);
  yield();  // Give a chance for other things to happen

  // Call off to process any input that we may have received from the AXBuss
  digitalWrite(4,  HIGH);
  did_something |= ProcessInputFromAXBuss();
  digitalWrite(4,  LOW);
  yield();

  // If we did not process any data input from USB or from AX Buss, maybe we should flush anything we have 
  // pending to go back to main processor
  if (!did_something) 
  {
    MaybeFlushUSBOutputData();
    digitalWrite(5,  HIGH);
    CheckBatteryVoltage();
    digitalWrite(5,  LOW);
  }
  yield();
}





