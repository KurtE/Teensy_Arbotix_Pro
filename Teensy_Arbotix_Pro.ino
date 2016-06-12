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
unsigned long baud = 1000000;
IntervalTimer interval_timer_background_;


//-----------------------------------------------------------------------------
// setup - Main Arduino setup function
//-----------------------------------------------------------------------------
void setup()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWriteFast(LED_PIN, HIGH);
#ifdef LED2_PIN
  pinMode(LED2_PIN, OUTPUT);
  digitalWriteFast(LED2_PIN, LOW);
#endif

  // Temporary Debug stuff
#ifdef USE_DEBUG_IOPINS
  pinMode(DEBUG_PIN_USB_INPUT, OUTPUT);
  pinMode(DEBUG_PIN_SEND_STATUS_PACKET, OUTPUT);
  pinMode(DEBUG_PIN_AX_INPUT, OUTPUT);
  pinMode(DEBUG_PIN_BACKGROUND, OUTPUT);
  pinMode(DEBUG_PIN_FLUSH_TO_HOST, OUTPUT);
  pinMode(DEBUG_PIN_AX_TO_HOST, OUTPUT);
#endif
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

  // Hack define which
#ifdef HWSerial_TXPIN
  pinMode(HWSerial_TXPIN, INPUT_PULLUP);
#endif

  pinMode(AX_BUS_POWER_PIN, OUTPUT);
  digitalWrite(AX_BUS_POWER_PIN, LOW);    // Start off with Servo power off.

  PCSerial.begin(baud);	// USB, communication to PC or Mac
  ax12Init(1000000, &HWSERIAL, SERVO_DIRECTION_PIN);

  setAXtoTX();
  InitalizeRegisterTable();

  // clear out USB Input queue
  FlushUSBInputQueue();

  // Make sure some of the output state matches our registers
  UpdateHardwareAfterLocalWrite(TDSC_LED_PANEL, TDSC_LED_HEAD_H - TDSC_LED_PANEL + 1);

  // Double bugbug... Maybe some get changed Temporary Debug stuff
#ifdef USE_DEBUG_IOPINS
  pinMode(DEBUG_PIN_USB_INPUT, OUTPUT);
  pinMode(DEBUG_PIN_SEND_STATUS_PACKET, OUTPUT);
  pinMode(DEBUG_PIN_AX_INPUT, OUTPUT);
  pinMode(DEBUG_PIN_BACKGROUND, OUTPUT);
  pinMode(DEBUG_PIN_FLUSH_TO_HOST, OUTPUT);
  pinMode(DEBUG_PIN_AX_TO_HOST, OUTPUT);
#endif

  // Start up our background task
  interval_timer_background_.begin(BackgroundTimerInterrupt, 100000);  // setup for 100 times per second...


#ifdef USE_LSM9DS1
  // Try to startup imu
  g_imu.begin();
#endif

  for (int i = 0; i < 2; i++) {
    digitalWriteFast(LED_PIN, HIGH);
    delay(250);
    digitalWriteFast(LED_PIN, LOW);
    delay(250);
  }

}

//-----------------------------------------------------------------------------
// loop - Main Arduino loop function.
//-----------------------------------------------------------------------------
void loop()
{

  debug_digitalWrite( DEBUG_PIN_USB_INPUT,  HIGH);
  bool did_something = ProcessInputFromUSB();
  debug_digitalWrite( DEBUG_PIN_USB_INPUT,  LOW);
  //  yield();  // Give a chance for other things to happen

  // Call off to process any input that we may have received from the AXBuss
  debug_digitalWrite( DEBUG_PIN_AX_INPUT,  HIGH);
  did_something |= ProcessInputFromAXBuss();
  debug_digitalWrite( DEBUG_PIN_AX_INPUT,  LOW);
  //  yield();
  // See if we need to do any Interpolation.
  PoseInterpolateStepTask();
  // If we did not process any data input from USB or from AX Buss, maybe we should flush anything we have
  // pending to go back to main processor
#if 0
  if (!did_something)
  {
    MaybeFlushDataBackToHost();
    debug_digitalWrite( DEBUG_PIN_BACKGROUND,  HIGH);
    CheckBatteryVoltage();
    debug_digitalWrite( DEBUG_PIN_BACKGROUND,  LOW);
  }
#endif
}

uint8_t g_Timer_loop_count = 0;
void BackgroundTimerInterrupt()
{
  debug_digitalWrite( DEBUG_PIN_BACKGROUND,  HIGH);
  CheckBatteryVoltage();

  g_Timer_loop_count++;
  if (g_Timer_loop_count == 0xff)
  {
    // Hack since reset button may not have pogo pin...
    CheckHardwareForLocalReadRequest(TDSC_BUTTON, 1);
    if ( g_controller_registers[TDSC_BUTTON] == 0x3)
      SoftwareReset();
  }
  debug_digitalWrite(5,  LOW);
}

void SoftwareReset() {
#define SCB_AIRCR (*(volatile uint32_t *)0xE000ED0C) // Application Interrupt and Reset Control location
  //  SCB_AIRCR = 0x05FA0004;  //value provided by P. Stoffregen, Beerware License Thank you!
}




