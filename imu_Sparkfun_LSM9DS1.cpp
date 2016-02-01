//=============================================================================
// File: Sparkfun_LSM9DS1.cpp
//  Handle the SyncRead command
//=============================================================================

//=============================================================================
// Header Files
//=============================================================================
#include <EEPROM.h>
#include <ax12Serial.h>
#include <BioloidSerial.h>
#include "globals.h"
#include "imu.h"

//-----------------------------------------------------------------------------
// This is an optional module so check to see if we are using it here
//-----------------------------------------------------------------------------
#ifdef USE_LSM9DS1
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

//-----------------------------------------------------------------------------
// Forward references.
//-----------------------------------------------------------------------------
void imuTimerInterrupt(void);

//-----------------------------------------------------------------------------
// Define Global variables
//-----------------------------------------------------------------------------
IMU g_imu;          // our wrapper class for calling IMU functions.
LSM9DS1 lsm9ds1;    // The Sparkfun object

// Use an Interval timer to control call back...
IntervalTimer interval_timer_;


// I pull these low
#define LSM9DS1_M  0x1c
#define LSM9DS1_AG  0x6a


//-----------------------------------------------------------------------------
// IMU::begin()
//-----------------------------------------------------------------------------
boolean IMU::begin() {
  lsm9ds1.settings.device.commInterface = IMU_MODE_I2C;
  lsm9ds1.settings.device.mAddress = LSM9DS1_M;
  lsm9ds1.settings.device.agAddress = LSM9DS1_AG;

  // Set gyroscope scale to +/-2000  dps: - Arbotix Pro does +/-1600
  lsm9ds1.settings.gyro.scale = 2000 ;
  // Set gyroscope (and accel) sample rate to 14.9 Hz
  //lsm9ds1.settings.gyro.sampleRate = 1;
  // Set accelerometer scale to +/-4g
  lsm9ds1.settings.accel.scale = 4;
  // Set magnetometer scale to +/- 4g
  //lsm9ds1.settings.mag.scale = 4;
  // Set magnetometer sample rate to 0.625 Hz
  //lsm9ds1.settings.mag.sampleRate = 0;


  // The above lines will only take effect AFTER calling
  // imu.begin(), which verifies communication with the IMU
  // and turns it on.
  if (!lsm9ds1.begin()) {
#ifdef DBGSerial
    DBGSerial.println("Sparkfun LSM9DS1 IMU failed to init");
#endif
    return false;
  }

  // Lets start up interval timer to read from IMU
  interval_timer_.begin(imuTimerInterrupt, 50000);  // setup for 200 times per second...

  return true;
}

//-----------------------------------------------------------------------------
// IMU::end()
//-----------------------------------------------------------------------------
void IMU::end() {
  interval_timer_.end();
}

//-----------------------------------------------------------------------------
// IMU::update()
//-----------------------------------------------------------------------------
void IMU::update() {

  // Read Gyro and accell... Hopefully 
  lsm9ds1.readGyro();
  g_controller_registers[CM730_GYRO_Z_L] = lsm9ds1.gz & 0xff;
  g_controller_registers[CM730_GYRO_Z_H] = (lsm9ds1.gz >> 8) & 0xff;
  g_controller_registers[CM730_GYRO_Y_L] = lsm9ds1.gy & 0xff;
  g_controller_registers[CM730_GYRO_Y_H] = (lsm9ds1.gy >> 8) & 0xff;
  g_controller_registers[CM730_GYRO_X_L] = lsm9ds1.gx & 0xff;
  g_controller_registers[CM730_GYRO_X_H] = (lsm9ds1.gx >> 8) & 0xff;

  lsm9ds1.readAccel();
  g_controller_registers[CM730_ACCEL_Z_L] = lsm9ds1.az & 0xff;
  g_controller_registers[CM730_ACCEL_Z_H] = (lsm9ds1.az >> 8) & 0xff;
  g_controller_registers[CM730_ACCEL_Y_L] = lsm9ds1.ay & 0xff;
  g_controller_registers[CM730_ACCEL_Y_H] = (lsm9ds1.ay >> 8) & 0xff;
  g_controller_registers[CM730_ACCEL_X_L] = lsm9ds1.ax & 0xff;
  g_controller_registers[CM730_ACCEL_X_H] = (lsm9ds1.ax >> 8) & 0xff;

}

//-----------------------------------------------------------------------------
// Interval Timer interrupt.
//-----------------------------------------------------------------------------
void imuTimerInterrupt() {
  g_imu.update();
}

#endif // USE_LSM9DS1
