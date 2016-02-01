#ifndef _IMU_H_
#define _IMU_H_

//==================================================================
// Define our IMU class, that we will call to handle maybe a couple
// of different IMU boards.  In particular the Sparkfun one and
// the Adafruit one 
//==================================================================
class IMU
{
  public:
    boolean begin(void);
    void end(void);       

    void update(void); 
};

extern IMU g_imu;

#endif //_IMU_H_
