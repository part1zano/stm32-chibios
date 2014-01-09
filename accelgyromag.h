#ifndef _ACCELGYROMAG_H_
#define _ACCELGYROMAG_H_
void initGyro(void);
void initMag(void);
void initAccel(void);

uint8_t readGyro(float *data);
uint8_t readAccel(float *data);
uint8_t readMag(float *data);
#endif // _ACCELGYROMAG_H_

