#define I2CD I2CD1
#define SPID SPID1
void initGyro(void);
void initMag(void);
void initAccel(void);

uint8_t readGyro(float *data);
uint8_t readAccel(float *data);
uint8_t readMag(float *data);

