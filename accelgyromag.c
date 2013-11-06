#include "ch.h"
#include "hal.h"
#include "accelgyromag.h"

#define I2CD I2CD1
#define SPID SPID1

static float mdps_per_digit = 8.75;

uint8_t readByteSPI(uint8_t reg)
{
	char txbuf[2] = {0x80 | reg, 0xFF};
	char rxbuf[2];
	spiSelect(&SPID);
	spiExchange(&SPID, 2, txbuf, rxbuf);
	spiUnselect(&SPID);
	return rxbuf[1];
}

uint8_t writeByteSPI(uint8_t reg, uint8_t val)
{
	char txbuf[2] = {reg, val};
	char rxbuf[2];
	spiSelect(&SPID);
	spiExchange(&SPID, 2, txbuf, rxbuf);
	spiUnselect(&SPID);
	return rxbuf[1];
}


uint8_t readByteI2C(uint8_t addr, uint8_t reg)
{
	uint8_t data;
	i2cAcquireBus(&I2CD);
	(void)i2cMasterTransmitTimeout(&I2CD, addr, &reg, 1, &data, 1, TIME_INFINITE);
	i2cReleaseBus(&I2CD);
	return data;
}

void writeByteI2C(uint8_t addr, uint8_t reg, uint8_t val)
{
	uint8_t cmd[] = {reg, val};
	i2cAcquireBus(&I2CD);
	(void)i2cMasterTransmitTimeout(&I2CD, addr, cmd, 2, NULL, 0, TIME_INFINITE);
	i2cReleaseBus(&I2CD);
}

void initGyro(void)
{
	/* see the L3GD20 Datasheet */
	writeByteSPI(0x20, 0xcF);
}
void initAccel(void)
{
	// Highest speed, enable all axes
	writeByteI2C(0x19, 0x20, 0x97);
}
void initMag(void)
{
	// Highest speed
	writeByteI2C(0x1E, 0x00, 0x1C);
	writeByteI2C(0x1E, 0x02, 0x00);
}
uint8_t readGyro(float* data)
{
	/* read from L3GD20 registers and assemble data */
	/* 0xc0 sets read and address increment */
	char txbuf[8] = {0xc0 | 0x27, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	char rxbuf[8];
	spiSelect(&SPID);
	spiExchange(&SPID, 8, txbuf, rxbuf);
	spiUnselect(&SPID);
	if (rxbuf[1] & 0x7) {
		int16_t val_x = (rxbuf[3] << 8) | rxbuf[2];
		int16_t val_y = (rxbuf[5] << 8) | rxbuf[4];
		int16_t val_z = (rxbuf[7] << 8) | rxbuf[6];
		data[0] = (((float)val_x) * mdps_per_digit)/1000.0;
		data[1] = (((float)val_y) * mdps_per_digit)/1000.0;
		data[2] = (((float)val_z) * mdps_per_digit)/1000.0;
		return 1;
	}
	return 0;
}
uint8_t readAccel(float* data)
{
	// setting MSB makes it increment the address for a multiple byte read
	uint8_t start_reg = 0x27 | 0x80;
	uint8_t out[7];
	i2cAcquireBus(&I2CD);
	msg_t f = i2cMasterTransmitTimeout(&I2CD, 0x19, &start_reg, 1, out, 7, TIME_INFINITE);
	if (f != RDY_OK) {
		i2cReleaseBus(&I2CD);
		return 0;
	}
	i2cReleaseBus(&I2CD);
	if (out[0] & 0x8) {
		int16_t val_x = (out[2] << 8) | out[1];
		int16_t val_y = (out[4] << 8) | out[3];
		int16_t val_z = (out[6] << 8) | out[5];
		// Accel scale is +- 2.0g
		data[0] = ((float)val_x)*(4.0/(65535.0))*9.81;
		data[1] = ((float)val_y)*(4.0/(65535.0))*9.81;
		data[2] = ((float)val_z)*(4.0/(65535.0))*9.81;
		return 1;
	}
	return 0;
}
uint8_t readMag(float* data)
{
	uint8_t start_reg = 0x03;
	uint8_t out[7];
	i2cAcquireBus(&I2CD);
	msg_t f = i2cMasterTransmitTimeout(&I2CD, 0x1E, &start_reg, 1, out, 7, TIME_INFINITE);
	if (f != RDY_OK) {
		i2cReleaseBus(&I2CD);
		return 0;
	}
	i2cReleaseBus(&I2CD);
	//out[6] doesn't seem to reflect actual new data, so just push out every time
	int16_t val_x = (out[0] << 8) | out[1];
	int16_t val_z = (out[2] << 8) | out[3];
	int16_t val_y = (out[4] << 8) | out[5];
	data[0] = ((float)val_x)*1.22;
	data[1] = ((float)val_y)*1.22;
	data[2] = ((float)val_z)*1.22;
	return 1;
}


