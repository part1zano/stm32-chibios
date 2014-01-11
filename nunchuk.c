#include "ch.h"
#include "hal.h"
#include "nunchuk.h"
#include "hw.h"

#ifndef NUNCHUK_I2CD
#define NUNCHUK_I2CD I2CD1
#endif

i2cflags_t nunchuk_init(void) {
	const uint8_t buf0[] = {0xf0, 0x65};
	const uint8_t buf1[] = {0xfb, 0x00};
	static uint8_t rxbuff[1];

	msg_t status = RDY_OK;

	i2cAcquireBus(&NUNCHUK_I2CD);

	status = i2cMasterTransmit(&NUNCHUK_I2CD, NUNCHUK_ADDR, buf0, 2, rxbuff, 1);
	if (status != RDY_OK) {
		i2cflags_t errors = i2cGetErrors(&NUNCHUK_I2CD);
		i2cReleaseBus(&NUNCHUK_I2CD);
		return errors; 
	}

	status = i2cMasterTransmit(&NUNCHUK_I2CD, NUNCHUK_ADDR, buf1, 2, rxbuff, 1);
	if (status != RDY_OK) {
		i2cReleaseBus(&NUNCHUK_I2CD);
		return 2;
	}


	i2cReleaseBus(&NUNCHUK_I2CD);
	return 0;
}

uint8_t * nunchuk_data(void) {
	const uint8_t txbuf[] = {0x00};
	static uint8_t rxbuf[6];


	msg_t status = RDY_OK;

	i2cAcquireBus(&NUNCHUK_I2CD);
	status = i2cMasterTransmit(&NUNCHUK_I2CD, NUNCHUK_ADDR, txbuf, 1, rxbuf, 6);
	if (status != RDY_OK) {
		i2cReleaseBus(&NUNCHUK_I2CD);
		return NULL;
	}
	i2cReleaseBus(&NUNCHUK_I2CD);
	return rxbuf;
}
