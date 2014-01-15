#include "ch.h"
#include "hal.h"
#include "nunchuk.h"
#include "hw.h"

#ifndef NUNCHUK_I2CD
#define NUNCHUK_I2CD I2CD1
#endif

i2cflags_t nunchuk_init(void) {
/*
	const uint8_t buf0[] = {0xf0, 0x65};
	const uint8_t buf1[] = {0xfb, 0x00};
*/
	const uint8_t buf1[] = {0x40, 0x00};

	msg_t status = RDY_OK;

	i2cAcquireBus(&NUNCHUK_I2CD);
/*
	status = i2cMasterTransmit(&NUNCHUK_I2CD, NUNCHUK_ADDR, buf0, 2, NULL, 0);
	if (status != RDY_OK) {
		i2cflags_t errors = i2cGetErrors(&NUNCHUK_I2CD);
		i2cReleaseBus(&NUNCHUK_I2CD);
		return errors; 
	}
*/
	status = i2cMasterTransmit(&NUNCHUK_I2CD, NUNCHUK_ADDR, buf1, 2, NULL, 0);
	if (status != RDY_OK) {
		i2cReleaseBus(&NUNCHUK_I2CD);
		return 2;
	}


	i2cReleaseBus(&NUNCHUK_I2CD);
	return 0;
}

msg_t nunchuk_data(uint8_t *data) {
	const uint8_t txbuf[] = {0x00};
	uint8_t rxbuf[6];
	uint8_t first_addr = 0x00;


	msg_t status = RDY_OK;

	i2cAcquireBus(&NUNCHUK_I2CD);
	status = i2cMasterTransmit(&NUNCHUK_I2CD, NUNCHUK_ADDR, txbuf, 1, NULL, 0);
	if (status != RDY_OK) {
		i2cReleaseBus(&NUNCHUK_I2CD);
		return status;
	}
	chThdSleepMilliseconds(200);
	status = i2cMasterTransmit(&NUNCHUK_I2CD, NUNCHUK_ADDR, NULL, 0, rxbuf, 6);
	if (status != RDY_OK) {
		i2cReleaseBus(&NUNCHUK_I2CD);
		return status;
	}
	i2cReleaseBus(&NUNCHUK_I2CD);

	uint8_t i;
	for (i = 0; i < 6; i++) {
		data[i] = rxbuf[i];
	}
	return 0;
}
