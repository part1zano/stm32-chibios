#include "ch.h"
#include "hal.h"
#include "nunchuk.h"
#include "hw.h"

#ifndef NUNCHUK_I2CD
#define NUNCHUK_I2CD I2CD1
#endif

uint8_t nunchuk_init(void) {
	const uint8_t buf0[] = {0xf0, 0x65};
	const uint8_t buf1[] = {0xfb, 0x00};
	uint8_t rxbuff[1];

	msg_t status = RDY_OK;
	systime_t tmo = MS2ST(4);

	i2cAcquireBus(&NUHCHUK_I2CD);

	status = i2cMasterTransmitTimeout(&NUNCHUK_I2CD, NUNCHUK_ADDR, buf0, 2, rxbuff, 1, tmo);
	if (status != RDY_OK) {
		i2cReleaseBus(&NUNCHUK_I2CD);
		return 1;
	}

	status = i2cMasterTransmitTimeout(&NUNCHUK_I2CD, NUNCHUK_ADDR, buf1, 2, rxbuff, 1, tmo);
	if (status != RDY_OK) {
		i2cReleaseBus(&NUNCHUK_I2CD);
		return 2;
	}


	i2cReleaseBus(&NUNCHUK_I2CD);
	return 0;
}

uint8_t[6] nunchuk_data(void) {
	uint8_t txbuf[] = {0x00};
	uint8_t rxbuf[6];

	msg_t status = RDY_OK;
	systime_t tmo = MS2ST(4);

	i2cAcquireBus(&NUNCHUK_I2CD);
	status = i2cMasterTransmitTimeout(&NUNCHUK_I2CD, NUNCHUK_ADDR, txbuf, 1, rxbuf, 6, tmo);
	if (status != RDY_OK) {
		i2cReleaseBus(&NUNCHUK_I2CD);
		return NULL;
	}
	i2cReleaseBus(&NUNCHUK_I2CD);
	return rxbuf;
}
