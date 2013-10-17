/*
   ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
   2011,2012 Giovanni Di Sirio.

   This file is part of ChibiOS/RT.

   ChibiOS/RT is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.

   ChibiOS/RT is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
   */

#include "ch.h"
#include "hal.h"
#include "test.h"
#include "spi.h"
#include "i2c.h"
#include "usbcfg.h"
#include "chprintf.h"
#include "shell.h"
#include "chthreads.h"
// testing rtc
#include "chrtclib.h"
// for compass demo
#include <math.h>
#define PI 3.1415f

#define usb_lld_connect_bus(usbp)
#define usb_lld_disconnect_bus(usbp)

/* Virtual serial port over USB.*/
SerialUSBDriver SDU1;

static float mdps_per_digit = 8.75;

uint8_t schema = 0;
#define MAXSCH 3

static const SPIConfig spi1cfg = {
	NULL,
	/* HW dependent part.*/
	GPIOE,
	GPIOE_SPI1_CS,
	SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA,
	0
};

static const I2CConfig i2cconfig = {
	0x00902025, //voodoo magic
	0,
	0
};
/*
static uint8_t readByteSPI(uint8_t reg)
{
	char txbuf[2] = {0x80 | reg, 0xFF};
	char rxbuf[2];
	spiSelect(&SPID1);
	spiExchange(&SPID1, 2, txbuf, rxbuf);
	spiUnselect(&SPID1);
	return rxbuf[1];
}
*/
static uint8_t writeByteSPI(uint8_t reg, uint8_t val)
{
	char txbuf[2] = {reg, val};
	char rxbuf[2];
	spiSelect(&SPID1);
	spiExchange(&SPID1, 2, txbuf, rxbuf);
	spiUnselect(&SPID1);
	return rxbuf[1];
}

/*
static uint8_t readByteI2C(uint8_t addr, uint8_t reg)
{
	uint8_t data;
	i2cAcquireBus(&I2CD1);
	(void)i2cMasterTransmitTimeout(&I2CD1, addr, &reg, 1, &data, 1, TIME_INFINITE);
	i2cReleaseBus(&I2CD1);
	return data;
}
*/
static void writeByteI2C(uint8_t addr, uint8_t reg, uint8_t val)
{
	uint8_t cmd[] = {reg, val};
	i2cAcquireBus(&I2CD1);
	(void)i2cMasterTransmitTimeout(&I2CD1, addr, cmd, 2, NULL, 0, TIME_INFINITE);
	i2cReleaseBus(&I2CD1);
}

static void initGyro(void)
{
	/* see the L3GD20 Datasheet */
	writeByteSPI(0x20, 0xcF);
}
static void initAccel(void)
{
	// Highest speed, enable all axes
	writeByteI2C(0x19, 0x20, 0x97);
}
static void initMag(void)
{
	// Highest speed
	writeByteI2C(0x1E, 0x00, 0x1C);
	writeByteI2C(0x1E, 0x02, 0x00);
}
static uint8_t readGyro(float* data)
{
	/* read from L3GD20 registers and assemble data */
	/* 0xc0 sets read and address increment */
	char txbuf[8] = {0xc0 | 0x27, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	char rxbuf[8];
	spiSelect(&SPID1);
	spiExchange(&SPID1, 8, txbuf, rxbuf);
	spiUnselect(&SPID1);
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
static uint8_t readAccel(float* data)
{
	// setting MSB makes it increment the address for a multiple byte read
	uint8_t start_reg = 0x27 | 0x80;
	uint8_t out[7];
	i2cAcquireBus(&I2CD1);
	msg_t f = i2cMasterTransmitTimeout(&I2CD1, 0x19, &start_reg, 1, out, 7, TIME_INFINITE);
	(void)f;
	i2cReleaseBus(&I2CD1);
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
static uint8_t readMag(float* data)
{
	uint8_t start_reg = 0x03;
	uint8_t out[7];
	i2cAcquireBus(&I2CD1);
	msg_t f = i2cMasterTransmitTimeout(&I2CD1, 0x1E, &start_reg, 1, out, 7, TIME_INFINITE);
	(void)f;
	i2cReleaseBus(&I2CD1);
	//out[6] doesn't seem to reflect actual new data, so just push out every time
	int16_t val_x = (out[0] << 8) | out[1];
	int16_t val_z = (out[2] << 8) | out[3];
	int16_t val_y = (out[4] << 8) | out[5];
	data[0] = ((float)val_x)*1.22;
	data[1] = ((float)val_y)*1.22;
	data[2] = ((float)val_z)*1.22;
	return 1;
}

#define SHELL_WA_SIZE   THD_WA_SIZE(1024)

static void cmd_test(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void) argc;
	(void) argv;
	chprintf(chp, "ChibiOS test suite\r\n");
	TestThread(chp);
}

static void cmd_gyrodata(BaseSequentialStream *chp, int argc, char *argv[]) {
	float gyrodata[3];
	uint32_t times = 5;
	uint32_t delay = 200;
	if (argc >= 1) {
		times = atoi(argv[0]);
	}
	if (argc >= 2) {
		delay = atoi(argv[1]);
	}
	uint32_t i = 0;
	chprintf(chp, " Number 1\t Number 2\t Number 3\t #\r\n");
	for (i = 0; i < times; i++) {
		if (readGyro(gyrodata)) {
			chprintf(chp, " %f\t %f\t %f\t %d\r\n", gyrodata[0], gyrodata[1], gyrodata[2], i);
		}
		chThdSleepMilliseconds(delay);
	}
}

static void cmd_magdata(BaseSequentialStream *chp, int argc, char *argv[]) {
	float magdata[3];
	uint32_t times = 5;
	uint32_t delay = 200;
	if (argc >= 1) {
		times = atoi(argv[0]);
	}
	if (argc >= 2) {
		delay = atoi(argv[1]);
	}
	uint32_t i = 0;
	chprintf(chp, " Number 1\t Number 2\t Number 3\t #\r\n");
	for (i = 0; i < times; i++) {
		if (readMag(magdata)) {
			chprintf(chp, " %f\t %f\t %f\t %d\r\n", magdata[0], magdata[1], magdata[2], i);
		}
		chThdSleepMilliseconds(delay);
	}
}

static void cmd_nextsch(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void) argc;
	(void) argv;
	if (schema < MAXSCH) {
		schema++;
	}
	else {
		schema = 0;
	}
	chprintf(chp, "Blinking schema set to %d\r\n", schema);
}

static void cmd_adjust(BaseSequentialStream *chp, int argc, char *argv[]) {
	float gyrodata[2][3];
	(void) argc;
	(void) argv;
	chprintf(chp, "Adjust the motorcycle vertically and press the User button\r\n");
	while (!palReadPad(GPIOA, GPIOA_BUTTON)) { }
	if (!readGyro(gyrodata[0])) {
		chprintf(chp, "Error getting gyrodata!\r\n");
		return;
	}
	else {
		chprintf(chp, "Got gyrodata for the 1st time!\r\n");
	}
	chThdSleepMilliseconds(5000);
	chprintf(chp, "Now place it on the side stand and press the button again\r\n");
	while (!palReadPad(GPIOA, GPIOA_BUTTON)) { }
	if (!readGyro(gyrodata[1])) {
		chprintf(chp, "Error getting gyrodata!\r\n");
		return;
	} else {
		chprintf(chp, "Got gyrodata for the 2nd time!\r\n");
	}
	chprintf(chp, "Got your gyrodata successfully\r\n");
	uint8_t i = 0;
	for (i = 0; i < 2; i++) {
		chprintf(chp, "%d\t %f\t %f\t %f\r\n", i, gyrodata[i][0], gyrodata[i][1], gyrodata[i][2]);
	}
}

static void cmd_time(BaseSequentialStream *chp, int argc, char *argv[]) {
	if (argc == 0) {
		time_t unixTime = rtcGetTimeUnixSec(&RTCD1);
		struct tm ts = *gmtime(&unixTime);
		chprintf(chp, "current rtc time: %d\r\n", unixTime);
		chprintf(chp, "which is %d-%02d-%02d %02d:%02d:%02d UTC\r\n", (1900+ts.tm_year), (1+ts.tm_mon), ts.tm_mday, ts.tm_hour, ts.tm_min, ts.tm_sec);
	} else if (argc == 1) {
		time_t newtime = atoi(argv[0]);
		rtcSetTimeUnixSec(&RTCD1, newtime);
		struct tm ts = *gmtime(&newtime);
		chprintf(chp, "New time is: %d\r\n", newtime);
		chprintf(chp, "which is %d-%02d-%02d %02d:%02d:%02d UTC\r\n", (1900+ts.tm_year), (1+ts.tm_mon), ts.tm_mday, ts.tm_hour, ts.tm_min, ts.tm_sec);

	}
}

static void cmd_mem(BaseSequentialStream *chp, int argc, char *argv[]) {
//	size_t n, size;
	(void) argv;
	(void) argc;

//	n = chHeapStatus(&size);
	chprintf(chp, "core free memory : %u bytes\r\n", chCoreStatus());
//	chprintf(chp, "heap fragments   : %u\r\n", n);
//	chprintf(chp, "heap free total  : %u bytes\r\n", size);
}

static void cmd_reboot(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void) argc;
	(void) argv;
	chprintf(chp, "Rebooting...\r\n");
	chThdSleepMilliseconds(100);
	NVIC_SystemReset();
}

static const ShellCommand shCmds[] = {
	{"test",      cmd_test},
	{"gyrodata",	cmd_gyrodata},
	{"magdata", cmd_magdata},
	{"adjust", cmd_adjust},
	{"nextsch", cmd_nextsch},
	{"time", cmd_time},
	{"free", cmd_mem},
	{"reboot", cmd_reboot},
	{NULL, NULL}
};

static const ShellConfig shCfg = {
	(BaseSequentialStream *)&SDU1,
	shCmds
};

static WORKING_AREA(waThreadBlink, 128);

static msg_t ThreadBlink(void *arg) {
	(void) arg;

	chRegSetThreadName("blinker");
	uint8_t i = GPIOE_LED4_BLUE;

	while (TRUE) { 
		if (schema == 0) { // all LEDs on, then off in the other direction
			if (i > GPIOE_LED6_GREEN + GPIOE_LED4_BLUE) {
				i = GPIOE_LED4_BLUE;
			}
			if (i <= GPIOE_LED6_GREEN) {
				palSetPad(GPIOE, i);
			}
			else {
				palClearPad(GPIOE, 31-i);
			}
			i++;
		}
		else if (schema == 1) { // All LEDs on, then off in same direction
			if (i > GPIOE_LED6_GREEN) {
				i = GPIOE_LED4_BLUE;
			}
			palTogglePad(GPIOE, i);
			i++;
		}
		else if (schema == 2) { // LED snake, length = 3
			if (i > GPIOE_LED6_GREEN) { 
				i = GPIOE_LED4_BLUE;
			}
			if (i+3 <= GPIOE_LED6_GREEN) {
				palSetPad(GPIOE, i+3);
			}
			else {
				palSetPad(GPIOE, i-5);
			}
			palClearPad(GPIOE, i);
			i++;
		}
		else if (schema == 3) { // compass
			uint8_t j;
			for (j = GPIOE_LED4_BLUE; j < GPIOE_LED6_GREEN; j++) {
				palClearPad(GPIOE, j);
			}
			float magdata[3];
			float accdata[3];
			readMag(magdata);
			readAccel(accdata);
			float fNormAcc = sqrt(pow(accdata[0], 2) + pow(accdata[1], 2) + pow(accdata[2], 2));
			float fSinRoll = -(accdata[1])/(fNormAcc);
			float fCosRoll = sqrt(1-pow(fSinRoll, 2));
			float fSinPitch = (accdata[0])/(fNormAcc);
			float fCosPitch = sqrt(1-pow(fSinPitch, 2));
			float RollAng, PitchAng;
			if (fSinRoll > 0) {
				if (fCosRoll > 0) {
					RollAng = acos(fCosRoll)*180/PI;
				}
				else {
					RollAng = acos(fCosRoll)*180/PI+180;
				}
			}
			else {
				if (fCosRoll > 0) {
					RollAng = acos(fCosRoll)*180/PI+360;
				}
				else {
					RollAng = acos(fCosRoll)*180/PI+180;
				}
			}

			if (fSinPitch > 0) {
				if (fCosPitch > 0) {
					PitchAng = acos(fCosPitch)*180/PI;
				}
				else {
					PitchAng = acos(fCosPitch)*180/PI+180;
				}
			}
			else {
				if (fCosPitch > 0) {
					PitchAng = acos(fCosPitch)*180/PI+360;
				}
				else {
					PitchAng = acos(fCosPitch)*180/PI+180;
				}
			}

			if (RollAng >= 360) {
				RollAng += 360;
			}
			if (PitchAng >= 360) {
				PitchAng += 360;
			}

			float fTiltedX = magdata[0]*fCosPitch + magdata[2]*fSinPitch;
			float fTiltedY = magdata[0]*fSinPitch + magdata[1]*fCosRoll - magdata[1]*fSinRoll*fSinPitch;
			float HeadingValue = atan2f((float)fTiltedY, (float)fTiltedX)*180.0f/PI;
			if (HeadingValue < 0) {
				HeadingValue += 360;
			}

		}
		chThdSleepMilliseconds(125);
	}
	return 0; // never returns
}

int main(void) {
	Thread *sh = NULL;

	halInit();
	chSysInit();

	shellInit();
	
	usbDisconnectBus(serusbcfg.usbp);
	chThdSleepMilliseconds(1000);
	usbStart(serusbcfg.usbp, &usbcfg);
	usbConnectBus(serusbcfg.usbp);
	
	sduObjectInit(&SDU1);
	sduStart(&SDU1, &serusbcfg);

	spiStart(&SPID1, &spi1cfg);
	i2cStart(&I2CD1, &i2cconfig);
	initGyro();
	initAccel();
	initMag();
	chThdCreateStatic(waThreadBlink, sizeof(waThreadBlink), NORMALPRIO, ThreadBlink, NULL);

    while (TRUE) {
		if (!sh) {
			sh = shellCreate(&shCfg, SHELL_WA_SIZE, NORMALPRIO);
		}
		else if (chThdTerminated(sh)) {
			chThdRelease(sh);
			sh = NULL;
		}
		chThdSleepMilliseconds(1000);
		/*
	float gyroData[3];
        float accelData[3];
        float magData[3];
        if (readGyro(gyroData) && readAccel(accelData) && readMag(magData)) {
            chprintf((BaseSequentialStream *)&SDU1, "%f %f %f ", gyroData[0], gyroData[1], gyroData[2]);
            chprintf((BaseSequentialStream *)&SDU1, "%f %f %f ", accelData[0], accelData[1], accelData[2]);
            chprintf((BaseSequentialStream *)&SDU1, "%f %f %f\n", magData[0], magData[1], magData[2]);
        }
		*/
	}
	return 0; // never returns, lol
}
