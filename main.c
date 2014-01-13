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
// for barometer
#include "bmp085.h"
// for gyro, mag & accel
#include "accelgyromag.h"
// atoi
#include <stdlib.h> 
#include "lcd5110.h"
#include "nunchuk.h"

/*
#define usb_lld_connect_bus(usbp)
#define usb_lld_disconnect_bus(usbp)
*/
// This didn't work =(^U IT'S ALIVE! ALIVE!!!
#define USB_GPIO_PORT GPIOA
#define USBDM_BIT GPIOA_USB_DM
#define USBDP_BIT GPIOA_USB_DP

void usb_lld_disconnect_bus(USBDriver *usbp)
{
	(void) usbp;
	palClearPort(USB_GPIO_PORT, (1<<USBDM_BIT) | (1<<USBDP_BIT));
	palSetPadMode(USB_GPIO_PORT, USBDM_BIT, PAL_MODE_OUTPUT_PUSHPULL);
	palSetPadMode(USB_GPIO_PORT, USBDP_BIT, PAL_MODE_OUTPUT_PUSHPULL);
}

void usb_lld_connect_bus(USBDriver *usbp)
{
	(void) usbp;
	palClearPort(USB_GPIO_PORT, (1<<USBDM_BIT) | (1<<USBDP_BIT));
	palSetPadMode(USB_GPIO_PORT, USBDM_BIT, PAL_MODE_ALTERNATE(14));
	palSetPadMode(USB_GPIO_PORT, USBDP_BIT, PAL_MODE_ALTERNATE(14));
}

/* Virtual serial port over USB.*/
SerialUSBDriver SDU1;

uint8_t schema = 0;
#define MAXSCH 2
#define POLLER_TIMEOUT 5000
#define TZ_HOURS 4
#define TZ_MINUTES 0
#define TZ_STR "MSK"

typedef struct {
	uint32_t temp;
	uint32_t press;
	time_t uTime;
} poller_data;

poller_data PollerData;

static const SPIConfig spi1cfg = {
	NULL,
	/* HW dependent part.*/
	GPIOE,
	GPIOE_SPI1_CS,
	SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA,
	0
};

static const SPIConfig spi2cfg = {
	NULL,
	GPIOB, 
	11,
	SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA,
	0
};

static const I2CConfig i2cconfig = {
	0x00902025, //voodoo magic
	0,
	0
};

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
		chThdSleepMilliseconds(delay);
		if (readGyro(gyrodata)) {
			chprintf(chp, " %f\t %f\t %f\t %d\r\n", gyrodata[0], gyrodata[1], gyrodata[2], i);
		}
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
		chThdSleepMilliseconds(delay);
		if (readMag(magdata)) {
			chprintf(chp, " %f\t %f\t %f\t %d\r\n", magdata[0], magdata[1], magdata[2], i);
		}
	}
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
		struct tm *ts = gmtime(&unixTime);
		chprintf(chp, "current rtc time: %d\r\n", unixTime);
		chprintf(chp, "which is %d-%02d-%02d %02d:%02d:%02d %s\r\n", (1900+ts->tm_year), (1+ts->tm_mon), ts->tm_mday, ts->tm_hour, ts->tm_min, ts->tm_sec, TZ_STR);
	} else if (argc == 1) {
		time_t newtime = atoi(argv[0]) + 3600*TZ_HOURS + TZ_MINUTES;
		rtcSetTimeUnixSec(&RTCD1, newtime);
		struct tm *ts = gmtime(&newtime);
		chprintf(chp, "New time is: %d\r\n", newtime);
		chprintf(chp, "which is %d-%02d-%02d %02d:%02d:%02d %s\r\n", (1900+ts->tm_year), (1+ts->tm_mon), ts->tm_mday, ts->tm_hour, ts->tm_min, ts->tm_sec, TZ_STR);
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

static void cmd_schema(BaseSequentialStream *chp, int argc, char *argv[]) {
	if (argc == 0) {
		chprintf(chp, "Current blink schema is %d\r\n", schema);
	} else {
		uint8_t newsch = atoi(argv[0]);
		if (newsch > MAXSCH) {
			newsch = schema;
		}
		schema = newsch;
		chprintf(chp, "Set blinking schema to %d\r\n", schema);
	}
}

uint8_t bmp085_status = 0;

static void cmd_pressure(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void) argc;
	(void) argv;
	
	if (bmp085_status == 0) {
		float temperature = PollerData.temp/10.0f;
		float mm = PollerData.press/133.322f;
		time_t unixTime = PollerData.uTime;
		struct tm *ts =  gmtime(& unixTime); // XXX :: possibly, an unneeded variable unixTime
		chprintf(chp, "By %d-%02d-%02d %02d:%02d:%02d %s\r\n", (1900+ts->tm_year), (1+ts->tm_mon), ts->tm_mday, ts->tm_hour, ts->tm_min, ts->tm_sec, TZ_STR);
		chprintf(chp, "Pressure is %ld Pa (%3.3f mm)\r\n", PollerData.press, mm);
		chprintf(chp, "Temperature is: %3.3f C\r\n", temperature);
	} else {
		chprintf(chp, "ERROR! bmp085 initialization returned %d\r\n", bmp085_status);
	}
}

msg_t nunchuk_status = 0;
static void cmd_chuk(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void) argc;
	(void) argv;
	uint8_t data[6];
	uint8_t i;
	uint8_t times = 100;
	msg_t status = RDY_OK;

	if (nunchuk_status == 0) {
		chprintf(chp, "#\t JX\t JY\t 3rd\r\n");
		for (i = 0; i < times; i++) {
			status = nunchuk_data(data);
			if (status == RDY_OK) {
				chprintf(chp, "%d\t %d\t %d\t %d\r\n", i, data[0], data[1], data[5]);
			}
		}
	}
	else {
		chprintf(chp, "ERROR! nunchuk returned non-zero: %d\r\n", nunchuk_status);
	}
}

static const ShellCommand shCmds[] = {
	{"test",      cmd_test},
	{"gyrodata",	cmd_gyrodata},
	{"magdata", cmd_magdata},
	{"adjust", cmd_adjust},
	{"schema", cmd_schema},
	{"time", cmd_time},
	{"free", cmd_mem},
	{"reboot", cmd_reboot},
	{"bmp", cmd_pressure},
	{"chuk", cmd_chuk},
	{NULL, NULL}
};

static const ShellConfig shCfg = {
	(BaseSequentialStream *)&SDU1,
	shCmds
};

static WORKING_AREA(waPoller, 128);
static msg_t ThreadPoller(void *arg) {
	(void) arg;

	char out[9];
	chRegSetThreadName("poller");
	while (TRUE) {
		if (bmp085_status == 0) {
			PollerData.temp = bmp085_read_temp();
			PollerData.press = bmp085_read_press();
			PollerData.uTime = rtcGetTimeUnixSec(&RTCD1);
			chsnprintf(out, sizeof(out), "%4.2f", PollerData.press/133.322);
			lcd5110SetPosXY(&SPID2, 30, 0);
			lcd5110WriteText(&SPID2, out);
			chsnprintf(out, sizeof(out), "%4.2f", PollerData.temp/10.0);
			lcd5110SetPosXY(&SPID2, 30, 1);
			lcd5110WriteText(&SPID2, out);
			chThdSleepMilliseconds(POLLER_TIMEOUT);
		}
	}

	return 0; // never returns
}

static WORKING_AREA(waThreadButton, 128);
static msg_t ThreadButton(void *arg) {
	(void) arg;

	chRegSetThreadName("button");

	while (TRUE) {
		while (!palReadPad(GPIOA, GPIOA_BUTTON)) {
		}
		if (schema == MAXSCH) {
			schema = 0;
		} else {
			schema++;
		}
		chprintf((BaseSequentialStream *)&SDU1, "Schema set to %d\r\n", schema);
		chThdSleepMilliseconds(500);
	}
	return 0; // nevar forget
}


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
		else if (schema == 3) { 
		}
		chThdSleepMilliseconds(125);
	}
	return 0; // never returns
}

int main(void) {
	Thread *sh = NULL;

	PollerData.temp = 0;
	PollerData.press = 0;
	PollerData.uTime = 0;

	halInit();
	chSysInit();

	shellInit();
	
	usbDisconnectBus(serusbcfg.usbp);
	chThdSleepMilliseconds(1000);
	usbStart(serusbcfg.usbp, &usbcfg);
	usbConnectBus(serusbcfg.usbp);
	
	sduObjectInit(&SDU1);
	sduStart(&SDU1, &serusbcfg);
// SPI-related pins (for display)
	palSetPadMode(GPIOB, 11, PAL_MODE_OUTPUT_PUSHPULL); 
	palSetPadMode(GPIOB, 10, PAL_MODE_OUTPUT_PUSHPULL);
	palSetPadMode(GPIOB, 13, PAL_MODE_ALTERNATE(5));
	palSetPadMode(GPIOB, 14, PAL_MODE_OUTPUT_PUSHPULL);
	palSetPadMode(GPIOB, 15, PAL_MODE_ALTERNATE(5));

	spiStart(&SPID1, &spi1cfg);
	spiStart(&SPID2, &spi2cfg);
	// i2c-related pins (for nunchuk)
	/*
	palSetPadMode(GPIOB, 8, PAL_MODE_ALTERNATE(4));
	palSetPadMode(GPIOB, 9, PAL_MODE_ALTERNATE(4));
	*/
	i2cStart(&I2CD1, &i2cconfig);
	initGyro();
	initAccel();
	initMag();
	bmp085_status = bmp085_init();
	nunchuk_status = nunchuk_init();
	lcd5110Init(&SPID2);
	lcd5110SetPosXY(&SPID2, 0, 0);
	lcd5110WriteText(&SPID2, "P :: ");
	lcd5110SetPosXY(&SPID2, 0, 1);
	lcd5110WriteText(&SPID2, "T :: ");
//	lcd5110SetPosXY(&SPID2, 30, 0);
//	lcd5110WriteText(&SPID2, "000");


	chThdCreateStatic(waThreadBlink, sizeof(waThreadBlink), NORMALPRIO, ThreadBlink, NULL);
	chThdCreateStatic(waThreadButton, sizeof(waThreadButton), NORMALPRIO, ThreadButton, NULL);
	chThdCreateStatic(waPoller, sizeof(waPoller), NORMALPRIO, ThreadPoller, NULL);

    while (TRUE) {
		if (!sh) {
			sh = shellCreate(&shCfg, SHELL_WA_SIZE, NORMALPRIO);
		}
		else if (chThdTerminated(sh)) {
			chThdRelease(sh);
			sh = NULL;
		}
		chThdSleepMilliseconds(1000);
	}
	return 0; // never returns, lol
}
