/*
    FreeCopter IMU firmware - Copyright (C) 2012, +inf
			      Roberto Marino

    FreeCopter IMU is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    FreeCopter IMU is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
	    
    FreeCopter is based on

    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012 Giovanni Di Sirio.

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

                                      ---

    A special exception to the GPL can be applied should you wish to distribute
    a combined work that includes ChibiOS/RT, without being obliged to provide
    the source code for any proprietary components. See the file exception.txt
    for full details of how and when the exception can be applied.
*/



#include <stdlib.h>
#include <ch.h>
#include <hal.h>

extern SDU1;
#define I2CD I2CD2 // ugly, but should work

#include "bmp085.h"

static bmp085_param param;

static i2cflags_t errors[11] = {0,0,0,0,0,0,0,0,0,0,0};
static i2cflags_t t_errors[2] = {0,0};
static i2cflags_t p_errors[2] = {0,0};

int bmp085_init(void)
{
	msg_t status = RDY_OK;
	uint8_t buffer_tx;
	uint8_t buffer_rx[2];
	systime_t tmo = MS2ST(4);

	i2cAcquireBus(&I2CD);

	buffer_tx = BMP_AC1;
	status = i2cMasterTransmitTimeout(&I2CD,BMP_ADDR,&buffer_tx,1,buffer_rx,2,tmo);	
	param.ac1 = ((buffer_rx[0] << 8) | buffer_rx[1]);
	if (status != RDY_OK){
		errors[0] = i2cGetErrors(&I2CD);
		i2cReleaseBus(&I2CD);
		return 1;
	}

	buffer_tx = BMP_AC2;
	status = i2cMasterTransmitTimeout(&I2CD,BMP_ADDR,&buffer_tx,1,buffer_rx,2,tmo);	
	param.ac2 = ((buffer_rx[0] << 8) | buffer_rx[1]);
	if (status != RDY_OK){
		errors[1] = i2cGetErrors(&I2CD);
		i2cReleaseBus(&I2CD);
		return 2;
	}

	buffer_tx = BMP_AC3;
	status = i2cMasterTransmitTimeout(&I2CD,BMP_ADDR,&buffer_tx,1,buffer_rx,2,tmo);	
	param.ac3 = ((buffer_rx[0] << 8) | buffer_rx[1]);
	if (status != RDY_OK){
		errors[2] = i2cGetErrors(&I2CD);
		i2cReleaseBus(&I2CD);
		return 3;
	}

	buffer_tx = BMP_AC4;
	status = i2cMasterTransmitTimeout(&I2CD,BMP_ADDR,&buffer_tx,1,buffer_rx,2,tmo);	
	param.ac4 = ((buffer_rx[0] << 8) | buffer_rx[1]);
	if (status != RDY_OK){
		errors[3] = i2cGetErrors(&I2CD);
		i2cReleaseBus(&I2CD);
		return 4;
	}

	buffer_tx = BMP_AC5;
	status = i2cMasterTransmitTimeout(&I2CD,BMP_ADDR,&buffer_tx,1,buffer_rx,2,tmo);	
	param.ac5 = ((buffer_rx[0] << 8) | buffer_rx[1]);
	if (status != RDY_OK){
		errors[4] = i2cGetErrors(&I2CD);
		i2cReleaseBus(&I2CD);
		return 5;
	}

	buffer_tx = BMP_AC6;
	status = i2cMasterTransmitTimeout(&I2CD,BMP_ADDR,&buffer_tx,1,buffer_rx,2,tmo);	
	param.ac6 = ((buffer_rx[0] << 8) | buffer_rx[1]);
	if (status != RDY_OK){
		errors[5] = i2cGetErrors(&I2CD);
		i2cReleaseBus(&I2CD);
		return 6;
	}

	buffer_tx = BMP_B1;
	status = i2cMasterTransmitTimeout(&I2CD,BMP_ADDR,&buffer_tx,1,buffer_rx,2,tmo);	
	param.b1 = ((buffer_rx[0] << 8) | buffer_rx[1]);
	if (status != RDY_OK){
		errors[6] = i2cGetErrors(&I2CD);
		i2cReleaseBus(&I2CD);
		return 7;
	}

	buffer_tx = BMP_B2;
	status = i2cMasterTransmitTimeout(&I2CD,BMP_ADDR,&buffer_tx,1,buffer_rx,2,tmo);	
	param.b2 = ((buffer_rx[0] << 8) | buffer_rx[1]);
	if (status != RDY_OK){
		errors[7] = i2cGetErrors(&I2CD);
		i2cReleaseBus(&I2CD);
		return 8;
	}

	buffer_tx = BMP_MB;
	status = i2cMasterTransmitTimeout(&I2CD,BMP_ADDR,&buffer_tx,1,buffer_rx,2,tmo);	
	param.mb = ((buffer_rx[0] << 8) | buffer_rx[1]);
	if (status != RDY_OK){
		errors[8] = i2cGetErrors(&I2CD);
		i2cReleaseBus(&I2CD);
		return 9;
	}

	buffer_tx = BMP_MC;
	status = i2cMasterTransmitTimeout(&I2CD,BMP_ADDR,&buffer_tx,1,buffer_rx,2,tmo);	
	param.mc = ((buffer_rx[0] << 8) | buffer_rx[1]);
	if (status != RDY_OK){
		errors[9] = i2cGetErrors(&I2CD);
		i2cReleaseBus(&I2CD);
		return 10;
	}

	buffer_tx = BMP_MD;
	status = i2cMasterTransmitTimeout(&I2CD,BMP_ADDR,&buffer_tx,1,buffer_rx,2,tmo);	
	param.md = ((buffer_rx[0] << 8) | buffer_rx[1]);
	if (status != RDY_OK){
		errors[10] = i2cGetErrors(&I2CD);
		i2cReleaseBus(&I2CD);
		return 11;
	}

	i2cReleaseBus(&I2CD);

	return 0;
}


int32_t bmp085_read_temp(void)
{
	int32_t utemp;
	int32_t x1,x2;
	int32_t temperature = 0;

	msg_t status = RDY_OK;
	uint8_t buffer_tx[2];
	uint8_t buffer_rx[2];
	systime_t tmo = MS2ST(4);

	// Read from I2C BUS
	i2cAcquireBus(&I2CD);
	buffer_tx[0] = BMP_CR;
	buffer_tx[1] = BMP_MODE_TEMP;
	status = i2cMasterTransmitTimeout(&I2CD,BMP_ADDR,buffer_tx,2,NULL,0,tmo);
	if (status != RDY_OK){
		t_errors[0] = i2cGetErrors(&I2CD);
		i2cReleaseBus(&I2CD);
		return 273;
	}
	buffer_tx[0] = BMP_DATA;
	status = i2cMasterTransmitTimeout(&I2CD,BMP_ADDR,&buffer_tx[0],1,buffer_rx,2,tmo);
	if (status != RDY_OK){
		t_errors[1] = i2cGetErrors(&I2CD);
		i2cReleaseBus(&I2CD);
		return 275;
	}

	i2cReleaseBus(&I2CD);

	// Building value
	utemp = (int32_t)((buffer_rx[0] << 8) | buffer_rx[1]);

	// Converting value
	x1 = ((utemp - param.ac6) * param.ac5)/32768;
	x2 = (param.mc*2048) / (x1 + param.md);
	param.b5 = x1 + x2;
	temperature = (param.b5 + 8) / 16;
	return temperature;
}

int32_t bmp085_read_press(int8_t cr_value)
{
	int32_t upress;
	int32_t x1,x2,x3;
	int32_t b3,b6;
	uint32_t b4,b7;
	uint8_t oss = 3;
	int32_t pressure = 0;
	
	msg_t status = RDY_OK;
	uint8_t buffer_tx[2];
	uint8_t buffer_rx[3];
	systime_t tmo = MS2ST(4);
	
	// Reading from I2C BUS
	i2cAcquireBus(&I2CD);
	buffer_tx[0] = BMP_CR;
	buffer_tx[1] = BMP_MODE_PR0+(oss<<6);
	status = i2cMasterTransmitTimeout(&I2CD, BMP_ADDR, buffer_tx, 2, NULL, 0, tmo);
	if (status != RDY_OK) {
		p_errors[0] = i2cGetErrors(&I2CD);
		i2cReleaseBus(&I2CD);
		return 22;
	}
	
	buffer_tx[0] = BMP_DATA;
	status = i2cMasterTransmitTimeout(&I2CD, BMP_ADDR, &buffer_tx[0], 1, buffer_rx, 3, tmo);
	if (status != RDY_OK) {
		p_errors[1] = i2cGetErrors(&I2CD);
		i2cReleaseBus(&I2CD);
		return 2;
	}
	i2cReleaseBus(&I2CD);
	
	// Building value
	/*
	oss = (cr_value - 0x34) >> 6;*/
	upress = (int32_t)((buffer_rx[0] << 16) | (buffer_rx[1] << 8) | buffer_rx[2]);
	upress = upress >> (8-oss);/*
	return upress;*/

	// Converting value
	b6 = param.b5 - 4000;
	x1 = (param.b2 * ((b6 * b6)/4096))/2048;
	x2 = (param.ac2 * b6)/2048;
	x3 = x1 + x2;
	b3 = ((param.ac1 * 4 + x3) << oss + 2)/4;
	x1 = ((param.ac3)*b6)/8192;
	x2 = (param.b1 * (b6*b6/4096))/65536;
	x3 = ((x1 + x2) + 2)/4;
	b4 = param.ac4 * (uint32_t)(x3 + 32768)/32768;
	b7 = ((uint32_t)upress - b3)*(50000 >> oss);
	if (b7 < 0x80000000) {
		pressure = (b7*2)/b4;
	}
	else {
		pressure = (b7/b4)*2;
	}
	x1 = (pressure/256)*(pressure/256);
	x1 = (x1*3038)/65536;
	x2 = (-7357*pressure)/65536;
	pressure = pressure + (x1 + x2 + 3791)/16;
	pressure = (uint32_t) (pressure * 3.3118); // an ugly and dirty hack, but that's all I could think of
	// Actually, lack of calibration matters here :(
	return pressure;
}	

