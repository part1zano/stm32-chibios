//------------------------------------------------
// BMP085 Register map
// Author: Roberto Marino <formica@member.fsf.org>
// Date: Jul 25 2011
//------------------------------------------------
#ifndef _BMP085_H_
#define _BMP085_H_
#include <stdint.h>

#define BMP_ADDR 	0x77	//I2c address of BMP085 without R/W bit
#define BMP_CR 		0xF4	//Control register's address

#define BMP_MODE_TEMP	0x2E	//Temperature measuring: Max conversion time 4.5ms
#define BMP_MODE_PR0	0x34	//Pressure measuring:    Max conversion time 4.5ms
#define BMP_MODE_PR1	0x74	//Pressure measuring:	 Max conversion time 7.5ms
#define BMP_MODE_PR2	0xB4	//Pressure measuring:	 Max conversion time 13.5ms
#define	BMP_MODE_PR3	0xF4	//Pressure measuring: 	 Max conversion time 25.5ms

//---- DATA REGISTER ----
#define BMP_DATA	0xF6
#define BMP_DATA_MSB	0xF6	
#define BMP_DATA_LSB	0xF7
#define BMP_DATA_XLSB	0xF8

//---- CALIBRATION REGISTER ADDRESS ----
#define BMP_AC1_MSB	0xAA
#define BMP_AC1_LSB	0xAB
#define BMP_AC2_MSB	0xAC
#define BMP_AC2_LSB	0xAD
#define BMP_AC3_MSB	0xAE
#define BMP_AC3_LSB	0xAF
#define BMP_AC4_MSB	0xB0
#define BMP_AC4_LSB	0xB1
#define BMP_AC5_MSB	0xB2
#define BMP_AC5_LSB	0xB3
#define BMP_AC6_MSB	0xB4
#define BMP_AC6_LSB	0xB5
#define	BMP_B1_MSB	0xB6
#define BMP_B1_LSB	0xB7
#define BMP_B2_MSB	0xB8
#define	BMP_B2_LSB	0xB9
#define BMP_MB_MSB	0xBA
#define BMP_MB_LSB	0xBB
#define BMP_MC_MSB	0xBC
#define BMP_MC_LSB	0xBD
#define BMP_MD_MSB	0xBE
#define BMP_MD_LSB	0xBF

#define BMP_AC1		0xAA
#define BMP_AC2		0xAC
#define BMP_AC3		0xAE
#define BMP_AC4		0xB0
#define BMP_AC5		0xB2
#define BMP_AC6		0xB4
#define	BMP_B1		0xB6
#define BMP_B2		0xB8
#define BMP_MB		0xBA
#define BMP_MC		0xBC
#define BMP_MD		0xBE

typedef struct bmp085_eprom_param
{
	int16_t ac1;
	int16_t ac2;
	int16_t ac3;
	uint16_t ac4;
	uint16_t ac5;
	uint16_t ac6;
	int16_t b1;
	int16_t b2;
	int16_t mb;
	int16_t mc;
	int16_t md;
	int32_t b5;
} bmp085_param;

int bmp085_init(void);
int32_t bmp085_read_temp(void);
int32_t bmp085_read_press(void);



#endif // _BMP085_H_
