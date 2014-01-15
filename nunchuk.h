#ifndef _NUNCHUK_H_
#define _NUNCHUK_H_
//#define NUNCHUK_ADDR 0xa4
#define NUNCHUK_ADDR 0x52
#define NUNCHUK_READ_ADDR 0xa5
#define NUNCHUK_WRITE_ADDR 0xa4


#define NUNCHUK_JX 0x00
#define NUNCHUK_JY 0x01
#define NUNCHUK_AX 0x02
#define NUNCHUK_AY 0x03
#define NUNCHUK_AZ 0x04
#define NUNCHUK_ABTN 0x05

i2cflags_t nunchuk_init(void);
msg_t nunchuk_data(uint8_t *data);
#endif // _NUNCHUK_H_
