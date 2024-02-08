#ifndef __MPQ3326_H
#define __MPQ3326_H

#pragma pack(push, 1) //do not add padding
typedef struct {
	uint8_t  current; //max is 0x3F
	uint8_t  duty_msb;    //max is 0x0F
	uint8_t  duty_lsb;    //max is 0xFF
} mpq3326_channel_t;

typedef struct {
	uint8_t fpwm;
	uint8_t control;
	uint16_t refresh;
	uint16_t enable;
	uint16_t openfault;
	uint16_t shortfault;
	mpq3326_channel_t  channels[16];
}mpq3326_t; //thats 58 bytes

#pragma pack(pop) // disables the effect of #pragma pack from now on


#endif  /* __MPQ3326_H */
