/*
 * asciihelper.c
 *
 *  Created on: Jul 30, 2024
 *      Author: alexis.tetar
 */

#include "keycodes.h"

const char ascii_to_hid_Key_map[95][2] = {
    {0, Key_SPACE}, {Key_MOD_LSHIFT, Key_1}, {Key_MOD_LSHIFT, Key_APOSTROPHE},
    {Key_MOD_LSHIFT, Key_3}, {Key_MOD_LSHIFT, Key_4}, {Key_MOD_LSHIFT, Key_5},
    {Key_MOD_LSHIFT, Key_7}, {0, Key_APOSTROPHE}, {Key_MOD_LSHIFT, Key_9},
    {Key_MOD_LSHIFT, Key_0}, {Key_MOD_LSHIFT, Key_8}, {Key_MOD_LSHIFT, Key_EQUAL},
    {0, Key_COMMA}, {0, Key_MINUS}, {0, Key_DOT}, {0, Key_SLASH}, {0, Key_0},
    {0, Key_1}, {0, Key_2}, {0, Key_3}, {0, Key_4}, {0, Key_5}, {0, Key_6},
    {0, Key_7}, {0, Key_8}, {0, Key_9}, {Key_MOD_LSHIFT, Key_SEMICOLON},
    {0, Key_SEMICOLON}, {Key_MOD_LSHIFT, Key_COMMA}, {0, Key_EQUAL},
    {Key_MOD_LSHIFT, Key_DOT}, {Key_MOD_LSHIFT, Key_SLASH}, {Key_MOD_LSHIFT, Key_2},
    {Key_MOD_LSHIFT, Key_A}, {Key_MOD_LSHIFT, Key_B}, {Key_MOD_LSHIFT, Key_C},
    {Key_MOD_LSHIFT, Key_D}, {Key_MOD_LSHIFT, Key_E}, {Key_MOD_LSHIFT, Key_F},
    {Key_MOD_LSHIFT, Key_G}, {Key_MOD_LSHIFT, Key_H}, {Key_MOD_LSHIFT, Key_I},
    {Key_MOD_LSHIFT, Key_J}, {Key_MOD_LSHIFT, Key_K}, {Key_MOD_LSHIFT, Key_L},
    {Key_MOD_LSHIFT, Key_M}, {Key_MOD_LSHIFT, Key_N}, {Key_MOD_LSHIFT, Key_O},
    {Key_MOD_LSHIFT, Key_P}, {Key_MOD_LSHIFT, Key_Q}, {Key_MOD_LSHIFT, Key_R},
    {Key_MOD_LSHIFT, Key_S}, {Key_MOD_LSHIFT, Key_T}, {Key_MOD_LSHIFT, Key_U},
    {Key_MOD_LSHIFT, Key_V}, {Key_MOD_LSHIFT, Key_W}, {Key_MOD_LSHIFT, Key_X},
    {Key_MOD_LSHIFT, Key_Y}, {Key_MOD_LSHIFT, Key_Z}, {0, Key_LEFTBRACE},
    {0, Key_BACKSLASH}, {0, Key_RIGHTBRACE}, {Key_MOD_LSHIFT, Key_6},
    {Key_MOD_LSHIFT, Key_MINUS}, {0, Key_GRAVE}, {0, Key_A}, {0, Key_B},
    {0, Key_C}, {0, Key_D}, {0, Key_E}, {0, Key_F}, {0, Key_G}, {0, Key_H},
    {0, Key_I}, {0, Key_J}, {0, Key_K}, {0, Key_L}, {0, Key_M}, {0, Key_N},
    {0, Key_O}, {0, Key_P}, {0, Key_Q}, {0, Key_R}, {0, Key_S}, {0, Key_T},
    {0, Key_U}, {0, Key_V}, {0, Key_W}, {0, Key_X}, {0, Key_Y}, {0, Key_Z},
    {Key_MOD_LSHIFT, Key_LEFTBRACE}, {Key_MOD_LSHIFT, Key_BACKSLASH},
    {Key_MOD_LSHIFT, Key_RIGHTBRACE}, {Key_MOD_LSHIFT, Key_GRAVE},
};


void hid_sendch(uint8_t c)
{
    uint8_t buffer[8] = {0};

    if (c > 127) return;
    if (c < 32) return;

    c -= 32; // offset ignore the first 32 symbols in ascii table

    buffer[0] = ascii_to_hid_key_map[c][0];
    buffer[2] = ascii_to_hid_key_map[c][1];

    FILE* f = fopen("/dev/hidg0", "wb");
    fwrite(buffer, sizeof (char), 8, f);
    fclose(f);

    usleep(10000);

    buffer[0] = 0;
    buffer[2]= 0;
    f = fopen("/dev/hidg0", "wb");
    fwrite(buffer, sizeof (char), 8, f);
    fclose(f);

    usleep(50000);
}
void hid_sendstr(char* str)
{
    unsigned char l = strlen(str);
    for (unsigned char i = 0; i < l; i++) {
        hid_sendch(str[i]);
    }
}
hid_sendstr("Hello World!");
