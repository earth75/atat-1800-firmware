/*
 * keycodes.h
 *
 *  Created on: Jan 14, 2024
 *#define Key_  Author: alexi
 */

#ifndef INC_KEYCODES_H_
#define Key_INC_KEYCODES_H_
/*#define Key_
This list is modeled after the names for USB keycodes defined in
https://usb.org/sites/default/files/hut1_21_0.pdf#page=83.
This list does not include every single code, but does include all the keys on
a regular PC or Mac keyboard.

Remember that keycodes are the names for key *positions* on a US keyboard, and may
not correspond to the character that you mean to send if you want to emulate non-US keyboard.
For instance, on a French keyboard (AZERTY instead of QWERTY),
the keycode for 'q' is used to indicate an 'a'. Likewise, 'y' represents 'z' on
a German keyboard. This is historical: the idea was that the keycaps could be changed
without changing the keycodes sent, so that different firmware was not needed for
different variations of a keyboard.

*/
#define Key_MACRO	0x00 // Dummy code for MACRO
        
#define Key_A		0x04 // ``a`` and ``A``
#define Key_B		0x05 // ``b`` and ``B``
#define Key_C		0x06 // ``c`` and ``C``
#define Key_D		0x07 // ``d`` and ``D``
#define Key_E		0x08 // ``e`` and ``E``
#define Key_F		0x09 // ``f`` and ``F``
#define Key_G		0x0A // ``g`` and ``G``
#define Key_H		0x0B // ``h`` and ``H``
#define Key_I		0x0C // ``i`` and ``I``
#define Key_J		0x0D // ``j`` and ``J``
#define Key_K		0x0E // ``k`` and ``K``
#define Key_L		0x0F // ``l`` and ``L``
#define Key_M		0x10 // ``m`` and ``M``
#define Key_N		0x11 // ``n`` and ``N``
#define Key_O		0x12 // ``o`` and ``O``
#define Key_P		0x13 // ``p`` and ``P``
#define Key_Q		0x14 // ``q`` and ``Q``
#define Key_R		0x15 // ``r`` and ``R``
#define Key_S		0x16 // ``s`` and ``S``
#define Key_T		0x17 // ``t`` and ``T``
#define Key_U		0x18 // ``u`` and ``U``
#define Key_V		0x19 // ``v`` and ``V``
#define Key_W		0x1A // ``w`` and ``W``
#define Key_X		0x1B // ``x`` and ``X``
#define Key_Y		0x1C // ``y`` and ``Y``
#define Key_Z		0x1D // ``z`` and ``Z``

#define Key_ONE			0x1E // ``1`` and ``!``
#define Key_TWO			0x1F // ``2`` and ``@``
#define Key_THREE		0x20 // ``3`` and ``#``
#define Key_FOUR		0x21 // ``4`` and ``$``
#define Key_FIVE		0x22 // ``5`` and ``%``
#define Key_SIX			0x23 // ``6`` and ``^``
#define Key_SEVEN		0x24 // ``7`` and ``&``
#define Key_EIGHT		0x25 // ``8`` and ``*``
#define Key_NINE		0x26 // ``9`` and ``(``
#define Key_ZERO		0x27 // ``0`` and ``)``
#define Key_ENTER		0x28 // Enter (Return)
#define Key_ESCAPE		0x29 // Escape
#define Key_BACKSPACE	0x2A // Delete backward (Backspace)
#define Key_TAB			0x2B // Tab and Backtab
#define Key_SPACEBAR	0x2C // Spacebar
#define Key_MINUS		0x2D // ``-` and ``_``
#define Key_EQUALS		0x2E // ``=` and ``+``
#define Key_L_BRACKET	0x2F // ``[`` and ``{``
#define Key_R_BRACKET	0x30 // ``]`` and ``}``
#define Key_BACKSLASH		0x31 // ``\`` and ``|``
#define Key_POUND			0x32 // ``#`` and ``~`` (Non-US keyboard)
#define Key_SEMICOLON		0x33 // ``;`` and ``:``
#define Key_QUOTE			0x34 // ``'`` and ``"``
#define Key_GRAVE_ACCENT	0x35 // :literal:`\`` and ``~``
#define Key_COMMA			0x36 // ``,`` and ``<``
#define Key_PERIOD			0x37 // ``.`` and ``>``
#define Key_FORWARD_SLASH	0x38 // ``/`` and ``?``

#define Key_CAPS_LOCK		0x39 // Caps Lock

#define Key_F1		0x3A // Function key F1
#define Key_F2		0x3B // Function key F2
#define Key_F3		0x3C // Function key F3
#define Key_F4		0x3D // Function key F4
#define Key_F5		0x3E // Function key F5
#define Key_F6		0x3F // Function key F6
#define Key_F7		0x40 // Function key F7
#define Key_F8		0x41 // Function key F8
#define Key_F9		0x42 // Function key F9
#define Key_F10		0x43 // Function key F10
#define Key_F11		0x44 // Function key F11
#define Key_F12		0x45 // Function key F12

#define Key_PRINT_SCREEN	0x46 // Print Screen (SysRq)
#define Key_SCROLL_LOCK		0x47 // Scroll Lock
#define Key_PAUSE			0x48 // Pause (Break)

#define Key_INSERT		0x49 // Insert
#define Key_HOME		0x4A // Home (often moves to beginning of line)
#define Key_PAGE_UP		0x4B // Go back one page
#define Key_DELETE		0x4C // Delete forward
#define Key_END			0x4D // End (often moves to end of line)
#define Key_PAGE_DOWN	0x4E // Go forward one page

#define Key_R_ARROW		0x4F // Move the cursor right
#define Key_L_ARROW		0x50 // Move the cursor left
#define Key_DOWN_ARROW		0x51 // Move the cursor down
#define Key_UP_ARROW		0x52 // Move the cursor up

#define Key_KEYPAD_NUMLOCK			0x53 // Num Lock (Clear on Mac)
#define Key_KEYPAD_FORWARD_SLASH	0x54 // Keypad ``/``
#define Key_KEYPAD_ASTERISK			0x55 // Keypad ``*``
#define Key_KEYPAD_MINUS			0x56 // Keyapd ``-``
#define Key_KEYPAD_PLUS				0x57 // Keypad ``+``
#define Key_KEYPAD_ENTER			0x58 // Keypad Enter
#define Key_KEYPAD_ONE				0x59 // Keypad ``1`` and End
#define Key_KEYPAD_TWO				0x5A // Keypad ``2`` and Down Arrow
#define Key_KEYPAD_THREE			0x5B // Keypad ``3`` and PgDn
#define Key_KEYPAD_FOUR				0x5C // Keypad ``4`` and Left Arrow
#define Key_KEYPAD_FIVE				0x5D // Keypad ``5``
#define Key_KEYPAD_SIX				0x5E // Keypad ``6`` and Right Arrow
#define Key_KEYPAD_SEVEN			0x5F // Keypad ``7`` and Home
#define Key_KEYPAD_EIGHT			0x60 // Keypad ``8`` and Up Arrow
#define Key_KEYPAD_NINE				0x61 // Keypad ``9`` and PgUp
#define Key_KEYPAD_ZERO				0x62 // Keypad ``0`` and Ins
#define Key_KEYPAD_PERIOD			0x63 // Keypad ``.`` and Del
#define Key_KEYPAD_BACKSLASH		0x64 // Keypad ``\\`` and ``|`` (Non-US)

#define Key_WIN				0x65 // Application: also known as the Menu key (Windows Key)
#define Key_POWER			0x66 // Power (Mac)
#define Key_KEYPAD_EQUALS	0x67 // Keypad ``=`` (Mac)

#define Key_F13		0x68 // Function key F13 (Mac)
#define Key_F14		0x69 // Function key F14 (Mac)
#define Key_F15		0x6A // Function key F15 (Mac)
#define Key_F16		0x6B // Function key F16 (Mac)
#define Key_F17		0x6C // Function key F17 (Mac)
#define Key_F18		0x6D // Function key F18 (Mac)
#define Key_F19		0x6E // Function key F19 (Mac)

#define Key_F20		0x6F // Function key F20
#define Key_F21		0x70 // Function key F21
#define Key_F22		0x71 // Function key F22
#define Key_F23		0x72 // Function key F23
#define Key_F24		0x73 // Function key F24

#define Key_L_CTL	0xE0 // Control modifier left of the spacebar
#define Key_L_SHIFT		0xE1 // Shift modifier left of the spacebar
#define Key_L_ALT		0xE2 // Alt modifier left of the spacebar
#define Key_L_WIN		0xE3 // GUI modifier left of the spacebar
#define Key_R_CTL	0xE4 // Control modifier right of the spacebar
#define Key_R_SHIFT		0xE5 // Shift modifier right of the spacebar
#define Key_R_ALT		0xE6 // Alt modifier right of the spacebar
#define Key_R_WIN		0xE7 // GUI modifier right of the spacebar


#define LED_NUM_LOCK    0x01 // LED Usage ID for Num Lock
#define LED_CAPS_LOCK   0x02 // LED Usage ID for Caps Lock
#define LED_SCROLL_LOCK 0x04 // LED Usage ID for Scroll Lock
#define LED_COMPOSE     0x08 // LED Usage ID for Compose


//optimized modifier bit calculator
inline int modifier_bit(int keycode) {return keycode&0x80 ? 1<<(keycode&0x0F) : 0;}


#endif /* INC_KEYCODES_H_ */
