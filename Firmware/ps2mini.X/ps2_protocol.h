/*
 *                      PS/2 Low Latency osu! Keyboard
 *  
 *  Copyright (C) 2019  Chris Bell, https://creationsofchris.wordpress.com/
 *
 *  Code modified from David Bern's PS/2 Keyer
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
 *  USA, or see <http://www.gnu.org/licenses/>.
 */

/* -------------------------------------------------------------------------- */

enum {
	LOW,
	HIGH,
    FLOATING,
};

enum {		/* keyboard host command codes */
	KEYBOARD_RESET				= 0xFF,
	KEYBOARD_RESEND				= 0xFE,	
    KEYBOARD_DISABLE            = 0xF5,
    KEYBOARD_ENABLE             = 0xF4,
    
	READ_ID_CMD					= 0xF2,

	SET_TYPEMATIC_CMD			= 0xF3,
		TYPEMATIC_VALUE1		= 0x00,
		TYPEMATIC_VALUE2		= 0x20,

	SET_DEFAULT_CMD				= 0xF6,

	SET_LED_CMD 				= 0xED,
		CAPS_LOCK_LED	 		= 0x04,
		NUM_LOCK_LED			= 0x02,
		SCROLL_LOCK_LED			= 0x01,
		CLEAR_LEDS				= 0x00,
};

enum {		/* keyboard device response codes */
	KEYBOARD_BAT			= 0xAA,
	KEYBOARD_ACK			= 0xFA,
	KEYBOARD_ID_1			= 0xAB,
	KEYBOARD_ID_2		    = 0x83,
};

enum {		/* keyboard device scan codes */
    	BREAK_CODE			= 0xF0,
        KEY_A               = 0x1C,
        KEY_B               = 0x32,
        KEY_C               = 0x21,
        KEY_D               = 0x23,
};

typedef struct SCANCODE{
    BYTE scancode;
    int delay;
} SCANCODE;

BOOLEAN is_host_requesting_to_send(void);

BOOLEAN device_communication_inhibited(void);

BYTE execute_host_command(void);

BYTE receive_from_host(void);

void send_to_host(SCANCODE packet);

/* -------------------------------------------------------------------------- */
