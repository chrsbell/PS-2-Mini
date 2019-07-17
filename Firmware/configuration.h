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

//#define DEBUG
//#define ERROR_LED	PIN_B12

/* -------------------------------------------------------------------------- */

		/* PS/2 port definitions */

#define keyboard_host_clock		PIN_B11	
#define keyboard_host_data		PIN_B12
#define keyboard_host_led		PIN_A0

#define CLOCK_PERIOD 20 //in microseconds, PS/2 clock period is ~30-50 microseconds
#define CLOCK_PERIOD_HALF 10

#define TIMER_CLOCKP_START 64535//65335//65035//65535-(CLOCK_PERIOD/(2/OSC_FREQ)) //triggers every clock period (20 us)
#define TIMER_CLOCKP_H_START 65035//65435//65285//65535-(CLOCK_PERIOD_HALF/(2/OSC_FREQ)) //triggers every half a clock period (10 us)

#define TIMER1_START 63035 //65535-(.00005/(2/OSC_FREQ))
#define DEBOUNCE_DELAY 50 //in microseconds, external circuit can debounce in ~1 microsecond
#define RESET_DELAY 10000 //in microseconds
#define ACK_DELAY 1000 //in microseconds
//#define SCANCODE_DELAY 150 //in microseconds, needed for PC to recognize scancodes very close to each other
                           //common when playing rhythm games
//#define TIMER3_START 65534//61935 //65535-(SCANCODE_DELAY/(2/OSC_FREQ))
#define TIMER2_PERIOD 48828 //OverflowTime / (2 * (1/OscFrequency) * Prescale) = Period
#define TIMER2_INCR 250 //overflow time, in milliseconds
#define BOOT_TIME 30000 //in milliseconds

#define QUEUE_SIZE 50 //max number of scancodes to queue

#define KEY_SWITCH1 PIN_B2 //EXT interrupt 0 on PIN_B7 is unchangeable, don't change this unless
                           //you're using a different chip
#define KEY_SWITCH2 PIN_B3 //EXT interrupt 1
#define KEY_SWITCH3 PIN_B4 //EXT interrupt 2
#define KEY_SWITCH4 PIN_B5 //EXT interrupt 3