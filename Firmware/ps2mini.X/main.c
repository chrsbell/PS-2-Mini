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

#include "device.h"
#include "configuration.h"
#include "ps2_protocol.h"

/******************************************************************************/
/*																			  */
/*		This is the firmware for a 4-key PS/2 keyboard                        */
/*      using a very low amount of latency, specifically                      */
/*		for use with osu!catch. Reference attached schematic.                 */
/*																			  */
/******************************************************************************/

void initialize(void);

void update_switches(void);

/* -------------------------------------------------------------------------- */

enum { //possible states of keyboard
    STATE_RX, //receiving data from host
    STATE_TX, //sending data to host
    STATE_IDLE, //idle state (ok to send)
    STATE_INHIBIT //communication inhibited by host
};

int currState = STATE_IDLE; //the current state

//microcontroller port input
int16 portInput;
int16 lastPortState = input_b();
int16 portChange;

/* -------------------------------------------------------------------------- */

/*
 * main initialization routine
 */

void initialize(void) {
    set_pullup(TRUE, KEYBOARD_CLOCK_PIN); //default state of clock/data is high
    set_pullup(TRUE, KEYBOARD_DATA_PIN);
    
    set_pulldown(TRUE, KEY_SWITCH1); //switches default low
    set_pulldown(TRUE, KEY_SWITCH2);
    set_pulldown(TRUE, KEY_SWITCH3);
    set_pulldown(TRUE, KEY_SWITCH4);
}

/* -------------------------------------------------------------------------- */

/*
 * main routine
 */

void main(void) {

    initialize();

    BYTE command; //current host command
    SCANCODE* packet = 0; //current packet being transmitted
    BOOLEAN packetSent = TRUE;

    while (1) { //receive commands if there are any, send scancodes if there are any

        if (currState != STATE_TX) {
            if (device_communication_inhibited()) { //check if communication is inhibited
                if (is_host_requesting_to_send()) {
                    currState = STATE_RX;
                } else {
                    currState = STATE_INHIBIT;
                }
            } else {
                currState = STATE_IDLE; //ok to send
            }
        }

        if (currState == STATE_RX) { //if there is a command
            command = execute_host_command(); //receive command from host
            switch (command) {
                case KEYBOARD_RESET: //reset if necessary
                    SCANCODE s;
                    s.scancode = KEYBOARD_BAT;
                    s.delay = RESET_DELAY; //delay before resetting
                    send_to_host(s);
                    break;
                case KEYBOARD_DISABLE:
                    break;
                case KEYBOARD_ENABLE:
                    break;
                case SET_LED_CMD:
                    break;
                case CLEAR_LEDS:
                    break;
                case NUM_LOCK_LED:
                    break;
                case SET_TYPEMATIC_CMD: //this kb isn't intended for typing so typematic delay is unnecessary
                    break;
                default:
                    break;
            }
            currState = STATE_IDLE;
        }
        
        update_switches();
        
    }
}

/* -------------------------------------------------------------------------- */

/******************************************************************************/
/*																			  */
/* 		PS/2 host and device keyboard low level PS/2 protocol routines	      */
/*																			  */
/*		A host is typically a personal computer with PS/2 connector, 		  */
/* 		and device is typically a PS/2 keyboard or a PS/2 three-button mouse  */
/*																			  */
/******************************************************************************/

/*
 * checks if communications are being inhibited by host
 */

BOOLEAN device_communication_inhibited(void) {
    //both clock/data lines must be high to be ok to send
    return (input(KEYBOARD_DATA_PIN) != HIGH || input(KEYBOARD_CLOCK_PIN) != HIGH);
}

/* -------------------------------------------------------------------------- */

/*
 * checks if host has sent a request-to-send
 */

BOOLEAN is_host_requesting_to_send(void) {
    //if both clock/data lines are low, host pc wants to send
    return (input(KEYBOARD_DATA_PIN) == LOW && input(KEYBOARD_CLOCK_PIN) == LOW);
}

/* -------------------------------------------------------------------------- */

/*
 * This routine receives a command code from the host
 *
 *		-- this routine acts as a device
 *
 *		precondition: host has done a request-to-send
 */

BYTE receive_from_host(void) {

    int i;
    BYTE commandcode = 0;
    
    while (input(KEYBOARD_CLOCK_PIN) != HIGH); //wait for host to release clock line

    output_drive(KEYBOARD_CLOCK_PIN); //set clock line to output mode
    
    /* read the eight data bits, one parity bit, and one stop bit */
    for (i = 0; i < 10; i++) {
        output_low(KEYBOARD_CLOCK_PIN);
        delay_us(CLOCK_PERIOD);
        if (i < 8) {
            if (input(KEYBOARD_DATA_PIN) == HIGH) { /* compiler bug: input() */
                bit_set(commandcode, i);
            } else {
                bit_clear(commandcode, i);
            }
        }
        output_high(KEYBOARD_CLOCK_PIN); /* generate clock pulse */
        delay_us(CLOCK_PERIOD);
    }

    output_drive(KEYBOARD_DATA_PIN);
    
    /* send the acknowledgment */
    output_low(KEYBOARD_DATA_PIN);
    delay_us(CLOCK_PERIOD_HALF);
    output_low(KEYBOARD_CLOCK_PIN);
    delay_us(CLOCK_PERIOD);
    output_high(KEYBOARD_CLOCK_PIN);
    delay_us(CLOCK_PERIOD_HALF);
    output_high(KEYBOARD_DATA_PIN);

    output_float(KEYBOARD_CLOCK_PIN); /* set to high impedance input */
    output_float(KEYBOARD_DATA_PIN);

    return commandcode;
}

/* -------------------------------------------------------------------------- */

/*
 * gets a command from the host and sends acknowledgment
 * precondition: host has sent a request-to-send
 */

BYTE execute_host_command() {
    BYTE commandcode;
    SCANCODE s;
    commandcode = receive_from_host(); //get a command code from host
    s.scancode = KEYBOARD_ACK;
    s.delay = ACK_DELAY;
    send_to_host(s);
    while (device_communication_inhibited()); //wait for host to release clock/data lines
    return commandcode;
}

/* -------------------------------------------------------------------------- */

/*
 * This routine sends a packet to the host 
 */

void send_to_host(SCANCODE packet) {
    
    while(device_communication_inhibited()); //wait for the ok to send
    
    output_drive(KEYBOARD_CLOCK_PIN); //set clock/data to output mode
    output_drive(KEYBOARD_DATA_PIN);
    
    if (packet.delay > CLOCK_PERIOD){
        delay_us(packet.delay); //custom delay
    } else {
        delay_us(CLOCK_PERIOD); //make sure clock line high for minimum duration before sending data
    }
    
    int i;
	int ndata = 0;
	int parity = HIGH;
	int data[11]; // one start bit, 8 data bits, one parity bit, one stop bit
	int a_bit;
    
    // set start bit
	data[ndata++] = LOW;

	// set data bits and compute parity bit
	for (i = 0; i < 8; i++) {
		a_bit = bit_test(packet.scancode, i);
		parity ^= a_bit;
		data[ndata++] = a_bit;
	}
    
	// set parity bit
	data[ndata++] = parity;

	// set stop bit
	data[ndata++] = HIGH;
    
    for (i = 0; i < ndata; i++) {
		output_bit(KEYBOARD_DATA_PIN, data[i]);
		delay_us(CLOCK_PERIOD_HALF);
		output_low(KEYBOARD_CLOCK_PIN);		// generate clock pulse
		delay_us(CLOCK_PERIOD);
		output_high(KEYBOARD_CLOCK_PIN);		// generate clock pulse
		delay_us(CLOCK_PERIOD_HALF);
	}
    
    set_pullup(TRUE, KEYBOARD_CLOCK_PIN); //re-enable pullups
    set_pullup(TRUE, KEYBOARD_DATA_PIN);
    
    output_float(KEYBOARD_DATA_PIN);
    output_float(KEYBOARD_CLOCK_PIN);
}

/* -------------------------------------------------------------------------- */

/*
 * this function checks for state changes on the input pins and pushes the corresponding key state change
 */

void update_switches(void) {
    //get key state changes
    portInput = input_b();
    portChange = lastPortState ^ portInput;
    lastPortState = portInput;
    //check key on pin 4 (index finger)
    if (bit_test(portChange, 4)) {
        SCANCODE s;
        s.delay = 0;
        if (bit_test(lastPortState, 4)) { //key press detected
            //send key press code
            s.scancode = KEY_A;
            send_to_host(s);
        } else {
            //send key release code
            s.scancode = BREAK_CODE;
            send_to_host(s);
            s.scancode = KEY_A;
            send_to_host(s);
        }
        //firmware side debounce
        delay_us(DEBOUNCE_DELAY);
    }
    //check key and repeat for pin 2 (middle finger)
    if (bit_test(portChange, 2)) {
        SCANCODE s;
        s.delay = 0;
        if (bit_test(lastPortState, 2)) {
            s.scancode = KEY_B;
            send_to_host(s);
        } else {
            s.scancode = BREAK_CODE;
            send_to_host(s);
            s.scancode = KEY_B;
            push(s);
        }
        delay_us(DEBOUNCE_DELAY);
    }
    //check key and repeat for pin 3 (ring finger)
    if (bit_test(portChange, 3)) {
        SCANCODE s;
        s.delay = 0;
        if (bit_test(lastPortState, 3)) {
            s.scancode = KEY_C;
            send_to_host(s);
        } else {
            s.scancode = BREAK_CODE;
            send_to_host(s);
            s.scancode = KEY_C;
            send_to_host(s);
        }
        delay_us(DEBOUNCE_DELAY);
    }
    //check key on pin 5 (dash key)
    if (bit_test(portChange, 5)) {
        SCANCODE s;
        s.delay = 0;
        if (bit_test(lastPortState, 5)) {
            s.scancode = KEY_D;
            send_to_host(s);
        } else {
            s.scancode = BREAK_CODE;
            send_to_host(s);
            s.scancode = KEY_D;
            send_to_host(s);
        }
        delay_us(DEBOUNCE_DELAY);
    }
}

/* -------------------------------------------------------------------------- */