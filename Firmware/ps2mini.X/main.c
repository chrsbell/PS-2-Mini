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

void initialize_interrupts(void);

void push(SCANCODE data);

SCANCODE* pop();

void update_switches(void);

/* -------------------------------------------------------------------------- */

enum { //possible states of keyboard
    STATE_RX, //receiving data from host
    STATE_TX, //sending data to host
    STATE_IDLE, //idle state (ok to send)
    STATE_INHIBIT //communication inhibited by host
};

int currState = STATE_IDLE; //the current state

SCANCODE queue[QUEUE_SIZE]; //queue of scancodes to send
int front = 0, back = 0;

int bit_index = 0;
int parity = HIGH;
BYTE tx_data; //data being transmitted
BOOLEAN doneTx = TRUE; //whether we are done transmitting

BOOLEAN bootFinished = FALSE;

//int edge[4] = {L_TO_H, L_TO_H, L_TO_H, L_TO_H}; //which edge to detect in external interrupt

//possible states for clock signal while txing
enum {
    CLK_STATE_OUTPUT_BIT,
    CLK_STATE_LOW,
    CLK_STATE_HIGH
};

//possible states for data signal while txing
enum {
    SEND_START_BIT,
    SEND_DATA_BIT,
    SEND_PARITY_BIT,
    SEND_STOP_BIT,
    SEND_FINISH_HIGH,
    SEND_FINISH_COMPLETE
};

int currClkState;
int currDataState;

int16 portInput;
int16 lastPortState = input_b();
int16 portChange;

BOOLEAN movementADown = FALSE;
BOOLEAN movementBDown = FALSE;
BOOLEAN movementDashDown = FALSE;

/* -------------------------------------------------------------------------- */

/*
 * simple circular queue push
 */

void push(SCANCODE data) {
    back = (back + 1) % QUEUE_SIZE;
    queue[back] = data;
}

/* -------------------------------------------------------------------------- */

/*
 * simple circular queue pop
 */

SCANCODE* pop() {
    if (front != back) {
        front = (front + 1) % QUEUE_SIZE;
        return &queue[front];
    }
    return 0; //there is no scancode
}

/* ------------------------------------------------------------------------ */

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

    initialize_interrupts();
}

/* -------------------------------------------------------------------------- */

/*
 * main routine
 */

void main(void) {
    
    initialize();
    
    BYTE command; //current host command
    SCANCODE* packet; //current packet being transmitted
    
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
                    push(s);
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

        if (currState == STATE_IDLE) {//ok to transmit in this state
            packet = pop();
            if (packet) {
                send_to_host(*packet);
            }
        }
        if (currState == STATE_TX) {
            if (doneTx) {
                currState = STATE_IDLE; //immediately go back to sending scancodes
            }
        }
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
    push(s);
    while (device_communication_inhibited()); //wait for host to release clock/data lines
    return commandcode;
}

/* -------------------------------------------------------------------------- */

/*
 * This routine sends a packet to the host 
 */

void send_to_host(SCANCODE packet) {
 
    while(device_communication_inhibited()); //wait for the ok to send
    if (packet.delay > CLOCK_PERIOD){
        delay_us(packet.delay); //custom delay
    } else {
        delay_us(CLOCK_PERIOD); //make sure clock line high for minimum duration before sending data
    }
    tx_data = packet.scancode;
    doneTx = FALSE;
    bit_index = 0;
    parity = HIGH;
    set_timer1(65535); //start sending immediately
    output_drive(KEYBOARD_CLOCK_PIN); //set clock/data to output mode
    output_drive(KEYBOARD_DATA_PIN);
    currDataState = SEND_START_BIT; //reset data line state machine
    currClkState = CLK_STATE_OUTPUT_BIT; //reset clock line state machine
    currState = STATE_TX;
    enable_interrupts(INT_TIMER1); //turn on tx handler
}

/******************************************************************************/
/*																			  */
/*				interrupt management and handling routines					  */
/*																			  */
/******************************************************************************/

/* -------------------------------------------------------------------------- */

/*
 * this routine initializes the interrupts
 */

void initialize_interrupts(void) {
    //key press/release interrupts for each switch
    /*ext_int_edge(0, L_TO_H);
    ext_int_edge(1, L_TO_H);
    ext_int_edge(2, L_TO_H);
    ext_int_edge(3, L_TO_H);

    enable_interrupts(INT_EXT0);
    enable_interrupts(INT_EXT1);
    enable_interrupts(INT_EXT2);
    enable_interrupts(INT_EXT3);*/
 
    setup_timer1(TMR_INTERNAL | TMR_DIV_BY_1); //prescalar 1
    //setup_timer2(TMR_INTERNAL | TMR_DIV_BY_1); //prescalar 1
    /*setup_ccp1(CCP_TIMER | CCP_DIV_BY_1); //use ccp1 as timer2
    setup_ccp1(CCP_COMPARE_SET_ON_MATCH | CCP_TIMER_32_BIT | CCP_DIV_BY_64); //use ccp1 as timer2
    
    set_ccp1_compare_time(TIMER1_PERIOD);
    */
    //enable_interrupts(INT_CCP1);
    //enable_interrupts(INT_CCP2);
    enable_interrupts(GLOBAL);
}

/* -------------------------------------------------------------------------- */

/*
 * this is the 16 bit timer1 overflow interrupt service used for txing data
 * note - the signals generated here may appear distorted if using debug mode
 */

#INT_TIMER1
void timer1_isr(void) {

    switch (currClkState) {

        case CLK_STATE_OUTPUT_BIT:
            //which data bit are we sending?
            switch (currDataState) {
                case SEND_START_BIT:
                    output_low(KEYBOARD_DATA_PIN);
                    currDataState++;
                    break;
                case SEND_DATA_BIT:
                    int currBit = bit_test(tx_data, bit_index);
                    output_bit(KEYBOARD_DATA_PIN, currBit);
                    //calculate parity
                    parity ^= currBit;
                    bit_index++;
                    if (bit_index == 8) {
                        currDataState++;
                    }
                    break;
                case SEND_PARITY_BIT:
                    output_bit(KEYBOARD_DATA_PIN, parity);
                    currDataState++;
                    break;
                case SEND_STOP_BIT:
                    output_high(KEYBOARD_DATA_PIN);
                    currDataState++;
                    break;
            }
            //send next data bit in middle of next clock pulse
            set_timer1(TIMER_CLOCKP_H_START);
            break;

        case CLK_STATE_LOW:

            output_low(KEYBOARD_CLOCK_PIN);
            set_timer1(TIMER_CLOCKP_START);
            break;

        case CLK_STATE_HIGH:

            if (currDataState == SEND_FINISH_HIGH) {
                output_high(KEYBOARD_CLOCK_PIN); //set clock high one last time to show we are done
                set_timer1(TIMER_CLOCKP_START);
                currDataState++;
                return;
            } else if (currDataState == SEND_FINISH_COMPLETE) { //we are done transmitting the packet
                output_float(KEYBOARD_DATA_PIN); //set data to high impedance
                output_float(KEYBOARD_CLOCK_PIN); //set clock to high impedance
                disable_interrupts(INT_TIMER1); //turn off tx handler
                doneTx = TRUE;
                return;
            } else {
                output_high(KEYBOARD_CLOCK_PIN);
                set_timer1(TIMER_CLOCKP_H_START);
            }
            currClkState = -1; //reset clock line state machine
            break;

    }
    currClkState++;
}

/* -------------------------------------------------------------------------- */

/*
 * this is the 16 bit timer2 overflow interrupt service used for checking the keyboard state
 */
/*
#INT_CCP1
void timer2_isr(void) {
    SCANCODE s;
    s.scancode = BREAK_CODE;
    push(s); //key release detected
    s.scancode = KEY_B;
    push(s);
    disable_interrupts(INT_CCP1);
}*/


/* -------------------------------------------------------------------------- */

/*
 * this is the 32 bit timer3 overflow interrupt service used for disabling keyboard state checks
 */

/*#INT_CCP1
void timer2_isr(void) {
    bootFinished = TRUE;
    currState = STATE_IDLE; //force idle state
    disable_interrupts(INT_CCP1); //turn off current isr
}*/

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
            //other movement key is down, so release it (joystick mode)
            if (movementBDown) {
                //send key release code
                s.scancode = BREAK_CODE;
                push(s);
                s.scancode = KEY_B;
                push(s);
            }
            //send key press code
            movementADown = TRUE;
            s.scancode = KEY_A;
            push(s);
        } else {
            //send key release code
            s.scancode = BREAK_CODE;
            push(s);
            s.scancode = KEY_A;
            push(s);
            movementADown = FALSE;
            //the other movement key is still down, so set it as the active one again
            if (movementBDown) {
                s.scancode = KEY_B;
                push(s);
            }
        }
        //firmware side debounce
        delay_us(DEBOUNCE_DELAY);
    }
    //check key and repeat for pin 2 (middle finger)
    if (bit_test(portChange, 2)) {
        SCANCODE s;
        s.delay = 0;
        if (bit_test(lastPortState, 2)) {
            if (movementADown) {
                s.scancode = BREAK_CODE;
                push(s);
                s.scancode = KEY_A;
                push(s);
            }
            movementBDown = TRUE;
            s.scancode = KEY_B;
            push(s);
        } else {
            s.scancode = BREAK_CODE;
            push(s);
            s.scancode = KEY_B;
            push(s);
            movementBDown = FALSE;
            if (movementADown) {
                s.scancode = KEY_A;
                push(s);
            }
        }
        delay_us(DEBOUNCE_DELAY);
    }
    //check key and repeat for pin 3 (ring finger)
    if (bit_test(portChange, 3)) {
        SCANCODE s;
        s.delay = 0;
        if (bit_test(lastPortState, 3)) {
            if (movementADown) {
                s.scancode = BREAK_CODE;
                push(s);
                s.scancode = KEY_A;
                push(s);
            }
            movementBDown = TRUE;
            s.scancode = KEY_B;
            push(s);
        } else {
            s.scancode = BREAK_CODE;
            push(s);
            s.scancode = KEY_B;
            push(s);
            movementBDown = FALSE;
            if (movementADown) {
                s.scancode = KEY_A;
                push(s);
            }
        }
        delay_us(DEBOUNCE_DELAY);
    }
    //check key on pin 5 (dash key)
    if (bit_test(portChange, 5)) {
        SCANCODE s;
        s.delay = 0;
        if (bit_test(lastPortState, 5)) {
            movementDashDown = TRUE;
            s.scancode = KEY_D;
            push(s);
        } else {
            s.scancode = BREAK_CODE;
            push(s);
            s.scancode = KEY_D;
            push(s);
            movementDashDown = FALSE;
        }
        delay_us(DEBOUNCE_DELAY);
    }
}

/* -------------------------------------------------------------------------- */