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

static void initialize(void);

static void initialize_interrupts(void);

static void update_switches(void);

static void push(SCANCODE data);

static SCANCODE* pop();

/* -------------------------------------------------------------------------- */

enum { //possible states of keyboard
    STATE_RX, //receiving data from host
    STATE_TX, //sending data to host
    STATE_IDLE, //idle state (ok to send)
    STATE_INHIBIT //communication inhibited by host
};

static int currState = STATE_IDLE; //the current state

static SCANCODE queue[QUEUE_SIZE]; //queue of scancodes to send
static int front = 0, back = 0;

static BOOLEAN bootFinished = FALSE;
//static BOOLEAN SCANCODE_DELAY_PASSED = TRUE; //see configuration.h
static int bit_index = 0;
static int parity = HIGH;
static BYTE tx_data; //data being transmitted
static BOOLEAN doneTx = TRUE; //whether we are done transmitting

static int edge[4] = {L_TO_H, L_TO_H, L_TO_H, L_TO_H};

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
    //SEND_FINISH_WAIT,
    SEND_FINISH
};

static int currClkState;
static int currDataState;

int16 portInput;
int16 lastPortState;
int16 portChange;

/* -------------------------------------------------------------------------- */

/*
 * simple circular queue push
 */

static void push(SCANCODE data) {
    back = (back + 1) % QUEUE_SIZE;
    queue[back] = data;
}

/* -------------------------------------------------------------------------- */

/*
 * simple circular queue pop
 */

static SCANCODE* pop() {
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

static void initialize(void) {
    set_pullup(TRUE, keyboard_host_clock); //default state of clock/data is high
    set_pullup(TRUE, keyboard_host_data);
    
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
#ifdef DEBUG
    printf("Starting program.\r\n");
#endif
    initialize();
    //delay_ms(1); //allow timers to interrupt at least once to check initial kb state
    
    /*while(1){
        SCANCODE s;
        s.delay = 100;
        s.scancode = KEYBOARD_BAT;
        send_to_host(s);
    }*/
    
    BYTE command; //current host command
    SCANCODE* packet; //current packet being transmitted
    int counter = 0;
    
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
            //disable_interrupts(INT_TIMER1); //make sure timer doesn't interfere with data transfer
            command = execute_host_command(); //receive command from host
            //enable_interrupts(INT_TIMER1);
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
                case SET_LED_CMD: //set led
                    output_high(keyboard_host_led);
                    break;
                case CLEAR_LEDS: //clear led
                    output_low(keyboard_host_led);
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
        if (currState == STATE_IDLE) {//ok to transmit in this state
            packet = pop();
            if (packet) {
                if (!bootFinished) { //still in boot up stage
                    //disable_interrupts(INT_TIMER1);
                    send_to_host(*packet);
                }else{
                    send_to_host(*packet);
                }
            }
        }
        if (currState == STATE_TX) {
            if (doneTx) {
                //set_pullup(TRUE, keyboard_host_clock); //reenable pullups
                //set_pullup(TRUE, keyboard_host_data);
                /*if(!bootFinished){
                    enable_interrupts(INT_TIMER1);
                }*/
                currState = STATE_IDLE; //immediately go back to sending scancodes
            }
        }
        //update_switches();
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
    //both must be high to be ok to send
    return (input(keyboard_host_data) != HIGH || input(keyboard_host_clock) != HIGH);
}

/* -------------------------------------------------------------------------- */

/*
 * checks if host has sent a request-to-send
 */

BOOLEAN is_host_requesting_to_send(void) {
    //if both are low, host pc wants to send
    return (input(keyboard_host_data) == LOW && input(keyboard_host_clock) == LOW);
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
    int rx_data[10]; /* 8 data bits, one parity bit, one stop bit */
    BYTE commandcode = 0;
    
    while (input(keyboard_host_clock) != HIGH); //wait for host to release clock line
    //delay_us(SCANCODE_DELAY);
    //output_drive(keyboard_host_data);
    output_drive(keyboard_host_clock); //set clock line to output mode
    /* read the eight data bits, one parity bit, and one stop bit */
    for (i = 0; i < 10; i++) {
        output_low(keyboard_host_clock);
        delay_us(CLOCK_PERIOD);
        if (i < 8) {
            if (input(keyboard_host_data) == HIGH) { /* compiler bug: input() */
                //rx_data[i] = HIGH; /* does not like to assign to an array element */
                bit_set(commandcode, i);
            } else {
                //rx_data[i] = LOW;
                bit_clear(commandcode, i);
            }
        }
        output_high(keyboard_host_clock); /* generate clock pulse */
        delay_us(CLOCK_PERIOD);
    }

    output_drive(keyboard_host_data);
    
    /* send the acknowledgment */
    output_low(keyboard_host_data);
    delay_us(CLOCK_PERIOD_HALF);
    output_low(keyboard_host_clock);
    delay_us(CLOCK_PERIOD);
    output_high(keyboard_host_clock);
    delay_us(CLOCK_PERIOD_HALF);
    output_high(keyboard_host_data);

    output_float(keyboard_host_clock); /* set to high impedance input */
    output_float(keyboard_host_data);

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
    if (packet.delay > 0){
        delay_us(packet.delay);
    }
    //delay_us(SCANCODE_DELAY);
    //set_pullup(FALSE, keyboard_host_clock); //make sure signal isn't distorted
    //set_pullup(FALSE, keyboard_host_data);
    tx_data = packet.scancode;
    doneTx = FALSE;
    bit_index = 0;
    parity = HIGH;
    set_timer1(65535); //start sending immediately
    output_drive(keyboard_host_clock); //set clock/data to output mode
    output_drive(keyboard_host_data);
    //output_high(keyboard_host_clock);
    //output_high(keyboard_host_data);
    currDataState = SEND_START_BIT; //reset data line state machine
    currClkState = CLK_STATE_OUTPUT_BIT; //reset clock line state machine
    currState = STATE_TX;
    enable_interrupts(INT_TIMER1); //turn on tx handler
    /*while(device_communication_inhibited()); //wait for the ok to send
    while(!SCANCODE_DELAY_PASSED); //make sure enough time between scancodes passes
    
    if (packet.delay){
        delay_us(packet.delay);
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
		output_bit(keyboard_host_data, data[i]);
		delay_us(CLOCK_PERIOD_HALF);
		output_low(keyboard_host_clock);		// generate clock pulse
		delay_us(CLOCK_PERIOD);
		output_high(keyboard_host_clock);		// generate clock pulse
		delay_us(CLOCK_PERIOD_HALF);
	}
    
    set_pullup(TRUE, keyboard_host_clock); //re-enable pullups
    set_pullup(TRUE, keyboard_host_data);
    
    output_float(keyboard_host_data);
    output_float(keyboard_host_clock);
    
    SCANCODE_DELAY_PASSED = FALSE;
    set_timer3(TIMER3_START);
    enable_interrupts(INT_TIMER3);*/
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

//#BYTE IOCPB = getenv("SFR:IOCPB")
//#BYTE IOCNB = getenv("SFR:IOCNB")
static void initialize_interrupts(void) {
#ifdef DEBUG
    printf("initialize_interrupts():\r\n");
#endif

    //key press/release interrupts for each switch
    ext_int_edge(0, L_TO_H);
    ext_int_edge(1, L_TO_H);
    ext_int_edge(2, L_TO_H);
    ext_int_edge(3, L_TO_H);

    enable_interrupts(INT_EXT0);
    enable_interrupts(INT_EXT1);
    enable_interrupts(INT_EXT2);
    enable_interrupts(INT_EXT3);
 
    setup_timer1(TMR_INTERNAL | TMR_DIV_BY_1); //prescalar 1
    //setup_timer2(TMR_INTERNAL | TMR_DIV_BY_256, TIMER2_PERIOD); //prescalar 256, triggers every 2 seconds
    //setup_timer3(TMR_INTERNAL | TMR_DIV_BY_1); //prescalar 1
  
    //set_timer1(TIMER1_START); //check for state changes every 0.05ms during boot up
    
    //enable_interrupts(INT_TIMER1);
    //enable_interrupts(INT_TIMER2);

    enable_interrupts(GLOBAL);

#ifdef DEBUG
    printf("initialize_interrupts() done\r\n");
#endif
}

/* -------------------------------------------------------------------------- */

/*
 * this is the 16 bit timer1 overflow interrupt service used to update the state of
 * the PS/2 keyboard. This is only run when the PC boots up, otherwise it is assumed the kb 
 * is in an idle state.
 */
/*
#INT_TIMER1
void timer1_isr(void) {
    if (device_communication_inhibited()) { //check if communication is inhibited
        if (is_host_requesting_to_send()) {
            currState = STATE_RX;
        } else {
            currState = STATE_INHIBIT;
        }
    } else {
        currState = STATE_IDLE; //ok to send
    }
    set_timer1(TIMER1_START); //reset timer
}*/

//static int milliseconds_passed = 0;

/*#INT_TIMER2
void timer2_isr(void) {
    milliseconds_passed += TIMER2_INCR;
    if (milliseconds_passed >= BOOT_TIME){
        disable_interrupts(INT_TIMER1); //turn off timer1 now
        bootFinished = TRUE;
        currState = STATE_IDLE; //force idle state
        disable_interrupts(INT_TIMER2); //and turn off timer2
    }
}*/

//static int timer3_cycles = 0;

/*#INT_TIMER3
void timer3_isr(void) {
    timer3_cycles += 1;
    if (timer3_cycles % 2 == 0) { //ignore initial interrupt trigger
        SCANCODE_DELAY_PASSED = TRUE;
        disable_interrupts(INT_TIMER3);
    }
    set_timer3(TIMER3_START);
}*/

/*
 * this is the 16 bit timer3 overflow interrupt service used for txing data
 * note - the signals generated here may appear distorted if using debug mode
 */

#INT_TIMER1
void timer1_isr(void) {

    switch (currClkState) {

        case CLK_STATE_OUTPUT_BIT:
            
            switch (currDataState) {
                case SEND_START_BIT:
                    output_low(keyboard_host_data);
                    currDataState++;
                    break;
                case SEND_DATA_BIT:
                    int currBit = bit_test(tx_data, bit_index);
                    output_bit(keyboard_host_data, currBit);
                    parity ^= currBit;
                    bit_index++;
                    if (bit_index == 8) {
                        currDataState++;
                    }
                    break;
                case SEND_PARITY_BIT:
                    output_bit(keyboard_host_data, parity);
                    currDataState++;
                    break;
                case SEND_STOP_BIT:
                    output_high(keyboard_host_data);
                    currDataState++;
                    break;
            }
            set_timer1(TIMER_CLOCKP_H_START);
            break;

        case CLK_STATE_LOW:

            output_low(keyboard_host_clock);
            set_timer1(TIMER_CLOCKP_START);
            break;

        case CLK_STATE_HIGH:

            /*if(currDataState == SEND_FINISH_WAIT){
                        set_timer2(65535); //add delay to end of scancode
                        output_float(keyboard_host_data); //set data to high impedance
                        output_float(keyboard_host_clock); //set clock to high impedance
                        currDataState++;
                        return;
                    }
                    else */
            if (currDataState == SEND_FINISH) { //we are done transmitting the packet
                //output_high(keyboard_host_clock);
                //output_high(keyboard_host_data);
                output_float(keyboard_host_data); //set data to high impedance
                output_float(keyboard_host_clock); //set clock to high impedance
                disable_interrupts(INT_TIMER1); //turn off tx handler
                doneTx = TRUE;
                return;
            } else {
                output_high(keyboard_host_clock);
                set_timer1(TIMER_CLOCKP_H_START);
            }
            currClkState = -1; //reset clock line state machine
            break;

    }
    currClkState++;
}

#INT_EXT0
void ext0_isr(void)
{
    SCANCODE s;
    s.delay = 0;
    output_toggle(keyboard_host_led);
    if(edge[0] == L_TO_H){ //key press detected
        edge[0] = H_TO_L;
        ext_int_edge(0, H_TO_L);
        s.scancode = KEY_A;
        push(s);
    } else { //key release detected
        edge[0] = L_TO_H;
        ext_int_edge(0, L_TO_H);
        s.scancode = BREAK_CODE;
        push(s);
        s.scancode = KEY_A;
        push(s);
    }
    clear_interrupt(INT_EXT0);
}

#INT_EXT1
void ext1_isr(void)
{
    SCANCODE s;
    s.delay = 0;
    output_toggle(keyboard_host_led);
    if(edge[1] == L_TO_H){ //key press detected
        edge[1] = H_TO_L;
        ext_int_edge(1, H_TO_L);
        s.scancode = KEY_B;
        push(s);
    } else { //key release detected
        edge[1] = L_TO_H;
        ext_int_edge(1, L_TO_H);
        s.scancode = BREAK_CODE;
        push(s);
        s.scancode = KEY_B;
        push(s);
    }
    clear_interrupt(INT_EXT1);
}

#INT_EXT2
void ext2_isr(void)
{
    SCANCODE s;
    s.delay = 0;
    output_toggle(keyboard_host_led);
    if(edge[2] == L_TO_H){ //key press detected
        edge[2] = H_TO_L;
        ext_int_edge(2, H_TO_L);
        s.scancode = KEY_C;
        push(s);
    } else { //key release detected
        edge[2] = L_TO_H;
        ext_int_edge(2, L_TO_H);
        s.scancode = BREAK_CODE;
        push(s);
        s.scancode = KEY_C;
        push(s);
    }
    clear_interrupt(INT_EXT2);
}

#INT_EXT3
void ext3_isr(void)
{
    SCANCODE s;
    s.delay = 0;
    output_toggle(keyboard_host_led);
    if(edge[3] == L_TO_H){ //key press detected
        edge[3] = H_TO_L;
        ext_int_edge(3, H_TO_L);
        s.scancode = KEY_D;
        push(s);
    } else { //key release detected
        edge[3] = L_TO_H;
        ext_int_edge(3, L_TO_H);
        s.scancode = BREAK_CODE;
        push(s);
        s.scancode = KEY_D;
        push(s);
    }
    clear_interrupt(INT_EXT3);
}

static void update_switches(void) {
    portInput = input_b();
    portChange = lastPortState ^ portInput;
    lastPortState = portInput;
    if (bit_test(portChange, 2)) {
        output_toggle(keyboard_host_led);
        SCANCODE s;
        s.delay = 0;
        if (bit_test(lastPortState, 2)) { //key press detected
            s.scancode = KEY_A;
            push(s);
        } else {
            s.scancode = BREAK_CODE;
            push(s); //key release detected
            s.scancode = KEY_A;
            push(s);
        }
        delay_us(DEBOUNCE_DELAY);
    }if (bit_test(portChange, 3)) {
        SCANCODE s;
        s.delay = 0;
        output_toggle(keyboard_host_led);
        if (bit_test(lastPortState, 3)) {
            s.scancode = KEY_B;
            push(s);
        } else {
            s.scancode = BREAK_CODE;
            push(s); //key release detected
            s.scancode = KEY_B;
            push(s);
        }
        delay_us(DEBOUNCE_DELAY);
    }if (bit_test(portChange, 4)) {
        SCANCODE s;
        s.delay = 0;
        output_toggle(keyboard_host_led);
        if (bit_test(lastPortState, 4)) {
            s.scancode = KEY_C;
            push(s);
        } else {
            s.scancode = BREAK_CODE;
            push(s); //key release detected
            s.scancode = KEY_C;
            push(s);
        }
        delay_us(DEBOUNCE_DELAY);
    }
}

/* -------------------------------------------------------------------------- */