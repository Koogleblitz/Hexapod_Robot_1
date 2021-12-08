/*	Author: Richard Tobing, rlumb001@ucr.edu
 *  Partner(s) Name: 
 *	Lab Section: 21
 *	Assignment:	Project Demo Final
 *	Exercise Description: [robot.c : This is the robot's code for receiving the bluetooth USART signal]
 *
 *	I acknowledge all content contained herein, excluding timer.h or example
 *	code, is my own original work.
	Demo link:	
 */


/*TODO List 8pm :
	[x]Complexity 1: Joystick
	[x]Complexity 2: DC motors + Motor Drivers
	[x]Complexity 3: Bluetooth + USART (phone)
	[x]Wired Mode: Movieement via joystick
	[x]Wireless Mode : Movement with Bluetooth + Phone  
		Bluetooth RC Car App (https://drive.google.com/store/apps/details?id=braulio.calle.bluetoothRCcontroller)
	[x]Adjust periods
	[x]Report
	[x]Adjust Joystick ADC
	[x]Safety Submission

	[x]Turn in at 5:02pm thinking it was due at 5pm
	[x]Proceed to having a 25 minute heart attack
	[x]realize that its not due until midnight, proceed to building an awesome robot
	
  	[x]Update Robot.c so int only calls the ADC function once per tick
  	[x]clean up code, omitt unused libraries, rename stuff
  	[x]Improve State machine
	[x]Rewire buttons:  gear2/RTrig: PB7(128), pause/midBtn: PB6(64), brake/LTrig: PB5(32)
  	[x]Add Breaks
	[x]Add proper turning 
  	[x]add 2nd gear (for speed)
	[x]update Report
	[x]Safety Submission

  	[]Collision detection with ultrasonic sensor
  	[]self driving mode
	[]Safety Submission
	
	[]Bluetooth with another breadboard
  	[]Safety Submission
  
  	[]Add Legs
  	[]Final Submission
	
	[]do laundry
	[]walk the dog
	[]cure cancer
	[]kill voldermort
	[]return sauron's ring to mordor

	[]Add legs. this is what seperates a "robot" from a meer "car"
	[]Show off to ur friends so they know what a bigbrain engeineer you are
*/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#ifdef _SIMULATE_
#include "simAVRHeader.h"
#include "io.h"
#include <stdio.h>
#endif
//////////////////////////////////////////////////////////////////////Headers////////////////////////////////////////////////////////////////////////////////
// Permission to copy is granted provided that this header remains intact. 										/////
// This software is provided with no warranties.													/////
//
//--------------------------------------------------------------timer.h----------------------------------------------------------------------------//
volatile unsigned char TimerFlag = 0; 
unsigned long _avr_timer_M = 1; 
unsigned long _avr_timer_cntcurr = 0; 
void TimerOn() {
  TCCR1B = 0x0B;
  OCR1A = 125;
  TIMSK1 = 0x02;
  TCNT1 = 0;
  _avr_timer_cntcurr = _avr_timer_M;
  SREG |= 0x80;
}
void TimerOff() {TCCR1B = 0x00;}
void TimerISR() {TimerFlag = 1;}
ISR(TIMER1_COMPA_vect) {
	_avr_timer_cntcurr--; 
	if(_avr_timer_cntcurr == 0) { 
		TimerISR(); 
		_avr_timer_cntcurr = _avr_timer_M;
	}
}
void TimerSet(unsigned long M) {
	_avr_timer_M = M;
	_avr_timer_cntcurr = _avr_timer_M;
}
//--------------------------------------------------------------/timer.h---------------------------------------------------------------------------//


//----------------------------------------------------------------bit.h---------------------------------------------------------------------//
//#ifndef BIT_H
//#define BIT_H
unsigned char SetBit(unsigned char pin, unsigned char number, unsigned char bin_value) 
{return (bin_value ? pin | (0x01 << number) : pin & ~(0x01 << number));}
unsigned char GetBit(unsigned char port, unsigned char number) 
{return ( port & (0x01 << number) );}
//------------------------------------------------------------------/bit.h---------------------------------------------------------------------//


//----------------------------------------------------------------------scheduler.h---------------------------------------------------------------------//
//#ifndef SCHEDULER_H
//#define SCHEDULER_H
unsigned long int findGCD(unsigned long int a, unsigned long int b){
	unsigned long int c;
	while(1){
		c = a % b;
		if( c == 0 ) { return b; }
		a = b;
		b = c;
	}
	return 0;
} 
//Struct for Tasks represent a running process in our simple real-time operating system
typedef struct _task{
	// Tasks should have members that include: state, period,
	//a measurement of elapsed time, and a function pointer.
	signed 	 char state; 		//Task's current state
	unsigned long period; 		//Task period
	unsigned long elapsedTime; 	//Time elapsed since last task tick
	int (*TickFct)(int); 		//Task tick function
} task;
//---------------------------------------------------------------------/scheduler.h--------------------------------------------------------------------//

//-------------------------------------------------ADC------------------------------------------//
void ADC_init() {ADCSRA |= (1 << ADEN) | (1 << ADSC) | (1 << ADATE); }

unsigned short xAxisADC(void){
	ADMUX = 0b01000000; 
	ADCSRA |= (1<<ADSC);
	while(!(ADCSRA & (1<<ADIF)));
	return(ADC);
}

unsigned short yAxisADC(void){
	ADMUX = 0b01000001; 
	ADCSRA |= (1<<ADSC);
	while(!(ADCSRA & (1<<ADIF)));
	return(ADC);
}
//-------------------------------------------------/ADC------------------------------------------//

////----------------------------------------PulseWidthModulator----------------------------------------------////
void set_PWM(double frequency) {
    static double current_frequency;
    if (frequency != current_frequency) {
        if (!frequency) { TCCR3B &= 0x08; }
        else { TCCR3B |= 0x03; }
        
        if (frequency < 0.954) { OCR3A = 0xFFFF; }
        else if (frequency > 31250) { OCR3A = 0x0000; }
        else { OCR3A = (short) (8000000 / (128 * frequency)) - 1; }
        
        TCNT3 = 0;
        current_frequency = frequency;
    }
}
void PWM_on() {
    TCCR3A = (1 << COM3A0);
    TCCR3B = (1 << WGM32) | (1 << CS31) | (1 << CS30);
    set_PWM(0);
}
void PWM_off() {
    TCCR3A = 0x00;
    TCCR3B = 0x00;
}
////----------------------------------------/PulseWidthModulatorh----------------------------------------------////

////----------------------------------------USART----------------------------------------------////
#ifndef USART_H
#define USART_H
// USART Setup Values
#define F_CPU 8000000UL // Assume uC operates at 8MHz
#define BAUD_RATE 9600
#define BAUD_PRESCALE (((F_CPU / (BAUD_RATE * 16UL))) - 1)
////////////////////////////////////////////////////////////////////////////////
//Functionality - Initializes TX and RX on PORT D
//Parameter: None
//Returns: None
void initUSART()
{
	/*Richard's Note: 
	these are the non-obvious changes I made to update this old USART library to work with the the Atmega1284p:
	URSEL is omitted, UCSZ0 to UCSZ00, UCSZ1 to UCSZ01
	other changes are simpler, like putting a 0 right before the last letter of the register name*/

	// Turn on the reception circuitry
	// Use 8-bit character sizes - URSEL bit set to select the UCRSC register
	// Turn on receiver and transmitter
	UCSR0B |= (1 << RXEN0)  | (1 << TXEN0);
	UCSR0C |=  (1 << UCSZ00) | (1 << UCSZ01);
	// Load lower 8-bits of the baud rate value into the low byte of the UBRR0L register
	UBRR0L = BAUD_PRESCALE;
	// Load upper 8-bits of the baud rate value into the high byte of the UBRR register
	UBRR0H = (BAUD_PRESCALE >> 8);
}

//Functionality - checks if USART is ready to send
//Parameter: None
//Returns: 1 if true else 0
unsigned char USART_IsSendReady()
{
	return (UCSR0A & (1 << UDRE0));
}

//Functionality - checks if USART has recieved data
//Parameter: None
//Returns: 1 if true else 0
unsigned char USART_HasTransmitted()
{
	return (UCSR0A & (1 << TXC0));
}

// **** WARNING: THIS FUNCTION BLOCKS MULTI-TASKING; USE WITH CAUTION!!! ****
//Functionality - checks if USART has recieved data
//Parameter: None
//Returns: 1 if true else 0
unsigned char USART_HasReceived()
{
	return (UCSR0A & (1 << RXC0));
}

//Functionality - Flushes the data register
//Parameter: None
//Returns: None
void USART_Flush()
{
	static unsigned char dummy;
	while ( UCSR0A & (1 << RXC0) ) { dummy = UDR0; }
}

// **** WARNING: THIS FUNCTION BLOCKS MULTI-TASKING; USE WITH CAUTION!!! ****
//Functionality - Sends an 8-bit char value
//Parameter: Takes a single unsigned char value
//Returns: None
void USART_Send(unsigned char sendMe)
{
	while( !(UCSR0A & (1 << UDRE0)) );
	UDR0 = sendMe;
}

// **** WARNING: THIS FUNCTION BLOCKS MULTI-TASKING; USE WITH CAUTION!!! ****
//Functionality - receives an 8-bit char value
//Parameter: None
//Returns: Unsigned char data from the receive buffer
unsigned char USART_Receive()
{
	while ( !(UCSR0A & (1 << RXC0)) ); // Wait for data to be received
	return UDR0; // Get and return received data from buffer
}

#endif //USART_H
////----------------------------------------/USART----------------------------------------------////
////																			     ////	
////																			     ////	
/////////////////////////////////////////////////////////////////////\Headers////////////////////////////////////////////////////////////////////////////////////





//-----------------Global Vars---------------//
unsigned char pseBtn = 1;
unsigned char prev;
unsigned char motorPulse = 0;
unsigned char dir;
unsigned char led;
unsigned long PWM = 0;
//-----------\Global Vars--------------//



//---------------------------------------States----------------------------------------------//
enum outputStates{sendOutput, wait};
enum getInput{drive, standby, pause};
//---------------------------------------\States----------------------------------------------//




//-------------------------------------------------------------------State Machines----------------------------------------------------//
int rxTick(int state){	
	unsigned char C = ~PINC & (32 + 64 + 128);
	motorPulse = !motorPulse;
	unsigned char rxBT = USART_Receive();
	
	switch(state){
		case standby:
			if(C == 64){state = standby;	prev = 0;}
			else{state = drive;	}	
		break;
		case drive: 
			if(C == 64){ state = pause;	prev = 1;}
			else{state = drive;   }
		break;
		case pause:
			state = (prev)? standby : drive;
		break;
		default: state = drive;	
		break;
	}
	//-----------------------------------------
	switch(state){
		case drive:			
			if(!pseBtn){
				/*     up/forward     */
				if((rxBT == 'F') ){
					dir = 0b00010100;
					led = 2;
				}
				/*     down/back     */
				else if(rxBT == 'B'){
					dir = 0b00001010;
					led = 4;
				}
				/*      left      */
				else if(rxBT == 'L'){
					dir = 0b00000100;                               //Rotate
					//dir = (PWM%3)? 0b00000100 : 0b00010000;	//Turn
					led = 8;
				}
				/*      right      */
				else if(rxBT == 'R'){
					dir = 0b00010000;				//Rotate
					//dir = (PWM%3)? 0b00010000 : 0b00000100;	//Turn
					led = 1;
				}
				/*      standby      */
				else{
					dir = 0;
					led = 0;
				}
			}
			USART_Flush();
		break;
		case standby: 
		break;
		case pause:
			pseBtn = !pseBtn; 
			led = 0;
			dir = 0;
		break;
	}
	return state;
}


int OutputTick(int state){
	unsigned char brake = ~PINC & (32);   
	unsigned char gear2 = ~PINC & (128);  
	PWM = (PWM <= 1000)? (PWM+1) : 0; 
	
	switch(state){
		case sendOutput: 
			state = (brake)? wait : sendOutput;		
		break;
		case wait:
			state = (brake)? wait : sendOutput;		
		break;
		default: state = sendOutput;	
		break;
	}
	//-----------------------------------------
	switch(state){
		case sendOutput:	
			//PORTB = dir + motorPulse + (motorPulse << 5);
			if(gear2){PORTB = dir + motorPulse + (motorPulse << 5);	}
			else{PORTB = (PWM%2)? 0 : dir + motorPulse + (motorPulse << 5);}
		break;
		case wait:
			PORTB = 0;
		break;
	}	
	return state;		
}


//---------------------------------------------------------------------\State Machines-----------------------------------------------------------------//



int main(void) {
    		DDRA = 0x00;	PORTA = 0xFF;
		DDRB = 0xFF;	PORTB = 0x00;
		DDRC = 0x00; 	PORTC = 0xFF;
		DDRD = 0xFF; 	PORTD = 0x00;
 		
		//#if 0
		LCD_init();
		ADC_init();
		PWM_on();
		initUSART();
		

		//Declare the task objects
		//NOTE: thee "task" variable type/struct is defined in the scheduler code
		static task task0, task1;

		//Declare the array of pointers that point to the tasks
		task *tasks[] = {&task0, &task1};
		const unsigned short numTasks = sizeof(tasks)/sizeof(task*);
		const char start = -1;

	
		//set the fields of each task, this could probably be done in a loop idk
		task0.state = start;
		task0.period = 20;
		task0.elapsedTime = task0.period;
		task0.TickFct = &rxTick;

		task1.state = start;
		task1.period = 10; 
		task1.elapsedTime = task1.period;
		task1.TickFct = &OutputTick;


		unsigned long GCD = tasks[0]->period;
		for(unsigned int i = 1; i < numTasks; i++){
			GCD = findGCD(GCD,tasks[i]->period);
		}
		//#endif

		TimerSet(GCD);
		TimerOn();
		
		//PORTB = 0b00110101;  //forward

   unsigned short i;
    while (1) {
        for (i = 0; i < numTasks; i++) {
            if (tasks[i]->elapsedTime == tasks[i]->period) {
                tasks[i]->state = tasks[i]->TickFct(tasks[i]->state);
                tasks[i]->elapsedTime = 0;
            }
            tasks[i]->elapsedTime += GCD;
        }
        while (!TimerFlag);
        TimerFlag = 0;
    }
    return 0;
}
                                         
