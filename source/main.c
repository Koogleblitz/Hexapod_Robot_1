/*	Author: Richard Tobing, rlumb001@ucr.edu
 *  Partner(s) Name: 
 *	Lab Section: 21
 *	Assignment:	Project Demo 1
 *	Exercise Description: [optional - include for your own benefit]
 *
 *	I acknowledge all content contained herein, excluding timer.h or example
 *	code, is my own original work.
	Demo link:	
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#ifdef _SIMULATE_
#include "simAVRHeader.h"
#include "io.h"
#include <stdio.h>
//#include <string>
#endif
//////////////////////////////////////////////////////////////////////Headers////////////////////////////////////////////////////////////////////////////////
// Permission to copy is granted provided that this header remains intact. 
// This software is provided with no warranties.
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
//Functionality - Sets bit on a PORTx
//Parameter: Takes in a uChar for a PORTx, the pin number and the binary value 
//Returns: The new value of the PORTx
unsigned char SetBit(unsigned char pin, unsigned char number, unsigned char bin_value) 
{
	return (bin_value ? pin | (0x01 << number) : pin & ~(0x01 << number));
}

////////////////////////////////////////////////////////////////////////////////
//Functionality - Gets bit from a PINx
//Parameter: Takes in a uChar for a PINx and the pin number
//Returns: The value of the PINx
unsigned char GetBit(unsigned char port, unsigned char number) 
{
	return ( port & (0x01 << number) );
}
//------------------------------------------------------------------/bit.h---------------------------------------------------------------------//


//----------------------------------------------------------------keypad.h---------------------------------------------------------------------//
// Returns '\0' if no key pressed, else returns char '1', '2', ... '9', 'A', ...
// If multiple keys pressed, returns leftmost-topmost one
// Keypad must be connected to port C
// Keypad arrangement
//        Px4 Px5 Px6 Px7
//	  col 1   2   3   4
//  row  ______________
//Px0 1	| 1 | 2 | 3 | A
//Px1 2	| 4 | 5 | 6 | B
//Px2 3	| 7 | 8 | 9 | C
//Px3 4	| * | 0 | # | D
//#ifndef KEYPAD_H
//#define KEYPAD_H
//#include <bit.h>

// Keypad Setup Values
#define KEYPADPORT PORTC
#define KEYPADPIN  PINC
#define ROW1 0
#define ROW2 1
#define ROW3 2
#define ROW4 3
#define COL1 4
#define COL2 5
#define COL3 6
#define COL4 7

//Functionality - Gets input from a keypad via time-multiplexing
//Parameter: None
//Returns: A keypad button press else '\0'
unsigned char GetKeypadKey() {
	// Check keys in col 1
	KEYPADPORT = SetBit(0xFF,COL1,0); // Set Px4 to 0; others 1
	asm("nop"); // add a delay to allow PORTx to stabilize before checking
	if ( GetBit(~KEYPADPIN,ROW1) ) { return '1'; }
	if ( GetBit(~KEYPADPIN,ROW2) ) { return '4'; }
	if ( GetBit(~KEYPADPIN,ROW3) ) { return '7'; }
	if ( GetBit(~KEYPADPIN,ROW4) ) { return '*'; }
	// Check keys in col 2
	KEYPADPORT = SetBit(0xFF,COL2,0); // Set Px5 to 0; others 1
	asm("nop"); // add a delay to allow PORTx to stabilize before checking
	if ( GetBit(~KEYPADPIN,ROW1) ) { return '2'; }
	if ( GetBit(~KEYPADPIN,ROW2) ) { return '5'; }
	if ( GetBit(~KEYPADPIN,ROW3) ) { return '8'; }
	if ( GetBit(~KEYPADPIN,ROW4) ) { return '0'; }
	// Check keys in col 3
	KEYPADPORT = SetBit(0xFF,COL3,0); // Set Px6 to 0; others 1
	asm("nop"); // add a delay to allow PORTx to stabilize before checking
	if ( GetBit(~KEYPADPIN,ROW1) ) { return '3'; }
	if ( GetBit(~KEYPADPIN,ROW2) ) { return '6'; }
	if ( GetBit(~KEYPADPIN,ROW3) ) { return '9'; }
	if ( GetBit(~KEYPADPIN,ROW4) ) { return '#'; }
	// Check keys in col 4
	KEYPADPORT = SetBit(0xFF,COL4,0); // Set Px7 to 0; others 1
	asm("nop"); // add a delay to allow PORTx to stabilize before checking
	if (GetBit(~KEYPADPIN,ROW1) ) { return 'A'; }
	if (GetBit(~KEYPADPIN,ROW2) ) { return 'B'; }
	if (GetBit(~KEYPADPIN,ROW3) ) { return 'C'; }
	if (GetBit(~KEYPADPIN,ROW4) ) { return 'D'; }
	return '\0';
}
//----------------------------------------------------------------------/keypad.h------------------------------------------------------------------------//


//----------------------------------------------------------------------scheduler.h---------------------------------------------------------------------//
//#ifndef SCHEDULER_H
//#define SCHEDULER_H
//Functionality - finds the greatest common divisor of two values
//Parameter: Two long int's to find their GCD
//Returns: GCD else 0
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

////----------------------------------------motorPulseWidthModulator----------------------------------------------////
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
////----------------------------------------/motorPulseWidthModulatorh----------------------------------------------////

/////////////////////////////////////////////////////////////////////\Headers////////////////////////////////////////////////////////////////////////////////////











//-----------------shared vars---------------//
unsigned char keyOut = 7;
unsigned char keypadIn;
unsigned short playerCol;
//---------------\shared vars----------------//



//-------------other global vars-------------//
unsigned short collumn = 1;
unsigned short collumn2 = 1;
unsigned char pseBtn = 1;
unsigned char gameOverFlag = 0;
unsigned char prev;

//unsigned long motorPulseCnt = 0;
unsigned char motorPulse = 0;
unsigned char dir;
//unsigned char dir1;
//unsigned char dir2;
unsigned char led;
//-----------\other global vars--------------//



//---------------------------------------States----------------------------------------------//
enum displayStates{displayOut, displayOut2, gameOver};
enum getInput{play, wait, pause};
//---------------------------------------\States----------------------------------------------//




//-------------------------------------------------------------------State Machines----------------------------------------------------//
int KeypadTick(int state){	

	
	unsigned char C = ~PINC & (128 + 64 + 32);
	motorPulse = !motorPulse;
	

	switch(state){
		case wait:
			if(C == 32){state = wait;	prev = 0;}
			else{state = play;	}	
			
		break;

		case play: 
			if(C == 32){ state = pause;	prev = 1;}
			else{state = play;   }
			//state = play;	

		break;

		case pause:
			state = (prev)? wait : play;
			//state = wait;
		break;

		default: state = play;	
		break;
	
	}
	//-----------------------------------------
	switch(state){
		case play:			
			if(!pseBtn){
				
				/*     left     */
				if((yAxisADC()>750) ){
					//dir1 = 1;
					//dir2 = 1;
					//dir = 0b00001010;
					  dir = 0b00000100; //right wheel forwardS
					//dir = 0b00000010; //right wheel backwarrd
					
					led = 1;
				}

				/*     down     */
				else if(xAxisADC() < 350){
					//dir1 = 0;SSS
					//dir2 = 1;
					//dir = 0b00000010; //right wheel backwrrd
					//dir = 0b00001010; //righ wheel backward
					  dir = 0b00010010; //left wheel forward
					led = 2;

				}

				/*      up      */
				else if((xAxisADC() > 750)  ){
					//dir1 = 1;
					//dir2 = 0;
					//dir = 0b00001000;
					led = 4;
				}
				
				/*      right      */
				else if(yAxisADC() < 350){
					//dir1 = 2;
					//dir2 = 2;
					//dir = 0b00010100;
					led = 8;
				}

				else{
					//dir1 = 0;
					//dir2 = 0;
					dir = 0;
				}
			}

		
			
		break;

		case wait: 
		break;

		case pause:
			pseBtn = !pseBtn; 
			gameOverFlag = 0;
			//dir1 = 0;
			//dir2 = 0;
			dir = 0;
		break;
	}
	return state;
}


int OutputTick(int state){
	
	switch(state){
		case displayOut: 
			state = displayOut;		
		break;

		case displayOut2:
			state = displayOut;		
		break;
		
		default: state = displayOut;	
		break;
	}
	//-----------------------------------------
	switch(state){
		case displayOut:	
			PORTB = dir + motorPulse + (motorPulse << 5);
			//PORTB = led;
			
		break;

		case displayOut2:
			if(!gameOverFlag){
				LCD_DisplayString(32-collumn2, "     #");
				LCD_Cursor(playerCol);	
			}
			else{LCD_DisplayString(1, " Game_Over");}
		break;
			

	
	}	
	return state;		
}


//---------------------------------------------------------------------\State Machines-----------------------------------------------------------------//





int main(void) {
		
		/*Port c is used for keypad input; half of the port should be input, the other half should be output*/
    		DDRA = 0x00;	PORTA = 0xFF;
		DDRB = 0xFF;	PORTB = 0x00;
		DDRC = 0x00; 	PORTC = 0xFF;
		DDRD = 0xFF; 	PORTD = 0x00;
 		LCD_init();
		ADC_init();
		PWM_on();
		

		//Declare the task objects
		//NOTE: thee "task" variable type/struct is defined in the scheduler code
		static task task0, task1;

		//Declare the array of pointers that point to the tasks
		task *tasks[] = {&task0, &task1};
		const unsigned short numTasks = sizeof(tasks)/sizeof(task*);
		const char start = -1;

	
		//set the fields of each task, this could probably be done in a loop idk
		task0.state = start;
		task0.period = 30;
		task0.elapsedTime = task0.period;
		task0.TickFct = &KeypadTick;

		task1.state = start;
		task1.period = 10; 
		task1.elapsedTime = task1.period;
		task1.TickFct = &OutputTick;


		unsigned long GCD = tasks[0]->period;
		for(unsigned int i = 1; i < numTasks; i++){
			GCD = findGCD(GCD,tasks[i]->period);
		}

		TimerSet(GCD);
		TimerOn();


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
                                         
