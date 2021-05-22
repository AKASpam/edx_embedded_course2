// TuningFork.c Lab 12
// Runs on LM4F120/TM4C123
// Use SysTick interrupts to create a squarewave at 440Hz.  
// There is a positive logic switch connected to PA3, PB3, or PE3.
// There is an output on PA2, PB2, or PE2. The output is 
//   connected to headphones through a 1k resistor.
// The volume-limiting resistor can be any value from 680 to 2000 ohms
// The tone is initially off, when the switch goes from
// not touched to touched, the tone toggles on/off.
//                   |---------|               |---------|     
// Switch   ---------|         |---------------|         |------
//
//                    |-| |-| |-| |-| |-| |-| |-|
// Tone     ----------| |-| |-| |-| |-| |-| |-| |---------------
//
// Daniel Valvano, Jonathan Valvano
// January 15, 2016

/* This example accompanies the book
   "Embedded Systems: Introduction to ARM Cortex M Microcontrollers",
   ISBN: 978-1469998749, Jonathan Valvano, copyright (c) 2015

 Copyright 2016 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */


#include "TExaS.h"
#include "..//tm4c123gh6pm.h"


// basic functions defined at end of startup.s
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void WaitForInterrupt(void);  // low power mode
void switch_init(void);
unsigned long delay=0;
unsigned long beep=0; //variable for beep
unsigned long SW1=0; //switch 1 pressed
volatile unsigned long Counts=0; 


//port definitions
#define GPIO_PORTA_DATA_R       (*((volatile unsigned long *)0x400043FC))
#define GPIO_PORTA_DIR_R        (*((volatile unsigned long *)0x40004400))
#define GPIO_PORTA_AFSEL_R      (*((volatile unsigned long *)0x40004420))
#define GPIO_PORTA_DEN_R        (*((volatile unsigned long *)0x4000451C))
#define GPIO_PORTA_AMSEL_R      (*((volatile unsigned long *)0x40004528))
#define GPIO_PORTA_PCTL_R       (*((volatile unsigned long *)0x4000452C))
#define PA2                     (*((volatile unsigned long *)0x40004010))
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
#define SYSCTL_RCGC2_GPIOF      0x00000020  // port F Clock Gating Control



// input from PA3, output from PA2, SysTick interrupts
void Sound_Init(void){ 
//read pa3 button status and toogle beep
	
	/*
	if(GPIO_PORTA_RIS_R&0x08){  // SW2 touch
    GPIO_PORTA_ICR_R = 0x08;  // acknowledge flag0
    beep=1;
	}

*/	
	
}

void GPIO_PortA_Handler(void){ //do something with PA2 input
	
	if(GPIO_PORTA_RIS_R&0x08){  // SW1 touch
    GPIO_PORTA_ICR_R = 0x08;  // acknowledge flag0
		beep=1;
	/* reinkopiert aus 12_1
if(GPIO_PORTF_RIS_R&0x01){  // SW2 touch
    GPIO_PORTF_ICR_R = 0x01;  // acknowledge flag0
    if(L>8000) L = L-8000;    // slow down
  }
  if(GPIO_PORTF_RIS_R&0x10){  // SW1 touch
    GPIO_PORTF_ICR_R = 0x10;  // acknowledge flag4
    if(L<72000) L = L+8000;   // speed up
  }
  H = 80000-L; // constant period of 1ms, variable duty cycle	
}
*/
	}
}
void switch_init(void){ //interrupt button k�se
	//switch init:
	SYSCTL_RCGCGPIO_R |= 0x00000001; // (a) activate clockactivated  for A
  //delay = SYSCTL_RCGCGPIO_R; - maybe not necessary 
  //GPIO_PORTF_LOCK_R = 0x4C4F434B; // unlock GPIO Port F
  //GPIO_PORTF_CR_R = 0x11;         // allow changes to PF4,0
//  GPIO_PORTF_DIR_R &= ~0x11;    // (c) make PF4,0 in (built-in button)
 // GPIO_PORTF_DEN_R |= 0x11;     //     enable digital I/O on PF4,0
//  GPIO_PORTF_PUR_R |= 0x11;     //     enable weak pull-up on PF4,0
  GPIO_PORTA_IS_R &= ~0x08;     // (d) PA3 is edge-sensitive
  GPIO_PORTA_IBE_R &= ~0x08;    //     PA3 is not both edges
  GPIO_PORTA_IEV_R &= ~0x08;    //     PA3 falling edge event
  GPIO_PORTA_ICR_R = 0x08;      // (e) clear flags 4,0
  GPIO_PORTA_IM_R |= 0x08;      // (f) arm interrupt on PF4,PF0
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00400000; // (g) priority 2
  NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC
	
	//kopierter k�se aus 12_1
	/*
	volatile uint32_t FallingEdges = 0;
void EdgeCounter_Init(void){       
  SYSCTL_RCGCGPIO_R |= 0x00000020; // (a) activate clock for port F
  FallingEdges = 0;             // (b) initialize count and wait for clock
  GPIO_PORTF_DIR_R &= ~0x10;    // (c) make PF4 in (built-in button)
  GPIO_PORTF_DEN_R |= 0x10;     //     enable digital I/O on PF4
  GPIO_PORTF_PUR_R |= 0x10;     //     enable weak pull-up on PF4
  GPIO_PORTF_IS_R &= ~0x10;     // (d) PF4 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x10;    //     PF4 is not both edges
  GPIO_PORTF_IEV_R &= ~0x10;    //     PF4 falling edge event
  GPIO_PORTF_ICR_R = 0x10;      // (e) clear flag4
  GPIO_PORTF_IM_R |= 0x10;      // (f) arm interrupt on PF4
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000; // (g) priority 5
  NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC
  EnableInterrupts();           // (i) Enable global Interrupt flag (I)
	*/
}

void SysTick_Init(unsigned long period){ // priority 2
  NVIC_ST_CTRL_R = 0;         // disable SysTick during setup
  NVIC_ST_RELOAD_R = period-1;// reload value
  NVIC_ST_CURRENT_R = 0;      // any write to current clears it
  NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x00FFFFFF)|0x40000000;          
  NVIC_ST_CTRL_R = 0x07; // enable SysTick with core clock and interrupts
  // finish all initializations and then enable interrupts
}

// called at 880 Hz
void SysTick_Handler(void){

	if(beep==1){
	GPIO_PORTA_DATA_R ^= 0x04;       // toggle PA2
  }
		//Counts = Counts + 1; auskommentiert - braucht es diese zeile ?
}

int main(void){// activate grader and set system clock to 80 MHz
  TExaS_Init(SW_PIN_PA3, HEADPHONE_PIN_PA2,ScopeOn); 
  Sound_Init();         
  EnableInterrupts();   // enable after all initialization are done
	GPIO_PortA_Handler();
	//switch_init(void);
	//insert portA pin 3 init here:
	
	SYSCTL_RCGC2_R |= 0x01; //activate PortA 0x00000001
	//RCGCGPIO |= 0x01; //Enable clock for PORTA - found online ; does not work
	//GPIODATA |= 0x01;
	SYSCTL_RCGC2_R |= 0x00000020; //activate PortF 
	GPIO_PORTA_DIR_R |= 0x04;   // make PA3 input PA2output
	GPIO_PORTA_AFSEL_R &= ~0x04;// disable alt funct on PA2 (0x04 wrong - bit 3 should be high /change &~ to |
  GPIO_PORTA_DEN_R |= 0x0E;   // enable digital I/O on PA2 & PA3
  GPIO_PORTA_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFFFF0FF)+0x00000000;
  GPIO_PORTA_AMSEL_R &= ~0x04;     // disable analog functionality on PF
  SysTick_Init(90909);        // initialize SysTick timer, every 1ms - changed from 16000; 80000 is too small
  EnableInterrupts();         // enable after everything initialized
	



  while(1){
    // main program is free to perform other tasks
    // do not use WaitForInterrupt() here, it may cause the TExaS to crash
		//SW1=GPIO_PORTA_DATA_R&0x08; //read PA2
		//if(SW1==0x08){  // SW1 touch
    //GPIO_PORTA_ICR_R = 0x08;  // acknowledge flag0
    //beep=1;}
		if(GPIO_PORTA_DATA_R&0x08){  // SW1 touch
    //GPIO_PORTA_ICR_R = 0x08;  // acknowledge flag0
		beep=1;
		}	
		else{
		beep=0;
		GPIO_PORTA_DATA_R |= ~0x04; //write 0
		}
		}
		}
	
				

