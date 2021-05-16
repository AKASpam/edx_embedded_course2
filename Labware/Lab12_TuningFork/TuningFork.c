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
	GPIO_PORTA_DATA_R ^= 0x04;       // toggle PA2
  Counts = Counts + 1;
}

int main(void){// activate grader and set system clock to 80 MHz
  TExaS_Init(SW_PIN_PA3, HEADPHONE_PIN_PA2,ScopeOn); 
  Sound_Init();         
  EnableInterrupts();   // enable after all initialization are done
	
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
  //SysTick_Init(80000);        // initialize SysTick timer, every 1ms - changed from 16000
  EnableInterrupts();         // enable after everything initialized
	
  //GPIO_PORTA_PCTL_R &= ~0x00000F00; // 3) regular GPIO

	
	
	
	
  while(1){
    // main program is free to perform other tasks
    // do not use WaitForInterrupt() here, it may cause the TExaS to crash
		SysTick_Handler();
  }
}
