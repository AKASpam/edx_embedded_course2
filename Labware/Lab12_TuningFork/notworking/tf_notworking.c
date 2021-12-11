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
//void switch_init(void);
unsigned long delay=0;
unsigned long beep=0; //variable for beep
unsigned long SW1=0; //switch 1 pressed
unsigned long old=0; //old switch status
unsigned long edgecounter=0;
unsigned long Hurtz=0;
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
	SYSCTL_RCGC2_R |= 0x00000001; // (a) activate clockactivated  for A (wirte a 0)
	edgecounter=0; //edge counter set to 0
	GPIO_PORTA_DIR_R |= 0x04;   // make PA3 input (0)  PA2 output (1) (0100)- 0x04 (set to 1)
	GPIO_PORTA_AFSEL_R &= ~0x08; //AFSEL (Alternate Function Select) register
	GPIO_PORTA_DEN_R |= 0x0C; //DEN bit 2 should be high DEN (Digital Enable) 0x08?
	GPIO_PORTA_PCTL_R &= ~0x0000FF00; //  configure PF4 as GPIO F mit A getauscht 0x000F0000 clear the corresponding bits in the PCTL register
	GPIO_PORTA_AMSEL_R &= ~0x08;  //    disable analog functionality on PF4  We clear bits in the AMSEL register to disable analog
	GPIO_PORTA_PUR_R &= ~0x08;     //     enable weak pull-up  pur3 shall be low (&~)set to 0
	GPIO_PORTA_IS_R &= ~0x08;     // (d) PA3 is edge-sensitive
	GPIO_PORTA_IBE_R &= ~0x08;    //     PF4 is not both edges
  GPIO_PORTA_IEV_R &= ~0x08;    //     PA3 rising edge event 
	GPIO_PORTA_ICR_R = 0x08;    //    set flag to 0
	GPIO_PORTA_IM_R |= 0x08;      // (f) arm interrupt on P  
  NVIC_PRI0_R = (NVIC_PRI0_R&0xFFFFFF00)|0x00000020; // (g) priority 2 00400000 0x41C 4000 00E0
	NVIC_EN0_R = 0x00000001;      // (h) enable interrupt 0 in NVIC (bit 0)
	//EnableInterrupts();           // (i) Enable global Interrupt flag (I)

}

void GPIO_PortA_Handler(void){ //do something with PA3 input 0x1000 0x08
	GPIO_PORTA_ICR_R=0x08; //acknowledge flag 
	edgecounter = edgecounter+1;
	
	
	if(GPIO_PORTA_DATA_R&0x08){  // SW2 touch GPIO_PORTA_RIS_R
	
	edgecounter = edgecounter+1;
	SW1=1;
	//if(GPIO_PORTA_RIS_R&0xFF){
	beep=!beep;
	
	}	
	edgecounter = edgecounter+1;
}

	
	
	


void SysTick_Init(unsigned long period){ // priority 2
  NVIC_ST_CTRL_R = 0;         // disable SysTick during setup
  NVIC_ST_RELOAD_R = period-1;// reload value
  NVIC_ST_CURRENT_R = 0;      // any write to current clears it
  NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0xFF00FFFF)|0x00020000;      //00FFFFFF   //4000000000 20.07. added 2 for lower prio 
  NVIC_ST_CTRL_R = 0x07; // enable SysTick with core clock and interrupts
  // finish all initializations and then enable interrupts
}


/* unlock if port F for debug
void PortF_init(void){
SYSCTL_RCGC2_R |= 0x00000020; // (a) activate clock for port F
  Hurtz = 0;             // (b) initialize count and wait for clock
  GPIO_PORTF_DIR_R &= ~0x10;    // (c) make PF4 in (built-in button)
  GPIO_PORTF_AFSEL_R &= ~0x10;  //     disable alt funct on PF4
  GPIO_PORTF_DEN_R |= 0x10;     //     enable digital I/O on PF4
  GPIO_PORTF_PCTL_R &= ~0x000F0000; //  configure PF4 as GPIO
  GPIO_PORTF_AMSEL_R &= ~0x10;  //    disable analog functionality on PF4
  GPIO_PORTF_PUR_R |= 0x10;     //     enable weak pull-up on PF4
  GPIO_PORTF_IS_R &= ~0x10;     // (d) PF4 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x10;    //     PF4 is not both edges
  GPIO_PORTF_IEV_R &= ~0x10;    //     PF4 falling edge event
  GPIO_PORTF_ICR_R = 0x10;      // (e) clear flag4
  GPIO_PORTF_IM_R |= 0x10;      // (f) arm interrupt on PF4
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000; // (g) priority 5
  NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC
  EnableInterrupts();           // (i) Enable global Interrupt flag (I)
	
	
}

void GPIO_PortF_Handler(void){
	GPIO_PORTF_ICR_R = 0x10;      // acknowledge flag4
  Hurtz = Hurtz + 1;
}
*/ //port F outcommented



// called at 880 Hz
void SysTick_Handler(void){
	
	
	SW1=GPIO_PORTA_DATA_R&0x08;
	if (SW1==1){
		beep=1;
	}
	
	//beep=1;
	while(beep){
	GPIO_PORTA_DATA_R ^= 0x04;       // toggle PA2
		//beep=!beep;
	Hurtz = Hurtz + 1;
	} 
	
	GPIO_PORTA_DATA_R &= ~0x04;       // mute PA2
		
  }

int main(void){// activate grader and set system clock to 80 MHz
  DisableInterrupts();
	TExaS_Init(SW_PIN_PA3, HEADPHONE_PIN_PA2,ScopeOn); 
  Sound_Init();         
	SysTick_Init(90909);        // initialize SysTick timer, every 1ms - changed from 16000; 80000 is too small
	//PortF_init();
	EnableInterrupts();   // enable after all initialization are done
	beep=0;



  while(1){
    // main program is free to perform other tasks
    // do not use WaitForInterrupt() here, it may cause the TExaS to crash
//WaitForInterrupt();
//	SW1=GPIO_PORTA_DATA_R&0x08; //write switch 1 to SW1	
						
}
}
