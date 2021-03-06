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
int beep=0; //variable for beep
int SW1=0; //switch 1 pressed
int old=0; //old switch status
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
	//GPIO_PORTA_ICR_R = 0x08;    //    set flag to 0 -DEACTIVATE real board grading
	//GPIO_PORTA_IM_R |= 0x08;      // (f) arm interrupt on P  DEACTIVATE real board grading
  NVIC_PRI0_R = (NVIC_PRI0_R&0xFFFFFF00)|0x00000020; // (g) priority 2 00400000 0x41C 4000 00E0
	NVIC_EN0_R = 0x00000001;      // (h) enable interrupt 0 in NVIC (bit 0)
	//EnableInterrupts();           // (i) Enable global Interrupt flag (I)

}
void SysTick_Init(unsigned long period){ // priority 2
  NVIC_ST_CTRL_R = 0;         // disable SysTick during setup
  NVIC_ST_RELOAD_R = period-1;// reload value
  NVIC_ST_CURRENT_R = 0;      // any write to current clears it
  NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0xFF00FFFF)|0x00020000;      //00FFFFFF   //4000000000 20.07. added 2 for lower prio 
  NVIC_ST_CTRL_R = 0x07; // enable SysTick with core clock and interrupts
  // finish all initializations and then enable interrupts
}

// called at 880 Hz
void SysTick_Handler(void){
	

SW1=GPIO_PORTA_DATA_R & 0x08; //switch status read
	
//prev_val=GPIO_PORTADATA_R&0x08; //storing the value of input received in prev_val
if(SW1 && !old){
beep = !beep;
	//old=SW1;
	Hurtz = Hurtz + 1;
}
//else{
	//beep=0;
//}

if(beep){
GPIO_PORTA_DATA_R ^= 0x04; //toggle PA2
}
else{
GPIO_PORTA_DATA_R &= ~0x04;       // mute PA2
}
old=SW1;
  }

int main(void){// activate grader and set system clock to 80 MHz
  DisableInterrupts();
	TExaS_Init(SW_PIN_PA3, HEADPHONE_PIN_PA2,ScopeOn); 
  Sound_Init();         
	SysTick_Init(90909);        // initialize SysTick timer, every 1ms - changed from 16000; 80000 is too small
	//PortF_init();
	EnableInterrupts();   // enable after all initialization are done
	beep=0;
	old=0;



  while(1){
    // main program is free to perform other tasks
    // do not use WaitForInterrupt() here, it may cause the TExaS to crash
						
}
}
