// EdgeInterrupt.c
// Runs on LM4F120 or TM4C123
// Request an interrupt on the falling edge of PF4 (when the user
// button is pressed) and increment a counter in the interrupt.  Note
// that button bouncing is not addressed.
// Daniel Valvano
// September 14, 2013

/* This example accompanies the book
   "Embedded Systems: Introduction to ARM Cortex M Microcontrollers"
   ISBN: 978-1469998749, Jonathan Valvano, copyright (c) 2013
   Volume 1, Program 9.4
   
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2013
   Volume 2, Program 5.6, Section 5.5

 Copyright 2013 by Jonathan W. Valvano, valvano@mail.utexas.edu
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

// user button connected to PF4 (increment counter on falling edge)

#include "stdio.h"
#include "tm4c123gh6pm.h"
/*
#define NVIC_EN0_R              (*((volatile unsigned long *)0xE000E100))  // IRQ 0 to 31 Set Enable Register
#define NVIC_PRI7_R             (*((volatile unsigned long *)0xE000E41C))  // IRQ 28 to 31 Priority Register
#define GPIO_PORTF_DIR_R        (*((volatile unsigned long *)0x40025400))
#define GPIO_PORTF_IS_R         (*((volatile unsigned long *)0x40025404))
#define GPIO_PORTF_IBE_R        (*((volatile unsigned long *)0x40025408))
#define GPIO_PORTF_IEV_R        (*((volatile unsigned long *)0x4002540C))
#define GPIO_PORTF_IM_R         (*((volatile unsigned long *)0x40025410))
#define GPIO_PORTF_LOCK_R       (*((volatile unsigned long *)0x40025520))
#define GPIO_PORTF_CR_R         (*((volatile unsigned long *)0x40025524))
#define GPIO_PORTF_RIS_R        (*((volatile unsigned long *)0x40025414))
#define GPIO_PORTF_ICR_R        (*((volatile unsigned long *)0x4002541C))
#define GPIO_PORTF_AFSEL_R      (*((volatile unsigned long *)0x40025420))
#define GPIO_PORTF_PUR_R        (*((volatile unsigned long *)0x40025510))
#define GPIO_PORTF_PDR_R        (*((volatile unsigned long *)0x40025514))
#define GPIO_PORTF_DEN_R        (*((volatile unsigned long *)0x4002551C))
#define GPIO_PORTF_AMSEL_R      (*((volatile unsigned long *)0x40025528))
#define GPIO_PORTF_PCTL_R       (*((volatile unsigned long *)0x4002552C))
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
#define GPIO_PORTF_DATA_R       (*((volatile unsigned long *)0x400253FC))
	

#define GPIO_PORTB_DATA_R       (*((volatile unsigned long *)0x400053FC))
#define GPIO_PORTB_DIR_R        (*((volatile unsigned long *)0x40005400))
#define GPIO_PORTB_AFSEL_R      (*((volatile unsigned long *)0x40005420))
#define GPIO_PORTB_PUR_R        (*((volatile unsigned long *)0x40005510))
#define GPIO_PORTB_DEN_R        (*((volatile unsigned long *)0x4000551C))
#define GPIO_PORTB_LOCK_R       (*((volatile unsigned long *)0x40005520))
#define GPIO_PORTB_CR_R         (*((volatile unsigned long *)0x40005524))
#define GPIO_PORTB_AMSEL_R      (*((volatile unsigned long *)0x40005528))
#define GPIO_PORTB_PCTL_R       (*((volatile unsigned long *)0x4000552C))
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
	

#define SYSCTL_RCGC2_GPIOF      0x00000020  // port F Clock Gating Control
*/
#define MAX_COUNT					16

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
unsigned int CountNumberSetBits(unsigned long n);

unsigned int COUNT_MOD = MAX_COUNT - 1;

// global variable visible in Watch window of debugger
// increments at least once per button press
volatile unsigned long CoveredCount = 0;
unsigned int count;


unsigned int CountNumberSetBits(unsigned long n)
{
	unsigned int count = 0; 
        while (n) { 
            n &= (n - 1); 
            count++; 
        } 
        return count; 
}

void InitStatusLightsAndResetSwitch(void){  
volatile unsigned long delay;
	SYSCTL_RCGC2_R |= 0x00000020;			// B clock
	delay = SYSCTL_RCGC2_R;				// dela	
	//GPIO_PORTF_LOCK_R = 0x4C4F434B;   // unlock PortF PF0  
	//GPIO_PORTF_CR_R |= 0x01;				// allow changes to PF0   
	GPIO_PORTF_DIR_R &= ~0x10;				//PF0 is an input 
	GPIO_PORTF_AFSEL_R &= ~0x10;  //     disable alt funct on PF0
  GPIO_PORTF_DEN_R |= 0x10;				// enable digital pin PF0  
	//GPIO_PORTF_PCTL_R &= ~0x00000000;
	GPIO_PORTF_AMSEL_R &= ~0x10;			// disable analog function 
	GPIO_PORTF_PUR_R |= 0x10;         // enable pullup resistors on PF0 
	GPIO_PORTF_IS_R &= ~0x10;     // (d) PF0 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x10;    //     PF0 is not both edges
  GPIO_PORTF_IEV_R &= ~0x10;    //     PF0 falling edge event
  GPIO_PORTF_ICR_R = 0x10;      // (e) clear flag0
  GPIO_PORTF_IM_R |= 0x10;      // (f) arm interrupt on PF0
	
	GPIO_PORTF_DIR_R |= 0x0A; //PF1 and PF3 are outputs 1010 == A
	GPIO_PORTF_DEN_R |= 0xA;
	
	
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000; // (g) priority 5
  NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC
}




void Delay(unsigned int time)
{
	int c, d;

	for (c = 1; c <= 8192; c++)
		for (d = 1; d <= time; d++) {}

}


/*
This function ensures that I count twice ONLY if the user double taps the switch rapidly, otherwise switch bounce causes lots of wrong values for the count.
*/
void AccountForBounce(void)
{
	GPIO_PORTF_LOCK_R = 0x4C4F434B;   // unlock PortF PF0  
	GPIO_PORTF_CR_R |= 0x01;				// allow changes to PF0 
	GPIO_PORTF_IM_R &= !0x11;      // (f) arm interrupt on PF4
	Delay(100);
	GPIO_PORTF_IM_R |= 0x11;      // (f) arm interrupt on PF4
}

void InitCountDisplay(void) {
	volatile unsigned long delay;
	SYSCTL_RCGC2_R |= 0x00000002;			// B clock
	delay = SYSCTL_RCGC2_R;				// delay   
	//GPIO_PORTB_CR_R |= 0x0F;				// allow changes to PB0-3       
	GPIO_PORTB_AMSEL_R &= ~0x0F;			// disable analog function
	//GPIO_PORTB_PCTL_R &= ~0x0000000F;		// GPIO clear bit PCTL  
	GPIO_PORTB_DIR_R |= 0x0F;			    // PB0-2 as outputs 
	GPIO_PORTB_AFSEL_R &= ~0x0F;			// no alternate function
	GPIO_PORTB_DEN_R |= 0x0F;				// enable digital pins PB0-3    	
}

void InitSensorArray(void)
{
	volatile unsigned long delay;
	SYSCTL_RCGC2_R |= 0x00000001;			// A clock
	delay = SYSCTL_RCGC2_R;				// delay   
	//GPIO_PORTA_CR_R |= 0xFC;				// allow changes to PA2-7       
	GPIO_PORTA_AMSEL_R &= ~0xFC;			// disable analog function
	GPIO_PORTA_PCTL_R &= ~0x000000FC;		// GPIO clear bit PCTL  
	GPIO_PORTA_DIR_R &= ~0xFC;			    // PA2-7 as inputs 
	GPIO_PORTA_AFSEL_R &= ~0xFC;			// no alternate function
	GPIO_PORTA_DEN_R |= 0xFC;				// enable digital pins PA2-7   


	GPIO_PORTA_IS_R &= ~0x80;     // (d) PF0 is edge-sensitive
  //GPIO_PORTA_IBE_R |= 0x80;    //     PF0 is  both edges
  GPIO_PORTA_IEV_R |= 0x80;    //     PF0 falling edge event
	GPIO_PORTA_IM_R |= 0x80;
  GPIO_PORTA_ICR_R = 0x80;      // (e) clear flag07
	 	
	NVIC_EN0_R |= 0x01; //Enable GPIOPortA_Handler
	
}

void GPIOPortF_Handler(void){
	AccountForBounce();
	if ((GPIO_PORTF_RIS_R & 0x10) != 0)
	{
		
		GPIO_PORTF_ICR_R |= 0x10;      
		CoveredCount = 0;
		GPIO_PORTB_DATA_R = 0;
		
	} 
	else if((GPIO_PORTF_RIS_R & 0x01) != 0)
	{
		GPIO_PORTF_ICR_R |= 0x01;      // acknowledge flag0
		CoveredCount = 0;
		GPIO_PORTB_DATA_R = 0;
		
	} 
}

void GPIOPortA_Handler(void)
{
	  GPIO_PORTA_ICR_R |= 0x80;      // acknowledge flag7
		count = CountNumberSetBits(~GPIO_PORTA_DATA_R & 0x7C);
		CoveredCount = (CoveredCount + count) & COUNT_MOD;
}



//debug code
int main(void){
  InitStatusLightsAndResetSwitch();           // initialize GPIO Port F interrupt
	InitCountDisplay();
	InitSensorArray();
	EnableInterrupts();           // (i) Enable global Interrupt flag (I)
  while(1){
    //WaitForInterrupt();
		if ((GPIO_PORTA_DATA_R & 0xFC) == 0xFC)
		{
				GPIO_PORTF_DATA_R &= ~0x02;
				GPIO_PORTF_DATA_R |= 0x08;
		}	else if ((GPIO_PORTA_DATA_R & 0xFC) < 4)
		{
			GPIO_PORTF_DATA_R &= ~0x08;
			GPIO_PORTF_DATA_R |= 0x02;
		} else 
		{
			GPIO_PORTF_DATA_R &= ~0x0A;
		}
		
		GPIO_PORTB_DATA_R = CoveredCount;
		
  }
}
