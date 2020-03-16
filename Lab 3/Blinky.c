
#include "MKL25Z4.h"                    // Device header

#define RED_LED 18 
// PortB Pin 18 
#define GREEN_LED 19 
// PortB Pin 19 
#define BLUE_LED 1 
// PortD Pin 1
#define SW_POS 6
#define DEBUG_PORT 4
#define DBG_ISR_POS 2 
#define MASK(x) (1 << (x)) 
#define del 0

int counter = 0;
volatile int status = 0;

void initGPIO(void) {  
  // Enable Clock to PORTB and PORTD  
	SIM->SCGC5 |= ((SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTD_MASK));    
	
	// Configure MUX settings to make all 3 pins GPIO    
	
	PORTB->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK;  
	PORTB->PCR[RED_LED] |= PORT_PCR_MUX(1);    
	
	PORTB->PCR[GREEN_LED] &= ~PORT_PCR_MUX_MASK;  
	PORTB->PCR[GREEN_LED] |= PORT_PCR_MUX(1);   
	
	PORTD->PCR[BLUE_LED] &= ~PORT_PCR_MUX_MASK;  
	PORTD->PCR[BLUE_LED] |= PORT_PCR_MUX(1);    
	
	// Set Data Direction Registers for PortB and PortD  
	// Set LED to output
	PTB->PDDR |= (MASK(RED_LED) | MASK(GREEN_LED));  
	PTD->PDDR |= MASK(BLUE_LED);   
} 

void initSwitch(void) {
	/* enable clock for port D */ 
	SIM->SCGC5 |=  SIM_SCGC5_PORTD_MASK; 
	/* Select GPIO and enable pull-up resistors and interrupts on falling edges for pin connected to switch */ 
	PORTD->PCR[SW_POS] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_IRQC(0x0a); 
	/* Set port D switch bit to inputs */ 
	PTD->PDDR &= ~MASK(SW_POS); 
	/* Enable Interrupts */ 
	NVIC_SetPriority(PORTD_IRQn, 128); 
	NVIC_ClearPendingIRQ(PORTD_IRQn); 
	NVIC_EnableIRQ(PORTD_IRQn);
}

void delay(long long mil) {
	int count = 0xF000;
	for (long long i = mil; i > 0; i--) {
		count++;
	}
}

void PORTD_IRQHandler() {  
	NVIC_ClearPendingIRQ(PORTD_IRQn);

	if ((PORTD->ISFR & MASK(SW_POS))) {
		status = 1;
		delay(del);
	}
	
	PORTD->ISFR = 0xffffffff;
} 

void control_RGB_LEDs(int counter) {
	PTB->PDOR |= MASK(RED_LED);
	PTD->PDOR |= MASK(BLUE_LED);
	PTB->PDOR |= MASK(GREEN_LED);	
	switch(counter%3) {
		case 0:
			PTB->PDOR &= ~MASK(RED_LED);
			break;
		case 1:
			PTB->PDOR &= ~MASK(GREEN_LED);
			break;
		case 2:
			PTD->PDOR &= ~MASK(BLUE_LED);
			break;
	}
}

int main(void) {
	SystemCoreClockUpdate();
	 
	initGPIO();
	initSwitch();
	__enable_irq();
	
	while (1) { 
    if (status) {
			control_RGB_LEDs(counter);
				counter++; status = 0; 
		}
	}
}