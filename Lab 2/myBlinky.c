// Device header
#include "MKL25Z4.h"                    

#define RED_LED 18 
// PortB Pin 18 
#define GREEN_LED 19 
// PortB Pin 19 
#define BLUE_LED 1 
// PortD Pin 1 
#define MASK(x) (1 << (x)) 
#define del 0xF0000


long long counter = 0;

void InitGPIO(void) {  
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

void delay(long long mil) {
	counter = 0;
	for (long long i = mil; i > 0; i--) {
		counter++;
	}
}

enum color {
	RED,BLUE,GREEN
};

void led_control(enum color t) {
	switch(t) {
		case RED:
			PTB->PDOR &= ~MASK(RED_LED);
			delay(del);
			break;
		case GREEN:
			PTB->PDOR &= ~MASK(GREEN_LED);
			delay(del);
			break;
		case BLUE:
			PTD->PDOR &= ~MASK(BLUE_LED);
			delay(del);
			break;
	}
	PTB->PDOR |= MASK(RED_LED);
	PTD->PDOR |= MASK(BLUE_LED);
	PTB->PDOR |= MASK(GREEN_LED);
}

int main(void) {
	SystemCoreClockUpdate();
	
	InitGPIO();
	PTB->PDOR |= MASK(RED_LED);
	PTD->PDOR |= MASK(BLUE_LED);
	PTB->PDOR |= MASK(GREEN_LED);
	while(1) {
			led_control(RED);
			led_control(GREEN);
			led_control(BLUE);
		}
}