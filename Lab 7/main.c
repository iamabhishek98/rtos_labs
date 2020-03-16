/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"

#include "MKL25Z4.h"

#define BAUD_RATE 9600
#define UART_TX_PORTE22 22
#define UART_TX_PORTE23 23
#define UART2_INT_PRIO 128

#define LED_RED 2
#define LED_MASK(x) (x & 0x06)
#define BIT0_MASK(x) (x & 0x01)

#define RED_LED 18
#define BLUE_LED 1
#define GREEN_LED 19
#define MASK(x) (1<<(x))

// PortD Pin 1
#define SW_POS 6
#define DEBUG_PORT 4
#define DBG_ISR_POS 2 
#define del 300000

const osThreadAttr_t thread_attr = {
	.priority = osPriorityAboveNormal1
};

osMutexId_t myMutex;

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

typedef enum led_colors {
	red_led = RED_LED,
	green_led = GREEN_LED,
	blue_led = BLUE_LED
} led_colors_t;

typedef enum states {
	led_off, led_on
} my_state_t;

void offRGB(void) {
	PTB->PSOR = (MASK(RED_LED) | MASK(GREEN_LED));
	PTD->PSOR = MASK(BLUE_LED);
}

void ledcontrol(led_colors_t colour, my_state_t state) {
	if (state) 
		PTB->PCOR = MASK(colour);
	else 
		offRGB();
}

static void delay(volatile uint32_t nof) {
 while(nof!=0) {
  __asm("NOP");
  nof--;
 }
}

/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
void led_red_thread (void *argument) {
 
  // ...
  for (;;) {
		osMutexAcquire(myMutex, osWaitForever);
		
		ledcontrol(red_led, led_on);
		//delay(0x80000);
		osDelay(1000);
		ledcontrol(red_led, led_off);
		//delay(0x80000);
		osDelay(1000);
		
		osMutexRelease(myMutex);
	}
}

void led_green_thread (void *argument) {
 
  // ...
  for (;;) {
		osMutexAcquire(myMutex, osWaitForever);
		
		ledcontrol(green_led, led_on);
		//delay(0x80000);
		osDelay(1000);
		ledcontrol(green_led, led_off);
		//delay(0x80000);
		osDelay(1000);
		//osMutexRelease(myMutex);
	}
}
 
int main (void) {
 
  // System Initialization
  SystemCoreClockUpdate();
	initGPIO();
	offRGB();
  // ...
 
  osKernelInitialize();                 // Initialize CMSIS-RTOS
  myMutex = osMutexNew(NULL);
	osThreadNew(led_red_thread, NULL, &thread_attr);    // Create application main thread
	osThreadNew(led_green_thread, NULL, NULL);    // Create application main thread
  osKernelStart();                      // Start thread execution
  for (;;) {}
}
