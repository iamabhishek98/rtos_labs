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
#define MSG_COUNT 1

osThreadId_t redLED_Id, greenLED_Id, blueLED_Id, control_Id;
osEventFlagsId_t redLED_flag, greenLED_flag, blueLED_flag;
osMessageQueueId_t redMsg, greenMsg, blueMsg;

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

typedef struct {
	uint8_t cmd;
	uint8_t data;
} myDataPkt;

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
	if (state && colour == blue_led) 
		PTD->PDOR &= ~MASK(colour);
	else if (state)
		PTB->PDOR &= ~MASK(colour);
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
	
  myDataPkt myRxData;
  
	// ...
	for (;;) {
		osMessageQueueGet(redMsg, &myRxData, NULL, osWaitForever);
		//osEventFlagsWait(blueLED_flag, 0x00000001, osFlagsWaitAny, osWaitForever);
		//osThreadFlagsWait(blueLED_flag, 0x00000001, osFlagsWaitAny, osWaitForever);
		if (myRxData.cmd == 0x01 && myRxData.data == 0x01) {
			ledcontrol(red_led, led_on);
			//delay(0x80000);
			osDelay(1000);
			ledcontrol(red_led, led_off);
			//delay(0x80000);
			osDelay(500);
		}	
	}
}
	

void led_green_thread (void *argument) {
	
	myDataPkt myRxData;
	
	// ...
  for (;;) {
		osMessageQueueGet(greenMsg, &myRxData, NULL, osWaitForever);
		//osEventFlagsWait(blueLED_flag, 0x00000001, osFlagsWaitAny, osWaitForever);
		//osThreadFlagsWait(blueLED_flag, 0x00000001, osFlagsWaitAny, osWaitForever);
		if (myRxData.cmd == 0x01 && myRxData.data == 0x02) {		
			ledcontrol(green_led, led_on);
			//delay(0x80000);
			osDelay(500);
			ledcontrol(green_led, led_off);
			//delay(0x80000);
			osDelay(250);
		}
	}
}

void led_blue_thread (void *argument) {
 
  myDataPkt myRxData;
  
	// ...
	for (;;) {
		osMessageQueueGet(blueMsg, &myRxData, NULL, osWaitForever);
		//osEventFlagsWait(blueLED_flag, 0x00000001, osFlagsWaitAny, osWaitForever);
		//osThreadFlagsWait(blueLED_flag, 0x00000001, osFlagsWaitAny, osWaitForever);
		if (myRxData.cmd == 0x01 && myRxData.data == 0x03) {
			ledcontrol(blue_led, led_on);
			//delay(0x80000);
			osDelay(250);
			ledcontrol(blue_led, led_off);
			//delay(0x80000);
			osDelay(1000);
		}
	}
}
 
void control_thread (void *argument) {
	myDataPkt redData, greenData, blueData;
	
	redData.cmd = 0x01;
	redData.data = 0x01;
	
	greenData.cmd = 0x01;
	greenData.data = 0x02;
	
	blueData.cmd = 0x01;
	blueData.data = 0x03;
	
	// ...
	for (;;) {
		osMessageQueuePut(redMsg, &redData, NULL, 0);
		//osEventFlagsSet(redLED_flag, 0x0000001);
		//osThreadFlagsSet(redLED_Id, 0x0000001);
		osDelay(1000);
		osMessageQueuePut(greenMsg, &greenData, NULL, 0);
		//osEventFlagsSet(greenLED_flag, 0x0000001);
		//osThreadFlagsSet(greenLED_Id, 0x0000001);
		osDelay(500);
		osMessageQueuePut(blueMsg, &blueData, NULL, 0);
		//osEventFlagsSet(blueLED_flag, 0x0000001);
		//osThreadFlagsSet(blueLED_Id, 0x0000001);
		osDelay(250);
	}	
}

int main (void) {
 
  // System Initialization
		SystemCoreClockUpdate();
	initGPIO();
	offRGB();
  // ...

	
  osKernelInitialize();                 // Initialize CMSIS-RTOS
  /*
	redLED_flag = osEventFlagsNew(NULL);
	greenLED_flag = osEventFlagsNew(NULL);
	blueLED_flag = osEventFlagsNew(NULL);
	*/
	
	redMsg = osMessageQueueNew(MSG_COUNT, sizeof(myDataPkt), NULL);
	greenMsg = osMessageQueueNew(MSG_COUNT, sizeof(myDataPkt), NULL);
	blueMsg = osMessageQueueNew(MSG_COUNT, sizeof(myDataPkt), NULL);
	
	redLED_Id = osThreadNew(led_red_thread, NULL, NULL);    // Create application main thread
	greenLED_Id = osThreadNew(led_green_thread, NULL, NULL); 
  blueLED_Id = osThreadNew(led_blue_thread, NULL, NULL);   
  control_Id = osThreadNew(control_thread, NULL, NULL);    
	osKernelStart();                      // Start thread execution
  for (;;) {}
}
