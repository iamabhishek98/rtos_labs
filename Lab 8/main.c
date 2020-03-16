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
#define Q_SIZE (32)

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

osSemaphoreId_t mySem;
osSemaphoreId_t mySem2;
volatile int count = 0;
volatile int check = 0;
uint8_t data = 0x01;
typedef struct{
unsigned char Data[Q_SIZE];
unsigned int Head; // points to oldest data element
unsigned int Tail; // points to next free space
unsigned int Size; // quantity of elements in queue
} Q_T;
 
Q_T TxQ, RxQ;

void Q_Init(Q_T * q) {
unsigned int i;
for (i=0; i<Q_SIZE; i++)
q->Data[i] = 0; // to simplify our lives when debugging
q->Head = 0;
q->Tail = 0;
q->Size = 0;
}

int Q_Empty(Q_T * q) {
return q->Size == 0;
}
int Q_Full(Q_T * q) {
return q->Size == Q_SIZE;
}
 
int Q_Enqueue(Q_T * q, unsigned char d) {
    // What if queue is full?
    if (!Q_Full(q)) {
        q->Data[q->Tail++] = d;
        q->Tail %= Q_SIZE;
        q->Size++;
        return 1; // success
    } else
        return 0; // failure
}
unsigned char Q_Dequeue(Q_T * q) {
    // Must check to see if queue is empty before dequeueing
    unsigned char t=0;
    if (!Q_Empty(q)) {
        t = q->Data[q->Head];
        q->Data[q->Head++] = 0; // to simplify debugging
        q->Head %= Q_SIZE;
        q->Size--;
    }
    return t;
}
 
int mod_value(int frequency){
    return DEFAULT_SYSTEM_CLOCK / (frequency * 128)  - 1;
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

void initUART2(uint32_t baud_rate) {
    uint32_t divisor, bus_clock;
 
    SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
 
    PORTE->PCR[UART_TX_PORTE22] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[UART_TX_PORTE22] |= PORT_PCR_MUX(4);
 
    PORTE->PCR[UART_TX_PORTE23] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[UART_TX_PORTE23] |= PORT_PCR_MUX(4);
 
    UART2->C2 &= ~((UART_C2_TE_MASK) | (UART_C2_RE_MASK));
 
    bus_clock = (DEFAULT_SYSTEM_CLOCK)/2;
    divisor = bus_clock / (baud_rate*16);
    UART2->BDH = UART_BDH_SBR(divisor >> 8);
    UART2->BDL = UART_BDL_SBR(divisor);
 
    NVIC_SetPriority(UART2_IRQn, 128);
    NVIC_ClearPendingIRQ(UART2_IRQn);
    NVIC_EnableIRQ(UART2_IRQn);
 
    UART2->C1 = 0;
    UART2->S2 = 0;
    UART2->C3 = 0;
 
    //UART2->C2 |= ((UART_C2_TE_MASK) | (UART_C2_RE_MASK));
    UART2->C2 |= ((UART_C2_TE_MASK) | (UART_C2_RE_MASK) | UART_C2_RIE_MASK);
    Q_Init(&RxQ);
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

void UART2_IRQHandler(void) {
    if (UART2->S1 & UART_S1_RDRF_MASK) {
        // received a character
				if (UART2->D == 0x02) {
						osSemaphoreRelease(mySem);
				} else if (UART2->D == 0x03) {
						osSemaphoreRelease(mySem2);
				} 					
				/*if (!Q_Full(&RxQ)) {
        Q_Enqueue(&RxQ, UART2->D);
        } else {
        // error -queue full.
        }*/
    }
}

void PORTD_IRQHandler() {  
	NVIC_ClearPendingIRQ(PORTD_IRQn);

	if ((PORTD->ISFR & MASK(SW_POS))) {
		count++;
		//delay(0x80000);
		//osSemaphoreRelease(mySem);
	}
	
	PORTD->ISFR = 0xffffffff;
} 
/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
void led_red_thread (void *argument) {
 
  // ...
  for (;;) {
		osSemaphoreAcquire(mySem, osWaitForever);
		
		ledcontrol(red_led, led_on);
		//delay(0x80000);
		osDelay(1000);
		//ledcontrol(red_led, led_off);
		//delay(0x80000);
		osDelay(1000);
		
		//osSemaphoreRelease(mySem);
	}
}

void led_green_thread (void *argument) {
 
  // ...
  for (;;) {
		osSemaphoreAcquire(mySem2, osWaitForever);
		
		ledcontrol(green_led, led_on);
		//delay(0x80000);
		osDelay(1000);
		//ledcontrol(green_led, led_off);
		//delay(0x80000);
		osDelay(1000);
		//osSemaphoreRelease(mySem);
	}
}
 
int main (void) {

  // System Initialization
  SystemCoreClockUpdate();
	initSwitch();
	initGPIO();
	initUART2(BAUD_RATE);
	offRGB();
  // ...
	data = 0x01;
	
  osKernelInitialize();                 // Initialize CMSIS-RTOS
  mySem = osSemaphoreNew(1,1,NULL);
	mySem2 = osSemaphoreNew(1,1,NULL);
	osThreadNew(led_red_thread, NULL, NULL);    // Create application main thread
	osThreadNew(led_green_thread, NULL, NULL);    // Create application main thread
  osKernelStart();                      // Start thread execution
  for (;;) {}
}