#include "MKL25Z4.h"
#define PTB0_Pin 0
#define PTB1_Pin 1
 
#define RED_LED 18 // PortB Pin 18
#define MASK(x) (1 << (x))
 
#define FREQ 50
#define BAUD_RATE 9600
#define UART_TX_PORTE22 22
#define UART_RX_PORTE23 23
#define UART_INT_PRIO 128
#define Q_SIZE (32)
 
volatile int check = 0;
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
 
static void delay(volatile uint32_t nof) {
    while(nof!=0) {
        __asm("NOP");
        nof--;
    }
}  
 
 
void UART2_IRQHandler(void) {
    check = 1;
    if (UART2->S1 & UART_S1_RDRF_MASK) {
        // received a character
        if (!Q_Full(&RxQ)) {
        Q_Enqueue(&RxQ, UART2->D);
        } else {
        // error -queue full.
        }
    }
}
void initLED() {
    // Enable Clock to PORTB and PORTD
    SIM->SCGC5 |= ((SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTD_MASK));
    // Configure MUX settings to make all 3 pins GPIO
    PORTB->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK;
    PORTB->PCR[RED_LED] |= PORT_PCR_MUX(1);
 
    PTB->PDDR |= MASK(RED_LED);
    //PTB->PDOR |= MASK(RED_LED);
}
 
void initUART2(uint32_t baud_rate) {
    uint32_t divisor, bus_clock;
 
    SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
 
    PORTE->PCR[UART_TX_PORTE22] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[UART_TX_PORTE22] |= PORT_PCR_MUX(4);
 
    PORTE->PCR[UART_RX_PORTE23] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[UART_RX_PORTE23] |= PORT_PCR_MUX(4);
 
 
 
 
 
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
int main(){
    //initPWM();
    uint8_t data = 0x01;
    initLED();
    initUART2(BAUD_RATE);
    while(1) {
    while (!Q_Empty(&RxQ)) {
        data = Q_Dequeue(&RxQ);
        if(data == 0x03) {
            PTB->PDOR = !MASK(RED_LED);
        }
        if(data == 0x02) {
            PTB->PDOR = MASK(RED_LED);
        }
    }
    }
}