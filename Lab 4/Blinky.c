#include<MKL25Z4.h>
#define PTB0_Pin 0
#define PTB1_Pin 1
#define TIMER_CLCK_FREQ 375000 

/* initPWM() */
void initPWM(void) {
 SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
 
 PORTB->PCR[PTB0_Pin] &= ~PORT_PCR_MUX_MASK;
 PORTB->PCR[PTB0_Pin] |= PORT_PCR_MUX(3);
 
 PORTB->PCR[PTB1_Pin] &= ~PORT_PCR_MUX_MASK;
 PORTB->PCR[PTB1_Pin] |= PORT_PCR_MUX(3);
 
 SIM_SCGC6 |= SIM_SCGC6_TPM1_MASK;
 
 SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
 SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
 
 // TPM1->MOD = 7499; //defines the time period
 // TPM1_C0V = 3750; //pulse width to control the duty cycle
	
 TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
 TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
 TPM1->SC &= ~(TPM_SC_CPWMS_MASK);
 
 TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSB_MASK));
 TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
}

void setFrequency(int frequency){
	int mod = (TIMER_CLCK_FREQ / frequency) - 1;
	TPM1->MOD = mod;
	TPM1_C0V = (mod+1)/2;
}

void setDelay(){
			for(int i=0;i<5000000;i++);
}
	
int main(void){
		SystemCoreClockUpdate();
	  initPWM();
		
		while(1){
			setFrequency(1300);
			setDelay();
			setFrequency(1600);
			setDelay();
			setFrequency(1900);
			setDelay();
			setFrequency(2100);
			setDelay();
		}
}
