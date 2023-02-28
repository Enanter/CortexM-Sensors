#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>


// Create definition for PORT B registers
#define GPIO_PORTB_DATA_R       (*((volatile unsigned long *)0x400053FC))
#define GPIO_PORTB_DIR_R        (*((volatile unsigned long *)0x40005400))
#define GPIO_PORTB_PUR_R        (*((volatile unsigned long *)0x40005510))
#define GPIO_PORTB_DEN_R        (*((volatile unsigned long *)0x4000551C))
#define GPIO_PORTB_CR_R         (*((volatile unsigned long *)0x40005524))
#define GPIO_PORTB_AMSEL_R      (*((volatile unsigned long *)0x40005528))
#define GPIO_PORTB_PCTL_R       (*((volatile unsigned long *)0x4000552C))

//Create definitions for Port E registers
#define GPIO_PORTE_DATA_R       (*((volatile unsigned long *)0x400243FC))
#define GPIO_PORTE_DIR_R        (*((volatile unsigned long *)0x40024400))
#define GPIO_PORTE_PUR_R        (*((volatile unsigned long *)0x40024510))
#define GPIO_PORTE_DEN_R        (*((volatile unsigned long *)0x4002451C))
#define GPIO_PORTE_CR_R         (*((volatile unsigned long *)0x40024524))
#define GPIO_PORTE_AMSEL_R      (*((volatile unsigned long *)0x40024528))
#define GPIO_PORTE_AFSEL_R      (*((volatile unsigned long *)0x40024420))
#define GPIO_PORTE_PCTL_R       (*((volatile unsigned long *)0x4002452C))
/// ADC
#define SYSCTL_RCGCADC_R        (*((volatile unsigned long *)0x400FE638))
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
#define GPIO_PORTD_DATA_R       (*((volatile unsigned long *)0x400073FC))
#define GPIO_PORTD_DIR_R        (*((volatile unsigned long *)0x40007400))
#define GPIO_PORTD_AFSEL_R      (*((volatile unsigned long *)0x40007420))
#define GPIO_PORTD_PUR_R        (*((volatile unsigned long *)0x40007510))
#define GPIO_PORTD_DEN_R        (*((volatile unsigned long *)0x4000751C))
#define GPIO_PORTD_LOCK_R       (*((volatile unsigned long *)0x40007520))
#define GPIO_PORTD_CR_R         (*((volatile unsigned long *)0x40007524))
#define GPIO_PORTD_AMSEL_R      (*((volatile unsigned long *)0x40007528))
#define GPIO_PORTD_PCTL_R       (*((volatile unsigned long *)0x4000752C))
#define ADC0_ACTSS_R            (*((volatile unsigned long *)0x40038000))
#define ADC0_EMUX_R             (*((volatile unsigned long *)0x40038014))
#define ADC0_SSMUX3_R           (*((volatile unsigned long *)0x400380A0))
#define ADC0_SSCTL3_R           (*((volatile unsigned long *)0x400380A4))
#define ADC0_PSSI_R             (*((volatile unsigned long *)0x40038028))
#define ADC0_RIS_R              (*((volatile unsigned long *)0x40038004))
#define ADC0_SSFIFO3_R          (*((volatile unsigned long *)0x400380A8))
#define ADC0_ISC_R              (*((volatile unsigned long *)0x4003800C))
#define ADC0_SSPRI_R            (*((volatile unsigned long *)0x40038020))//sample sequencer priotize register
#define ADC0_PC_R               (*((volatile unsigned long *)0x40038FC4))//program counter register
#define ADC0_IM_R               (*((volatile unsigned long *)0x40038008))//interrupt mask
#define TIMER0_CFG_R            (*((volatile unsigned long *)0x40030000))
#define TIMER0_TAMR_R           (*((volatile unsigned long *)0x40030004))
#define TIMER0_CTL_R            (*((volatile unsigned long *)0x4003000C))
#define TIMER0_RIS_R            (*((volatile unsigned long *)0x4003001C))
#define TIMER0_ICR_R            (*((volatile unsigned long *)0x40030024))
#define TIMER0_TAILR_R          (*((volatile unsigned long *)0x40030028))
#define TIMER0_TAPR_R           (*((volatile unsigned long *)0x40030038))	
#define TIMER0_IMR_R            (*((volatile unsigned long *)0x40030018))					
#define TIMER0_MIS_R            (*((volatile unsigned long *)0x40030020))
#define TIMER0_TAPV_R           (*((volatile unsigned long *)0x40030064))
	
#define TIMER1_CFG_R            (*((volatile unsigned long *)0x40031000))
#define TIMER1_TAMR_R           (*((volatile unsigned long *)0x40031004))
#define TIMER1_CTL_R            (*((volatile unsigned long *)0x4003100C))
#define TIMER1_IMR_R            (*((volatile unsigned long *)0x40031018))
#define TIMER1_RIS_R            (*((volatile unsigned long *)0x4003101C))
#define TIMER1_MIS_R            (*((volatile unsigned long *)0x40031020))
#define TIMER1_ICR_R            (*((volatile unsigned long *)0x40031024))
#define TIMER1_TAILR_R          (*((volatile unsigned long *)0x40031028))
#define TIMER1_TAPR_R           (*((volatile unsigned long *)0x40031038))
	
#define TIMER2_CFG_R            (*((volatile unsigned long *)0x40032000))
#define TIMER2_TAMR_R           (*((volatile unsigned long *)0x40032004))
#define TIMER2_CTL_R            (*((volatile unsigned long *)0x4003200C))
#define TIMER2_IMR_R            (*((volatile unsigned long *)0x40032018))
#define TIMER2_RIS_R            (*((volatile unsigned long *)0x4003201C))
#define TIMER2_MIS_R            (*((volatile unsigned long *)0x40032020))
#define TIMER2_ICR_R            (*((volatile unsigned long *)0x40032024))
#define TIMER2_TAILR_R          (*((volatile unsigned long *)0x40032028))
#define TIMER2_TAPR_R           (*((volatile unsigned long *)0x40032038))
#define TIMER2_TAR_R 						(*((volatile unsigned long *)0x40032048))

#define TIMER3_CFG_R            (*((volatile unsigned long *)0x40033000))
#define TIMER3_TAMR_R           (*((volatile unsigned long *)0x40033004))
#define TIMER3_CTL_R            (*((volatile unsigned long *)0x4003300C))
#define TIMER3_IMR_R            (*((volatile unsigned long *)0x40033018))
#define TIMER3_RIS_R            (*((volatile unsigned long *)0x4003301C))
#define TIMER3_MIS_R            (*((volatile unsigned long *)0x40033020))
#define TIMER3_ICR_R            (*((volatile unsigned long *)0x40033024))
#define TIMER3_TAILR_R          (*((volatile unsigned long *)0x40033028))
#define TIMER3_TAPR_R           (*((volatile unsigned long *)0x40033038))
#define TIMER3_TAR_R            (*((volatile unsigned long *)0x40033048))


#define SYSCTL_RCGCTIMER_R      (*((volatile unsigned long *)0x400FE604))

//Create definition for Port F
#define GPIO_PORTF_DATA_R       (*((volatile unsigned long *)0x400253FC))
#define GPIO_PORTF_DIR_R        (*((volatile unsigned long *)0x40025400))
#define GPIO_PORTF_AFSEL_R      (*((volatile unsigned long *)0x40025420))
#define GPIO_PORTF_PUR_R        (*((volatile unsigned long *)0x40025510))
#define GPIO_PORTF_DEN_R        (*((volatile unsigned long *)0x4002551C))
#define GPIO_PORTF_LOCK_R       (*((volatile unsigned long *)0x40025520))
#define GPIO_PORTF_CR_R         (*((volatile unsigned long *)0x40025524))
#define GPIO_PORTF_AMSEL_R      (*((volatile unsigned long *)0x40025528))
#define GPIO_PORTF_PCTL_R       (*((volatile unsigned long *)0x4002552C))
//#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
#define UART5_DR_R              (*((volatile unsigned long *)0x40011000))
#define UART5_FR_R              (*((volatile unsigned long *)0x40011018))
#define UART5_IBRD_R            (*((volatile unsigned long *)0x40011024))
#define UART5_FBRD_R            (*((volatile unsigned long *)0x40011028))
#define UART5_LCRH_R            (*((volatile unsigned long *)0x4001102C))
#define UART5_CTL_R             (*((volatile unsigned long *)0x40011030))
#define UART5_CC_R              (*((volatile unsigned long *)0x40011FC8))
	
//update
#define NVIC_EN0_R              (*((volatile unsigned long *)0xE000E100))
#define NVIC_PRI4_R             (*((volatile unsigned long *)0xE000E410))
#define NVIC_PRI3_R             (*((volatile unsigned long *)0xE000E40C))
	
#define SYSCTL_RCGCUART_R       (*((volatile unsigned long *)0x400FE618))
//#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
//#define SYSCTL_RCGCTIMER_R      (*((volatile unsigned long *)0x400FE604))

void HC05_Init(void);
void ADC_Init(void);
void timer0A_delayMs(int ttime);
void timer0_Init(void);
void PortBE_Init(void);
void PortF_Init(void);
//void Display(int digit, int number);
//void NumSplit(int counted);
void DisableInterrupts(void);
void EnableInterrupts(void);
void WaitForInterrupt(void);
void timer1_Init(void);	//for ADC0
void timer2_Init(void); //for counting echo
char blueTooth_Read(void);
void blueTooth_Write(unsigned char data);
void write_string(char *str);
//char switchCheck(void);
void timer3_Init(void);

int temperature=0;
int last_temperature=0;
int diff=0;
int EchoDelay=0;
int distance=0; 
char snum[50];



int main(){
	char BluInput;
	ADC_Init();
  HC05_Init();
  timer0_Init();
  PortBE_Init();
  PortF_Init();
  timer1_Init();
timer2_Init();
 timer3_Init();

  while (1) {
		timer0A_delayMs(1);
		BluInput = blueTooth_Read();
		timer0A_delayMs(1);
    if (BluInput == 'A') { // check temperature
			sprintf(snum,"Current Temperature is: %d", temperature);//50char snum
			write_string("To end the measurement, enter any character except A and B .");
      write_string(snum);
			
      while (BluInput =='A'){
				while(TIMER1_RIS_R&0x01);
				if(temperature>last_temperature){
					diff = temperature - last_temperature;
					sprintf(snum,"Temperature increase: %d", diff);
					write_string(snum);
				}
				else if(temperature<last_temperature){
					diff = last_temperature-temperature;
					sprintf(snum,"Temperature decrease: %d", diff);
					write_string(snum);
				}
				else{
					
					write_string("emperature unchanged");
				}
			}
		}
		else if(BluInput == 'B'){ //check distance
			write_string("Measuring the distance in every 20 second.");
			write_string("To end the measurement, enter any character except A and B .");
			TIMER3_ICR_R =0x01;
			while(BluInput =='B'){
				if((TIMER3_RIS_R & 0x01) ==0x1){
					distance = EchoDelay*340/2*100/50000000;
					sprintf(snum,"Distance is: %d (cm)", distance);
				}
				
			}
		}
	}
}





char blueTooth_Read(void){
		
		char data;
	  while((UART5_FR_R & (1<<4)) != 0); /* wait until Rx buffer is not full */
    
		//switchCheck();
		data = UART5_DR_R ;  	/* before giving it another byte */
		
			return (unsigned char) data;
}

void blueTooth_Write(unsigned char data){
		while ((UART5_FR_R & 0x20)!=0);
				UART5_DR_R = data;
		
}

void write_string(char *str){
		int i;
		for (i=0; i < strlen(str); i++){
				blueTooth_Write(str[i]);
		}
		
}

void HC05_Init(void){
		SYSCTL_RCGCUART_R |= 0x20;
	  timer0A_delayMs(1);
		UART5_CTL_R = 0;
		UART5_IBRD_R = 325;
		UART5_FBRD_R = 34;
		UART5_CC_R = 0;
		UART5_LCRH_R = 0x60;
		UART5_CTL_R = 0x301;
}
void timer0_Init(void){
		SYSCTL_RCGCTIMER_R |= 0x01;     /* b'000001 second bit for TIMER0*/
    TIMER0_CTL_R = 0x00;         /* disable Timer before initialization */
    TIMER0_CFG_R = 0x04;         /* b'100;16-bit Mode */
    TIMER0_TAMR_R = 0x02;        /* periodic mode and down-counter */
    TIMER0_TAILR_R = 16000 - 1;  /* Timer A interval load value register */
    TIMER0_ICR_R = 0x1;          /* clear the TimerA timeout flag*/
    TIMER0_CTL_R |= 0x01;        /* enable Timer A after initialization */
		TIMER0_TAPR_R = 1; // Prescalar value.. Can extend the cycle time max 256 times
}


void timer1_Init(void){					//timer to delay ADC0 for 10 seconds
		SYSCTL_RCGCTIMER_R |= 0x02;     /* b'000010 second bit for TIMER1*/
    TIMER1_CTL_R = 0x00;         /* disable Timer before initialization */
		TIMER1_CFG_R = 0x00;         /* b'000: 32-bit option */
		TIMER1_TAMR_R = 0x02;        /* b'01 one shot, b'10 periodic b'11 capture; b'1xx edge time b'0xx edge count*/
		TIMER1_TAILR_R = 500000000 - 1;  /* 10 second. */
    TIMER1_ICR_R = 0x1;          /* clear the TimerA timeout flag; it goes 0 after reset the timer*/
		TIMER1_CTL_R |= 0x21;        /* h'2x ADC trigger enlable; b'1 timer1 enabled */
	//TIMER1_IMR_R = 0x01;

}
void timer2_Init(void){
		SYSCTL_RCGCTIMER_R |= 0x04;     /* b'000100 second bit for TIMER1*/
    TIMER2_CTL_R = 0x00;         /* disable Timer before initialization */
		TIMER2_CFG_R = 0x04;         /* b'100: 16-bit option */
		TIMER2_TAMR_R = 0x17;        /* b'10111 4bit=1 count UP(output: last- first),3bit=0 capture,
																						2bit=1 edge time, 1~0bit=3 capture mode*/
	//TIMER2_ICR_R = 0x4;          /* No use: it is called in the method*/
		TIMER2_CTL_R |= 0x0C;        /* b'1100 3~2bit=3both edge,1bit=0 no stall,0bit=0 disable yet(still setting) */
		TIMER2_CTL_R |= 0x01;        /* b'1100 3~2bit=3both edge,1bit=0 no stall,0bit=0 disable yet(still setting) */
	//TIMER2_IMR_R = 0x01;

	
}

void timer3_Init(void){					//timer to delay 20 second for Bluetooth input 'B'
		SYSCTL_RCGCTIMER_R |= 0x08;     /* b'001000 second bit for TIMER3*/
    TIMER3_CTL_R = 0x00;         /* disable Timer before initialization */
		TIMER3_CFG_R = 0x00;         /* b'000: 32-bit option */
		TIMER3_TAMR_R = 0x02;        /* b'01 one shot, b'10 periodic b'11 capture; b'1xx edge time b'0xx edge count*/
		TIMER3_TAILR_R = 1000000000 - 1;  /* 20 seconds. */
    TIMER3_ICR_R = 0x1;          /* clear the TimerA timeout flag; it goes 0 after reset the timer*/
		TIMER3_CTL_R |= 0x01;        /* b'1 timer3 enabled */
	//TIMER1_IMR_R = 0x01;

}
void ADC_Init(void){//ADC0IM-address, ADC0ISC-address, NVICpri,NVICen,ADC0pc
		SYSCTL_RCGCADC_R |= 0x01;   // Enable clock ADC0
	SYSCTL_RCGC2_R |= 0x08;
	//ADC0_PC_R = 0x01;
/* initialize PD0 for AIN0 input */
  GPIO_PORTD_AFSEL_R |= 0x1;        /* enable alternate function */
  GPIO_PORTD_DEN_R &= ~0x1;         /* disable digital function */
  GPIO_PORTD_AMSEL_R |= 0x1;        /* enable analog function */

/* initialize ADC0 */
  ADC0_ACTSS_R &= ~0x8;             /* disable SS3 during configuration */
  ADC0_EMUX_R |= 0x5000;         /* software trigger conversion */
  ADC0_SSMUX3_R |= 0x7;              /* d'7 AIN7 =PD0  */
  ADC0_SSCTL3_R |= 0x6;             /* b'0110 b3'1: internal temp sensor b2'1: raw interrupt signal (INRn bit) */
	/* b1'1: end the sequence b0'1differentially sampled*/
  ADC0_ACTSS_R |= 0x8;              /* b'1000 enable SS3 */
	/*ADC0_SSPRI_R = 0x0123;						 priotize SS3 and SS2 SS1 SS0 0000 0001 0010 0011(SS3,SS2,SS1,SS0)*/
	ADC0_IM_R = 0x8;									//interrupt mask; enable interrupt mask3 which is the alined componont with SS3
	NVIC_EN0_R = 0x20000; /*enable SS3 on bit 1 */
	NVIC_PRI4_R = (NVIC_PRI4_R &0xFFFF00FF) | 0x6000;
	ADC0_PC_R = 0x00;
	ADC0_ISC_R =0x08;

}

// Subroutine to initialize ports B, E
//PB0 is used for distance measuring
void PortBE_Init(void){
  SYSCTL_RCGC2_R |= 0x12;     // Port B,E clock initialized
  GPIO_PORTB_CR_R = 0x7F;           // Allow changes to PB6-PB0
  GPIO_PORTB_AMSEL_R = 0x00;        // Disable analog function
  GPIO_PORTB_PCTL_R = 0x00000000;   // GPIO clear bit PCTL
  GPIO_PORTB_DIR_R = 0x7F;          // Set PB6-PB0 outputs
  GPIO_PORTB_PUR_R = 0x00;          // Enable pullup resistors on PB4,PF0
  GPIO_PORTB_DEN_R = 0x7F;          // 7) Enable digital pins PB6-PB0

	//GPIO_PORTE_CR_R = 0x30;           // Allow changes to PE4-0       
	GPIO_PORTE_AMSEL_R = 0x00;        // Disable analog function
	GPIO_PORTE_PCTL_R = 0x00110000;   // GPIO clear bit PCTL  
	// GPIO_PORTE_DIR_R = 0x0F;          // PE3-PE0 output   
	// GPIO_PORTE_PUR_R = 0x00;          // Disable pullup resistors       
	GPIO_PORTE_DEN_R = 0x30;          // Enable digital pins PE3-PE0  
	GPIO_PORTE_AFSEL_R = 0x30;				// Alternate function
	timer0A_delay(2);

  	GPIO_PORTE_CR_R |= 0x3F;           // Allow changes to PE4-0
  	//GPIO_PORTE_AMSEL_R = 0x00;        // Disable analog function
  	//GPIO_PORTE_PCTL_R = 0x00000000;   // GPIO clear bit PCTL
  	GPIO_PORTE_DIR_R |= 0x2F;          // PE3-PE0 output
  	//GPIO_PORTE_PUR_R = 0x00;          // Disable pullup resistors
	GPIO_PORTE_DEN_R |= 0x3F;          // Enable digital pins PE3-PE0   
}

void PortF_Init(void){ volatile unsigned long d;
  SYSCTL_RCGC2_R |= 0x20;     // 1) F clock 10 0000
  d = SYSCTL_RCGC2_R;               //Dummy instruction to wait for clock to stabilize
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock PortF PF0
  GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0
  GPIO_PORTF_AMSEL_R = 0x00;        // 3) disable analog function
  GPIO_PORTF_PCTL_R = 0x00000000;   // 4) GPIO clear bit PCTL
  GPIO_PORTF_DIR_R = 0x0E;          // 5) PF4,PF0 input, PF3,PF2,PF1 output
  GPIO_PORTF_AFSEL_R = 0x00;        // 6) no alternate function
  GPIO_PORTF_PUR_R = 0x11;          // enable pullup resistors on PF4,PF0
  GPIO_PORTF_DEN_R = 0x1F;          // 7) enable digital pins PF4-PF0
}

void ADC0SS3_Handler(void){	//measure the temperature
		timer0A_delayMs(1);
		// Trigger the ADC conversion
    ADC0_PSSI_R |= 0x08;
	
    // Read the temperature from the ADC
		while((ADC0_RIS_R & 0x08)==0);
    temperature = (ADC0_SSFIFO3_R & 0xFFF);
    // Calculate the temperature in Celsius
    temperature = ((temperature * 165) / 2300) - 50;
    // Check if the temperature has decreased
    if (last_temperature-temperature >2) {
    GPIO_PORTF_DATA_R = 0x02; // turn on red LED
    timer0A_delayMs(500); // blink twice
    GPIO_PORTF_DATA_R = 0x00; // turn off LED
    timer0A_delayMs(500);
    }
    else if (temperature - last_temperature>2) {
        // Temperature has increased
        GPIO_PORTF_DATA_R = 0x04;  // Turn on red LED
        timer0A_delayMs(500);
        GPIO_PORTF_DATA_R = 0x00; // Turn off red LED
        timer0A_delayMs(500);
    }

    // Save the current temperature for the next comparison
    last_temperature = temperature;
		// Wait for 10 second
		while(TIMER1_RIS_R&0x01);

    // Clear the ADC interrupt flag
    ADC0_ISC_R |= 0x08;

    // Clear the timer interrupt flag
    TIMER1_ICR_R = 0x01;
}
  
void timer0A_delayMs(int ttime)
{
    timer0_Init();
    int i;
    for(i = 0; i < ttime; i++) {
        while ((TIMER0_RIS_R & 0x01) == 0);      /* wait for TimerA timeout flag */
        TIMER0_ICR_R = 0x01;      /* clear the TimerA timeout flag */
    }
}

void TIMER2A_Handler(void){ //capture timer2A to get start posedge and end negedge.
	int startEdge, endEdge;
	//Trigger is connected to PA4
	GPIO_PORTB_DATA_R &= ~ 0x10; //reset PA4 for the trigger
	timer0A_delayMs(1);
	GPIO_PORTB_DATA_R |=  0x10; //make PA4 high for the trigger
	timer0A_delayMs(10);
	GPIO_PORTB_DATA_R &= ~0x10; //make PA4 low after 10 microsecond
	//Echo is connected to PB0 - PCTL PB0= T2CCP0(TIMER2A)
	
		TIMER2_ICR_R = 0x4; 								//clear timer2 interrupt for the start edge (reset RIS 3bit: Raw interrupt)
		while((TIMER2_RIS_R & 0x4) ==0);		//Wait for the timer2 to start capturing
		if((GPIO_PORTB_DATA_R & 0x1)==0x1){	//if PB0 rising edge occurs save it to startEdge
			startEdge = TIMER2_TAR_R;					//save the time stamp
			TIMER2_ICR_R = 0x4;								//clear timer2 interrupt for the end edge
			/* PB0 should be 0 to get negative edge*/
			while((GPIO_PORTB_DATA_R & 0x1)==0x1);		//Wait for the end edge- same as if((GPIO_PORTB_DATA_R & 0x1)==0x0){ ~ }
			TIMER2_ICR_R = 0x4;
			/* For capture mode, TIMER2_RIS_R should be 0 */
			while((TIMER2_RIS_R & 0x4)== 0){ 
				endEdge = TIMER2_TAR_R;
			}
		}
		EchoDelay = endEdge - startEdge;
		TIMER3_ICR_R =0x01;
}
void DisableInterrupts(void)
{
    __asm ("    CPSID  I\n");
}

void EnableInterrupts(void)
{
    __asm  ("    CPSIE  I\n");
}

void WaitForInterrupt(void)
{
    __asm  ("    WFI\n");
//GPIOPortF_Handler();
}