#include <XC.h>
#include <sys/attribs.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "lcd.h"
//REMOTE CONTROLLER MASTER
/* Pinout for DIP28 PIC32MX130:
                                          --------
                                   MCLR -|1     28|- AVDD 
  VREF+/CVREF+/AN0/C3INC/RPA0/CTED1/RA0 -|2     27|- AVSS 
        VREF-/CVREF-/AN1/RPA1/CTED2/RA1 -|3     26|- AN9/C3INA/RPB15/SCK2/CTED6/PMCS1/RB15
   PGED1/AN2/C1IND/C2INB/C3IND/RPB0/RB0 -|4     25|- CVREFOUT/AN10/C3INB/RPB14/SCK1/CTED5/PMWR/RB14
  PGEC1/AN3/C1INC/C2INA/RPB1/CTED12/RB1 -|5     24|- AN11/RPB13/CTPLS/PMRD/RB13
   AN4/C1INB/C2IND/RPB2/SDA2/CTED13/RB2 -|6     23|- AN12/PMD0/RB12
     AN5/C1INA/C2INC/RTCC/RPB3/SCL2/RB3 -|7     22|- PGEC2/TMS/RPB11/PMD1/RB11
                                    VSS -|8     21|- PGED2/RPB10/CTED11/PMD2/RB10
                     OSC1/CLKI/RPA2/RA2 -|9     20|- VCAP
                OSC2/CLKO/RPA3/PMA0/RA3 -|10    19|- VSS
                         SOSCI/RPB4/RB4 -|11    18|- TDO/RPB9/SDA1/CTED4/PMD3/RB9
         SOSCO/RPA4/T1CK/CTED9/PMA1/RA4 -|12    17|- TCK/RPB8/SCL1/CTED10/PMD4/RB8
                                    VDD -|13    16|- TDI/RPB7/CTED3/PMD5/INT0/RB7
                    PGED3/RPB5/PMD7/RB5 -|14    15|- PGEC3/RPB6/PMD6/RB6
                                          --------
*/
 
// Configuration Bits (somehow XC32 takes care of this)
#pragma config FNOSC = FRCPLL       // Internal Fast RC oscillator (8 MHz) w/ PLL
#pragma config FPLLIDIV = DIV_2     // Divide FRC before PLL (now 4 MHz)
#pragma config FPLLMUL = MUL_20     // PLL Multiply (now 80 MHz)
#pragma config FPLLODIV = DIV_2     // Divide After PLL (now 40 MHz)
 
#pragma config FWDTEN = OFF         // Watchdog Timer Disabled
#pragma config FPBDIV = DIV_1       // PBCLK = SYCLK
#pragma config FSOSCEN = OFF        // Turn off secondary oscillator on A4 and B4
// Defines
#define SYSCLK 40000000L
#define DEF_FREQ 16000L
//#define PWM_FREQ    2000L
#define DUTY_CYCLE  50
#define Baud2BRG(desired_baud)( (SYSCLK / (16*desired_baud))-1)
#define Baud1BRG(desired_baud)( (SYSCLK / (16*desired_baud))-1)

#define FREQ 100000L
// servo values
volatile int ISR_pw=100, ISR_cnt=0, ISR_frc;
 
// The Interrupt Service Routine for timer 1 is used to generate one or more standard
// hobby servo signals.  The servo signal has a fixed period of 20ms and a pulse width
// between 0.6ms and 2.4ms.
void __ISR(_TIMER_1_VECTOR, IPL5SOFT) Timer1_Handler(void)
{
	IFS0CLR=_IFS0_T1IF_MASK; // Clear timer 1 interrupt flag, bit 4 of IFS0

	ISR_cnt++;
	if(ISR_cnt<ISR_pw)
	{
		LATBbits.LATB0 = 1;
	}
	else
	{
		LATBbits.LATB0 = 0;
	}
	if(ISR_cnt>=2000)
	{
		ISR_cnt=0; // 2000 * 10us=20ms
		ISR_frc++;
	}
}

void SetupTimer1 (void)
{
	// Explanation here: https://www.youtube.com/watch?v=bu6TTZHnMPY
	__builtin_disable_interrupts();
	PR1 =(SYSCLK/FREQ)-1; // since SYSCLK/FREQ = PS*(PR1+1)
	TMR1 = 0;
	T1CONbits.TCKPS = 0; // 3=1:256 prescale value, 2=1:64 prescale value, 1=1:8 prescale value, 0=1:1 prescale value
	T1CONbits.TCS = 0; // Clock source
	T1CONbits.ON = 1;
	IPC1bits.T1IP = 5;
	IPC1bits.T1IS = 0;
	IFS0bits.T1IF = 0;
	IEC0bits.T1IE = 1;
	
	INTCONbits.MVEC = 1; //Int multi-vector
	__builtin_enable_interrupts();
}

void UART2Configure(int baud_rate)
{
    // Peripheral Pin Select
    U2RXRbits.U2RXR = 4;    //SET RX to RB8
    RPB9Rbits.RPB9R = 2;    //SET RB9 to TX

    U2MODE = 0;         // disable autobaud, TX and RX enabled only, 8N1, idle=HIGH
    U2STA = 0x1400;     // enable TX and RX
    U2BRG = Baud2BRG(baud_rate); // U2BRG = (FPb / (16*baud)) - 1
    
    U2MODESET = 0x8000;     // enable UART2
}

// Needed to by scanf() and gets()
int _mon_getc(int canblock)
{
	char c;
	
    if (canblock)
    {
	    while( !U2STAbits.URXDA); // wait (block) until data available in RX buffer
	    c=U2RXREG;
        while( U2STAbits.UTXBF);    // wait while TX buffer full
        U2TXREG = c;          // echo
	    if(c=='\r') c='\n'; // When using PUTTY, pressing <Enter> sends '\r'.  Ctrl-J sends '\n'
		return (int)c;
    }
    else
    {
        if (U2STAbits.URXDA) // if data available in RX buffer
        {
		    c=U2RXREG;
		    if(c=='\r') c='\n';
			return (int)c;
        }
        else
        {
            return -1; // no characters to return
        }
    }
}

/////////////////////////////////////////////////////////
// UART1 functions used to communicate with the JDY40  //
/////////////////////////////////////////////////////////

// TXD1 is in pin 26
// RXD1 is in pin 24

int UART1Configure(int desired_baud)
{
	int actual_baud;

    // Peripheral Pin Select for UART1.  These are the pins that can be used for U1RX from TABLE 11-1 of '60001168J.pdf':
    // 0000 = RPA2
	// 0001 = RPB6
	// 0010 = RPA4
	// 0011 = RPB13
	// 0100 = RPB2

	// Do what the caption of FIGURE 11-2 in '60001168J.pdf' says: "For input only, PPS functionality does not have
    // priority over TRISx settings. Therefore, when configuring RPn pin for input, the corresponding bit in the
    // TRISx register must also be configured for input (set to ?1?)."
    
    ANSELB &= ~(1<<13); // Set RB13 as a digital I/O
    TRISB |= (1<<13);   // configure pin RB13 as input
    CNPUB |= (1<<13);   // Enable pull-up resistor for RB13
    U1RXRbits.U1RXR = 3; // SET U1RX to RB13

    // These are the pins that can be used for U1TX. Check table TABLE 11-2 of '60001168J.pdf':
    // RPA0
	// RPB3
	// RPB4
	// RPB15
	// RPB7

    ANSELB &= ~(1<<15); // Set RB15 as a digital I/O
    RPB15Rbits.RPB15R = 1; // SET RB15 to U1TX
	
    U1MODE = 0;         // disable autobaud, TX and RX enabled only, 8N1, idle=HIGH
    U1STA = 0x1400;     // enable TX and RX
    U1BRG = Baud1BRG(desired_baud); // U1BRG = (FPb / (16*baud)) - 1
    // Calculate actual baud rate
    actual_baud = SYSCLK / (16 * (U1BRG+1));

    U1MODESET = 0x8000;     // enable UART1

    return actual_baud;
}

void putc1 (char c)
{
	while( U1STAbits.UTXBF);   // wait while TX buffer full
	U1TXREG = c;               // send single character to transmit buffer
}
 
int SerialTransmit1(const char *buffer)
{
    unsigned int size = strlen(buffer);
    while(size)
    {
        while( U1STAbits.UTXBF);    // wait while TX buffer full
        U1TXREG = *buffer;          // send single character to transmit buffer
        buffer++;                   // transmit next character on following loop
        size--;                     // loop until all characters sent (when size = 0)
    }
 
    while( !U1STAbits.TRMT);        // wait for last transmission to finish
 
    return 0;
}
 
unsigned int SerialReceive1(char *buffer, unsigned int max_size)
{
    unsigned int num_char = 0;
 
    while(num_char < max_size)
    {
        while( !U1STAbits.URXDA);   // wait until data available in RX buffer
        *buffer = U1RXREG;          // empty contents of RX buffer into *buffer pointer
 
        // insert nul character to indicate end of string
        if( *buffer == '\n')
        {
            *buffer = '\0';     
            break;
        }
 
        buffer++;
        num_char++;
    }
 
    return num_char;
}

// Copied from here: https://forum.microchip.com/s/topic/a5C3l000000MRVAEA4/t335776
void delayus(uint16_t uiuSec)
{
    uint32_t ulEnd, ulStart;
    ulStart = _CP0_GET_COUNT();
    ulEnd = ulStart + (SYSCLK / 2000000) * uiuSec;
    if(ulEnd > ulStart)
        while(_CP0_GET_COUNT() < ulEnd);
    else
        while((_CP0_GET_COUNT() > ulStart) || (_CP0_GET_COUNT() < ulEnd));
}

void delay_ms (int msecs)
{	
	int ticks;
	ISR_frc=0;
	ticks=msecs/20;
	while(ISR_frc<ticks);
}

unsigned int SerialReceive1_timeout(char *buffer, unsigned int max_size)
{
    unsigned int num_char = 0;
    int timeout_cnt;
 
    while(num_char < max_size)
    {
    	timeout_cnt=0;
    	while(1)
    	{
	        if(U1STAbits.URXDA) // check if data is available in RX buffer
	        {
	        	*buffer = U1RXREG; // copy RX buffer into *buffer pointer
	        	break;
	        }
	        if(++timeout_cnt==100) // 100 * 100us = 10 ms
	        {
	        	*buffer = '\n';
	        	break;
	        }
	        delayus(100);
	    }
 
        // insert nul character to indicate end of string
        if( *buffer == '\n')
        {
            *buffer = '\0';     
            break;
        }
 
        buffer++;
        num_char++;
    }
 
    return num_char;
}

// Use the core timer to wait for 1 ms.
void wait_1ms(void)
{
    unsigned int ui;
    _CP0_SET_COUNT(0); // resets the core timer count

    // get the core timer count
    while ( _CP0_GET_COUNT() < (SYSCLK/(2*1000)) );
}

void delayms(int len)
{
	while(len--) wait_1ms();
}

void ClearFIFO (void)
{
	unsigned char c;
	U1STA = 0x1400;     // enable TX and RX, clear FIFO
	while(U1STAbits.URXDA) c=U1RXREG;
}

void SendATCommand (char * s)
{
	char buff[40];
	printf("Command: %s", s);
	LATB &= ~(1<<14); // 'SET' pin of JDY40 to 0 is 'AT' mode.
	delayms(10);
	SerialTransmit1(s);
	U1STA = 0x1400;     // enable TX and RX, clear FIFO
	SerialReceive1(buff, sizeof(buff)-1);
	LATB |= 1<<14; // 'SET' pin of JDY40 to 1 is normal operation mode.
	delayms(10);
	printf("Response: %s\n", buff);
}

void ReceptionOff (void)
{
	LATB &= ~(1<<14); // 'SET' pin of JDY40 to 0 is 'AT' mode.
	delayms(10);
	SerialTransmit1("AT+DVID0000\r\n"); // Some unused id, so that we get nothing.
	delayms(10);
	ClearFIFO();
	LATB |= 1<<14; // 'SET' pin of JDY40 to 1 is normal operation mode.
}

void Timer4us(unsigned char t) 
{
	T4CON = 0x8000; // enable Timer4, source PBCLK, 1:1 prescaler
 
    // delay 100us per loop until less than 100us remain
    while( t >= 100){
        t-=100;
        TMR4=0;
        while(TMR4 < SYSCLK/10000L);
    }
 
    // delay 10us per loop until less than 10us remain
    while( t >= 10){
        t-=10;
        TMR4=0;
        while(TMR4 < SYSCLK/100000L);
    }
 
    // delay 1us per loop until finished
    while( t > 0)
    {
        t--;
        TMR4=0;
        while(TMR4 < SYSCLK/1000000L);
    }
    // turn off Timer4 so function is self-contained
    T4CONCLR=0x8000;
}

void waitms(unsigned int ms)
{
	unsigned int j;
	unsigned char k;
	for(j=0;j<ms;j++)
		for(k=0;k<4;k++)
			Timer4us(250);
}

void LCD_pulse(void)
{
	LCD_E = 1;
	Timer4us(40);
	LCD_E = 0;
}

void LCD_byte(unsigned char x)
{
	LCD_D7=(x&0x80)?1:0;
	LCD_D6=(x&0x40)?1:0;
	LCD_D5=(x&0x20)?1:0;
	LCD_D4=(x&0x10)?1:0;
	LCD_pulse();
	Timer4us(40);
	LCD_D7=(x&0x08)?1:0;
	LCD_D6=(x&0x04)?1:0;
	LCD_D5=(x&0x02)?1:0;
	LCD_D4=(x&0x01)?1:0;
	LCD_pulse();
}

void WriteData(unsigned char x)
{
	LCD_RS = 1;
	LCD_byte(x);
	waitms(2);
}

void WriteCommand(unsigned char x)
{
	LCD_RS = 0;
	LCD_byte(x);
	waitms(5);
}

void LCD_4BIT(void)
{
	// Configure the pins used to communicate with the LCD as outputs
	LCD_RS_ENABLE = 0;
	LCD_E_ENABLE = 0;
	LCD_D4_ENABLE = 0;
	LCD_D5_ENABLE = 0;
	LCD_D6_ENABLE = 0;
	LCD_D7_ENABLE = 0;
	
	LCD_E = 0; // Resting state of LCD's enable is zero
	// LCD_RW = 0; Not used in this code.  Connect to ground.
	waitms(20);
	// First make sure the LCD is in 8-bit mdode, then change to 4-bit mode
	WriteCommand(0x33);
	WriteCommand(0x33);
	WriteCommand(0x32); // Change to 4-bit mode
	
	// Configure the LCD
	WriteCommand(0x28);
	WriteCommand(0x0c);
	WriteCommand(0x01); // Clear screen command (takes some time)
	waitms(20); // Wait for clear screen command to finish
	LATBbits.LATB0 = 	!LATBbits.LATB0;
}

void LCDprint(char * string, unsigned char line, unsigned char clear)
{
	int j;
	
	WriteCommand(line==2?0xc0:0x80);
	waitms(5);
	for(j=0;string[j]!=0;j++)
		WriteData(string[j]); //Write the message character by character
	if(clear)
		for(;j<CHARS_PER_LINE;j++)
			WriteData(' '); //Clear the rest of the line if clear is 1
}

void Init_pwm (long PWM_FREQ)
{
    // OC1 can be assigned to PA0, PB3, PB4, PB15, and PB7(in use).
    // Check TABLE 11-2: OUTPUT PIN SELECTION in datasheet.
    // Set OC1 to pin PA0 (pin 2 of DIP 28) with peripheral pin select
    RPA0Rbits.RPA0R = 0x0005;
 
    // Configure standard PWM mode for output compare module 1
    OC1CON = 0x0006; 
 
	T2CONbits.TCKPS=0x0; // Set pre-scaler to 1	
	
    // A write to PRy configures the PWM frequency
    // PR = [FPB / (PWM Frequency * TMR Prescale Value)] ? 1
    PR2 = (SYSCLK / (PWM_FREQ*1)) - 1;
 
    // A write to OCxRS configures the duty cycle
    // : OCxRS / PRy = duty cycle
    OC1RS = (PR2 + 1) * ((float)DUTY_CYCLE / 100);

 	T2CON = 0x0;
    T2CONSET = 0x8000;      // Enable Timer2, prescaler 1:1
    OC1CONSET = 0x8000;     // Enable Output Compare Module 1
}



void Set_pwm (unsigned char val)
{
	OC1RS = (PR2 + 1) * ((float)val / 256.0);
}

void ADCConf(void)
{
    AD1CON1CLR = 0x8000;    // disable ADC before configuration
    AD1CON1 = 0x00E0;       // internal counter ends sampling and starts conversion (auto-convert), manual sample
    AD1CON2 = 0;            // AD1CON2<15:13> set voltage reference to pins AVSS/AVDD
    AD1CON3 = 0x0f01;       // TAD = 4*TPB, acquisition time = 15*TAD 
    AD1CON1SET=0x8000;      // Enable ADC
}

int ADCRead(char analogPIN)
{
    AD1CHS = analogPIN << 16;    // AD1CHS<16:19> controls which analog pin goes to the ADC
 
    AD1CON1bits.SAMP = 1;        // Begin sampling
    while(AD1CON1bits.SAMP);     // wait until acquisition is done
    while(!AD1CON1bits.DONE);    // wait until conversion done
 
    return ADC1BUF0;             // result stored in ADC1BUF0
}

void ConfigurePins(void)
{
    // Configure pins as analog inputs
    ANSELBbits.ANSB2 = 1;   // set RB2 (AN4, pin 6 of DIP28) as analog pin
    TRISBbits.TRISB2 = 1;   // set RB2 as an input
    ANSELBbits.ANSB1 = 1;	// set rb1 an3 as analog pin
	TRISBbits.TRISB1 = 1;   // set rb1 as an input
  /*
    ANSELBbits.ANSB3 = 1;   // set RB3 (AN5, pin 7 of DIP28) as analog pin
    TRISBbits.TRISB3 = 1;   // set RB3 as an input
    
	// Configure digital input pin to measure signal period
	ANSELB &= ~(1<<5); // Set RB5 as a digital I/O (pin 14 of DIP28)
    TRISB |= (1<<5);   // configure pin RB5 as input
    CNPUB |= (1<<5);   // Enable pull-up resistor for RB5
    
    // We can do the three lines above using this instead:
    // ANSELBbits.ANSELB5=0;  Not needed because RB5 can not be analog input?
    // TRISBbits.TRISB5=1;
    // CNPUBbits.CNPUB5=1;
    
    // Configure output pins
	TRISAbits.TRISA0 = 0; // pin  2 of DIP28
	TRISAbits.TRISA1 = 0; // pin  3 of DIP28
	TRISBbits.TRISB0 = 0; // pin  4 of DIP28
	TRISBbits.TRISB1 = 0; // pin  5 of DIP28
	TRISAbits.TRISA2 = 0; // pin  9 of DIP28
	TRISAbits.TRISA3 = 0; // pin 10 of DIP28
	TRISBbits.TRISB4 = 0; // pin 11 of DIP28
	INTCONbits.MVEC = 1;
*/
}

void uart_puts(char * s)
{
	while(*s)
	{
		putchar(*s);
		s++;
	}
}

char HexDigit[]="0123456789ABCDEF";
void PrintNumber(long int val, int Base, int digits)
{ 
	int j;
	#define NBITS 32
	char buff[NBITS+1];
	buff[NBITS]=0;

	j=NBITS-1;
	while ( (val>0) | (digits>0) )
	{
		buff[j--]=HexDigit[val%Base];
		val/=Base;
		if(digits!=0) digits--;
	}
	uart_puts(&buff[j+1]);
}

/*
returns 2 ADCvalues to convert to pwm signals
hopes does the ratio properly
fornite skibbi balls 
*/
int CoinDecider(long int freq)
{
	if(freq>=56300) // detects a coin
	{
		// if frequency is in dime range 10 cents
		if((freq >= 56200) && (freq < 56400))
		{
			printf(" DIME");
			return 1;
		}

		// nickel 25 cents
		else if ((freq >= 56400) && (freq < 56700))
		{
			printf(" NICKEL");
			return 1;
		}

		// loonie 1 dollar
		else
		{
			printf(" LOONIE");
		}

		return 1;
	}

	else
	{
//		printf(" NO COIN");
	}

	return 0;
}

unsigned int ADCtoPWM(int adc_value)
{
//	if ( adc_value == 503 || adc_value == 504 ) adc_value = 0;
//    else if(adc_value > 1023) adc_value = 1023; // Protection against overflow

    return (unsigned int)((adc_value * 65535UL) / 1023UL);
}

/*
returns 2 ADCvalues to convert to pwm signals
hopes does the ratio properly
fornite skibbi balls 
*/
void ADCsteeringRatio(int steering, int speed, int *ADCwheel1, int *ADCwheel2) 
{
	  int centersteering;
	  int centerspeed;
	  float steeringFactor;
	  int baseSpeed;
	  int baseSteer;
	  int wheel1Speed;
	  int wheel2Speed;
	  int delta;
	
	
	 centersteering = steering - 508;
	 centerspeed = speed - 504;
	// incase is 1 or something but lowkey idk is low enough the pwm signal wont turn it
	 baseSpeed = abs(centerspeed);
	 baseSteer = abs(centersteering);
	 if ( baseSpeed < 3 && baseSteer < 3 ) 
	 {
	 	*ADCwheel1 = 0;
	 	*ADCwheel2 = 0;
	 	return;
	 } 
		
	 	
	 steeringFactor = (float)centersteering / 508; // ranges from -1.0 (full left) to +1.0 (full right)
	
	if ( steeringFactor > 1 ) steeringFactor = 1;
	
	    // Calculate
	delta = ((int)(baseSpeed * steeringFactor));
	    		
	 wheel1Speed = baseSpeed + delta;
	 wheel2Speed = baseSpeed - delta;
	if (wheel1Speed > 507) wheel1Speed = 507;
	if (wheel1Speed < 0) wheel1Speed = 0;
	
	if (wheel2Speed > 507) wheel2Speed = 507;
	if (wheel2Speed < 0) wheel2Speed = 0;
	
	if ( baseSpeed < 3 && baseSteer > 3 ) 
	{
		
		wheel1Speed = 507 + centersteering;
		wheel2Speed = 507 - centersteering;	
		
		if (wheel1Speed > 507) wheel1Speed = 507;
		if (wheel1Speed < 0) wheel1Speed = 0;
		
		if (wheel2Speed > 507) wheel2Speed = 507;
		if (wheel2Speed < 0) wheel2Speed = 0;
	}
	*ADCwheel1 = (unsigned int)((wheel1Speed * 1023L) / 507L);
	*ADCwheel2 = (unsigned int)((wheel2Speed * 1023L) / 507L);	
}

void main(void)
{
	char buff[80];
	char sendbuff[80];
    int timeout_cnt=0, coindecide, coinflag;
    int cont1=0, cont2=100;
    unsigned char evilcode;
	volatile unsigned long t=0;
	unsigned char myduty=0;
	unsigned int thing, thing1, thing2;
	float stringtobuff;
	int adcvalx;
	int adcvaly;
	int adcvalx1, adcvaly1;
	int newadcx, newadcy;
	int which;
	long int counter = 0;
	char c;
	
	// define pin 4 as servo output
	TRISBbits.TRISB0 = 0; // Set RB0 as output
	LATBbits.LATB0 = 0;   // Start LOW
	INTCONbits.MVEC = 1;
	
	SetupTimer1();
	
	ISR_pw = 60; // initialize "locked" position of servo
    
	DDPCON = 0;
	CFGCON = 0;
	Init_pwm(25);
	LCD_4BIT();
    UART2Configure(115200);  // Configure UART2 for a baud rate of 115200
    UART1Configure(9600);  // Configure UART1 to communicate with JDY40 with a baud rate of 9600
 
	delayms(500); // Give putty time to start before we send stuff.
    printf("PIC32 MASTER REMOTE CONTROL TEST\r\n");
	// RB14 is connected to the 'SET' pin of the JDY40.  Configure as output:;
    ANSELB &= ~(1<<14); // Set RB14 as a digital I/O
    TRISB &= ~(1<<14);  // configure pin RB14 as output
	LATB |= (1<<14);    // 'SET' pin of JDY40 to 1 is normal operation mode
	
	ANSELB &= ~(1<<10); // Set RB14 as a digital I/O
    TRISB &= ~(1<<10);  // configure pin RB14 as output
	LATB |= (1<<10);    // 'SET' pin of JDY40 to 1 is normal operation mode
	
	ANSELB &= ~(1<<12); // Set RB14 as a digital I/O
    TRISB &= ~(1<<12);  // configure pin RB14 as output
	LATB |= (1<<12);    // 'SET' pin of JDY40 to 1 is normal operation mode
	   // 'SET' pin of JDY40 to 1 is normal operation mode

	ANSELAbits.ANSA1 = 0; // Make RA1 digital
	TRISAbits.TRISA1 = 1; // Set RA1 as input
	CNPUAbits.CNPUA1 = 1; // Enable pull-up resistor (optional)
	
	ReceptionOff();
	ConfigurePins();
 	ADCConf(); // Configure ADC
	// To check configuration
	SendATCommand("AT+VER\r\n");
	SendATCommand("AT+BAUD\r\n");
	SendATCommand("AT+RFID\r\n");
	SendATCommand("AT+DVID\r\n");
	SendATCommand("AT+RFC120\r\n");
	SendATCommand("AT+POWE\r\n");
	SendATCommand("AT+CLSS\r\n");
	// We should select an unique device ID.  The device ID can be a hex
	// number from 0x0000 to 0xFFFF.  In this case is set to 0xABBA
	SendATCommand("AT+DVIDFFFF\r\n");
	Set_pwm(0);
	while(1)
	{	

  	
		
		adcvalx = ADCRead(4);
		adcvaly = ( ADCRead(3));
		if ( adcvaly < 450 ) which = 0;
	
		else which = 1;
		putc1('!');
				ADCsteeringRatio(adcvalx, adcvaly, &newadcx, &newadcy);
		thing = ADCtoPWM(newadcx);
		thing1 = ADCtoPWM(newadcy);	
		sprintf(sendbuff, "K%uW%uG%d\n", thing, thing1, which);
		SerialTransmit1(sendbuff);
		waitms(25);
		putc1('@');
		waitms(25);

		if(U1STAbits.URXDA) // Something has arrived from the slave
			{
				SerialReceive1(buff, sizeof(buff)-1);
		//		printf("asdiubasd");
				
		//		printf("%s\n\r", buff);
				stringtobuff = atof(buff);
				evilcode = ((stringtobuff - 63400.0) * 255.0 / (64500.0 - 63400.0));


				if (evilcode > 255 ) evilcode =255;
				if (evilcode < 70 ) 
				{
				evilcode = 0;
				Init_pwm(1);
				}
				Init_pwm(2000L);
				Set_pwm(evilcode);
				printf("%s %u\n\r", buff, evilcode);
				LCDprint(buff, 1,1);
			
			} else 
			{
			ClearFIFO();
			UART1Configure(9600);
			}
		


	//	printf("jacob park has cp\n\r");
//		printf ("sendbuff= %s %u %u, %d\n\r",sendbuff, thing, thing1, which);	
		
		thing = PORTB&(1<<10) ? 0 : 1;
			if( thing == 1 ) 
				{
				printf ("the automatic mode button has been pressed\n\r");
				waitms(5000);
				sprintf(sendbuff, "A"); 
				SerialTransmit1(sendbuff);
				delayms(5000);
    			UART1Configure(9600);
    			ClearFIFO();
			}
			
		thing1 = (PORTAbits.RA1==0);
			if ( thing1 == 1)
			{
				printf ("the servo mode button has been pressed\n\r");
				waitms(5000);
				sprintf(sendbuff, "S"); 
				SerialTransmit1(sendbuff);
				delayms(5000);	
    			UART1Configure(9600);
    			ClearFIFO();			
			}
			
		thing2 = PORTB&(1<<12) ? 0 : 1;
            if( thing2 == 1 ) 
                {
                printf("launching coins!!\n\r");
                //launch coins
                //ISR_pw = 240;
        Init_pwm(25);
        Set_pwm(127);
  		waitms(500);
  		Init_pwm(50);	
    			UART1Configure(9600);
    			ClearFIFO();
                //ISR_pw = 60;
            }
	//		printf("%s\n\r", sendbuff);
			
		
		delayms(50);  // Set the information interchange pace: communicate about every 50ms
	}
}
