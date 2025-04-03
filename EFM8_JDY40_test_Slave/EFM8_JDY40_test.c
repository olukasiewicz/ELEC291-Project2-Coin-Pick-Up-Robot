#include <EFM8LB1.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#define SYSCLK 72000000
#define BAUDRATE 115200L

#define SARCLK 18000000L // needed for initializing the ADC
#define PERIOD_PIN P0_6
#define VDD 3.3035 // The measured value of VDD in volts
#define p_thresh 0.1
//////////////LCD/////////////

#define LCD_RS P2_6
//#define LCD_RW Px_x // Not used in this code.  Connect to GND
#define LCD_E  P2_5
#define LCD_D4 P2_4
#define LCD_D5 P2_3
#define LCD_D6 P2_2
#define LCD_D7 P2_1
#define CHARS_PER_LINE 16

const unsigned char customMouth[8] = {0x0E,0x04,0x00,0x00,0x11,0x15,0x0A,0x00};
const unsigned char customEye[8] = {0x00,0x0E,0x19,0x19,0x1F,0x17,0x0E,0x00};
const unsigned char customOpenMouth[8] = {0x0E,0x04,0x00,0x0E,0x11,0x11,0x0E,0x00};
const unsigned char customSparkle [8] = {0x04,0x04,0x0A,0x11,0x0A,0x04,0x04,0x00};
const unsigned char customMoney [8] = {0x04,0x0E,0x15,0x14,0x0E,0x05,0x15,0x0E};
const unsigned char customHappyMouth [8] = {0x0E,0x04,0x00,0x15,0x0A,0x0A,0x0E,0x00};

//////////////////////////////
/*
						P3_7=0;  //wheel 1
						P3_2=0;	// wheel 1 
						P3_0=0; // wheel 2
						P2_5=0; // wheel 2
*/
////////////timer//////////////////////////
#define PWM_FREQ 10000L
////////////timer5 pwm///////////////////////
volatile unsigned int pwm_counter4=0;
volatile unsigned int pwm_duty4=65535; //(0?65535)
#define TIMER4_RELOAD (0x10000L - (SYSCLK/(12L*PWM_FREQ)))
#define PWMOUT4 P3_0
#define PWMOUT4R P2_5
////////////timer2 pwm///////////////////////
volatile unsigned int pwm_counter2=0;
volatile unsigned int pwm_duty2=65535; //(0?65535)
volatile int direction=0;
volatile int peggingsidnatu=0;
#define TIMER2_RELOAD (0x10000L - (SYSCLK/(12L*PWM_FREQ)))
#define PWMOUT2 P3_2
#define PWMOUT2R P3_7

//////////////////SERVO pins//////////////////////
#define SERVO1   P1_3 // top servo
#define SERVO2   P1_4 // bottom servo
#define EMAGNET  P1_5 // magnet

volatile unsigned int servo_counter=0;
volatile unsigned char servo1=250, servo2=250;

	
/////////////////////////////////////////////////

//SlAVE FILE FOR EFM8 
//mommy farts
idata char buff[20];
idata char msg[20];
unsigned char overflow_count;

char _c51_external_startup (void)
{
	// Disable Watchdog with key sequence
	SFRPAGE = 0x00;
	WDTCN = 0xDE; //First key
	WDTCN = 0xAD; //Second key
  
	VDM0CN=0x80;       // enable VDD monitor
	RSTSRC=0x02|0x04;  // Enable reset on missing clock detector and VDD

	#if (SYSCLK == 48000000L)	
		SFRPAGE = 0x10;
		PFE0CN  = 0x10; // SYSCLK < 50 MHz.
		SFRPAGE = 0x00;
	#elif (SYSCLK == 72000000L)
		SFRPAGE = 0x10;
		PFE0CN  = 0x20; // SYSCLK < 75 MHz.
		SFRPAGE = 0x00;
	#endif
	
	#if (SYSCLK == 12250000L)
		CLKSEL = 0x10;
		CLKSEL = 0x10;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 24500000L)
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 48000000L)	
		// Before setting clock to 48 MHz, must transition to 24.5 MHz first
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
		CLKSEL = 0x07;
		CLKSEL = 0x07;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 72000000L)
		// Before setting clock to 72 MHz, must transition to 24.5 MHz first
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
		CLKSEL = 0x03;
		CLKSEL = 0x03;
		while ((CLKSEL & 0x80) == 0);
	#else
		#error SYSCLK must be either 12250000L, 24500000L, 48000000L, or 72000000L
	#endif
	
	P0MDOUT |= 0x11; // Enable UART0 TX (P0.4) and UART1 TX (P0.0) as push-pull outputs
	P2MDOUT |= 0x01; // P2.0 in push-pull mode
	XBR0     = 0x01; // Enable UART0 on P0.4(TX) and P0.5(RX)                     
	XBR1     = 0X00;
	XBR2     = 0x41; // Enable crossbar and uart 1

	P3MDOUT |= 0b10000101;
	XBR2     = 0x41; // existing line
	P3MDOUT |= 0b10000101;  // <== Add this

	// Configure Uart 0
	#if (((SYSCLK/BAUDRATE)/(2L*12L))>0xFFL)
		#error Timer 0 reload value is incorrect because (SYSCLK/BAUDRATE)/(2L*12L) > 0xFF
	#endif
	SCON0 = 0x10;
	TH1 = 0x100-((SYSCLK/BAUDRATE)/(2L*12L));
	TL1 = TH1;      // Init Timer1
	TMOD &= ~0xf0;  // TMOD: timer 1 in 8-bit auto-reload
	TMOD |=  0x20;                       
	TR1 = 1; // START Timer1
	TI = 1;  // Indicate TX0 ready
  	
	
	// Initialize timer 4 for periodic interrupts
	SFRPAGE=0x10;
	TMR4CN0=0x00;   // Stop Timer4; Clear TF4; WARNING: lives in SFR page 0x10
	CKCON1|=0b_0000_0001; // Timer 4 uses the system clock
	TMR4RL = TIMER4_RELOAD;
	TMR4=0xffff;   // Set to reload immediately
	EIE2|=0b_0000_0100;     // Enable Timer4 interrupts
	TR4=1;
	EA=1;
	
		// Initialize timer 2 for periodic interrupts
	TMR2CN0=0x00;   // Stop Timer2; Clear TF2;
	CKCON0|=0b_0001_0000; // Timer 2 uses the system clock
	TMR2RL=TIMER2_RELOAD; // Initialize reload value
	TMR2=0xffff;   // Set to reload immediately
	ET2=1;         // Enable Timer2 interrupts
	TR2=1;         // Start Timer2 (TMR2CN is bit addressable)
	SFRPAGE=0x00;
	
	// Initialize timer 5 for periodic interrupts
	SFRPAGE=0x10;
	TMR5CN0=0x00;
	TMR5=0xffff;   // Set to reload immediately
	EIE2|=0b_0000_1000; // Enable Timer5 interrupts
	TR5=1;         // Start Timer5 (TMR5CN0 is bit addressable)
	
	EA=1;
	
	SFRPAGE=0x00;
	

	return 0;
}

void Timer5_ISR (void) interrupt INTERRUPT_TIMER5
{
	SFRPAGE=0x10;
	TF5H = 0; // Clear Timer5 interrupt flag
	TMR5RL=(0x10000L-(SYSCLK/(12L*100000L)));
	servo_counter++;
	if(servo_counter==2000)
	{
		servo_counter=0;
	}
	if(servo1>=servo_counter)
	{
		SERVO1=1;
	}
	else
	{
		SERVO1=0;
	}
	if(servo2>=servo_counter)
	{
		SERVO2=1;
	}
	else
	{
		SERVO2=0;
	}
}

// Uses Timer3 to delay <us> micro-seconds. 
void Timer3us(unsigned char us)
{
	unsigned char i;               // usec counter
	
	// The input for Timer 3 is selected as SYSCLK by setting T3ML (bit 6) of CKCON0:
	CKCON0|=0b_0100_0000;
	
	TMR3RL = (-(SYSCLK)/1000000L); // Set Timer3 to overflow in 1us.
	TMR3 = TMR3RL;                 // Initialize Timer3 for first overflow
	
	TMR3CN0 = 0x04;                 // Sart Timer3 and clear overflow flag
	for (i = 0; i < us; i++)       // Count <us> overflows
	{
		while (!(TMR3CN0 & 0x80));  // Wait for overflow
		TMR3CN0 &= ~(0x80);         // Clear overflow indicator
	}
	TMR3CN0 = 0 ;                   // Stop Timer3 and clear overflow flag
}


void waitms (unsigned int ms)
{
	unsigned int j;
	unsigned char k;
	for(j=0; j<ms; j++)
		for (k=0; k<4; k++) Timer3us(250);
}

void UART1_Init (unsigned long baudrate)
{
    SFRPAGE = 0x20;
	SMOD1 = 0x0C; // no parity, 8 data bits, 1 stop bit
	SCON1 = 0x10;
	SBCON1 =0x00;   // disable baud rate generator
	SBRL1 = 0x10000L-((SYSCLK/baudrate)/(12L*2L));
	TI1 = 1; // indicate ready for TX
	SBCON1 |= 0x40;   // enable baud rate generator
	SFRPAGE = 0x00;
}

void putchar1 (char c) 
{
    SFRPAGE = 0x20;
	while (!TI1);
	TI1=0;
	SBUF1 = c;
	SFRPAGE = 0x00;
}

void sendstr1 (char * s)
{
	while(*s)
	{
		putchar1(*s);
		s++;	
	}
}

char getchar1 (void)
{
	char c;
    SFRPAGE = 0x20;
	while (!RI1);
	RI1=0;
	// Clear Overrun and Parity error flags 
	SCON1&=0b_0011_1111;
	c = SBUF1;
	SFRPAGE = 0x00;
	return (c);
}

char getchar1_with_timeout (void)
{
	char c;
	unsigned int timeout;
    SFRPAGE = 0x20;
    timeout=0;
	while (!RI1)
	{
		SFRPAGE = 0x00;
		Timer3us(20);
		SFRPAGE = 0x20;
		timeout++;
		if(timeout==25000)
		{
			SFRPAGE = 0x00;
			return ('\n'); // Timeout after half second
		}
	}
	RI1=0;
	// Clear Overrun and Parity error flags 
	SCON1&=0b_0011_1111;
	c = SBUF1;
	SFRPAGE = 0x00;
	return (c);
}

void getstr1 (char * s, unsigned char n)
{
	char c;
	unsigned char cnt;
	
	cnt=0;
	while(1)
	{
		c=getchar1_with_timeout();
		if(c=='\n')
		{
			*s=0;
			return;
		}
		
		if (cnt<n)
		{
			cnt++;
			*s=c;
			s++;
		}
		else
		{
			*s=0;
			return;
		}
	}
}

// RXU1 returns '1' if there is a byte available in the receive buffer of UART1
bit RXU1 (void)
{
	bit mybit;
    SFRPAGE = 0x20;
	mybit=RI1;
	SFRPAGE = 0x00;
	return mybit;
}

void waitms_or_RI1 (unsigned int ms)
{
	unsigned int j;
	unsigned char k;
	for(j=0; j<ms; j++)
	{
		for (k=0; k<4; k++)
		{
			if(RXU1()) return;
			Timer3us(250);
		}
	}
}

void SendATCommand (char * s)
{
	printf("Command: %s", s);
	P2_0=0; // 'set' pin to 0 is 'AT' mode.
	waitms(5);
	sendstr1(s);
	getstr1(buff, sizeof(buff)-1);
	waitms(10);
	P2_0=1; // 'set' pin to 1 is normal operation mode.
	printf("Response: %s\r\n", buff);
}

void ReceptionOff (void)
{
	P2_0=0; // 'set' pin to 0 is 'AT' mode.
	waitms(10);
	sendstr1("AT+DVID0000\r\n"); // Some unused id, so that we get nothing in RXD1.
	waitms(10);
	// Clear Overrun and Parity error flags 
	SCON1&=0b_0011_1111;
	P2_0=1; // 'set' pin to 1 is normal operation mode.
}

// timer 0 initialization
void TIMER0_Init(void)
{
	TMOD&=0b_1111_0000; // Set the bits of Timer/Counter 0 to zero
	TMOD|=0b_0000_0001; // Timer/Counter 0 used as a 16-bit timer
	TR0=0; // Stop Timer/Counter 0
}

// INITIALIZING THE ADC PINS
void InitADC (void)
{
	SFRPAGE = 0x00;
	ADEN=0; // Disable ADC
	
	ADC0CN1=
		(0x2 << 6) | // 0x0: 10-bit, 0x1: 12-bit, 0x2: 14-bit
        (0x0 << 3) | // 0x0: No shift. 0x1: Shift right 1 bit. 0x2: Shift right 2 bits. 0x3: Shift right 3 bits.		
		(0x0 << 0) ; // Accumulate n conversions: 0x0: 1, 0x1:4, 0x2:8, 0x3:16, 0x4:32
	
	ADC0CF0=
	    ((SYSCLK/SARCLK) << 3) | // SAR Clock Divider. Max is 18MHz. Fsarclk = (Fadcclk) / (ADSC + 1)
		(0x0 << 2); // 0:SYSCLK ADCCLK = SYSCLK. 1:HFOSC0 ADCCLK = HFOSC0.
	
	ADC0CF1=
		(0 << 7)   | // 0: Disable low power mode. 1: Enable low power mode.
		(0x1E << 0); // Conversion Tracking Time. Tadtk = ADTK / (Fsarclk)
	
	ADC0CN0 =
		(0x0 << 7) | // ADEN. 0: Disable ADC0. 1: Enable ADC0.
		(0x0 << 6) | // IPOEN. 0: Keep ADC powered on when ADEN is 1. 1: Power down when ADC is idle.
		(0x0 << 5) | // ADINT. Set by hardware upon completion of a data conversion. Must be cleared by firmware.
		(0x0 << 4) | // ADBUSY. Writing 1 to this bit initiates an ADC conversion when ADCM = 000. This bit should not be polled to indicate when a conversion is complete. Instead, the ADINT bit should be used when polling for conversion completion.
		(0x0 << 3) | // ADWINT. Set by hardware when the contents of ADC0H:ADC0L fall within the window specified by ADC0GTH:ADC0GTL and ADC0LTH:ADC0LTL. Can trigger an interrupt. Must be cleared by firmware.
		(0x0 << 2) | // ADGN (Gain Control). 0x0: PGA gain=1. 0x1: PGA gain=0.75. 0x2: PGA gain=0.5. 0x3: PGA gain=0.25.
		(0x0 << 0) ; // TEMPE. 0: Disable the Temperature Sensor. 1: Enable the Temperature Sensor.

	ADC0CF2= 
		(0x0 << 7) | // GNDSL. 0: reference is the GND pin. 1: reference is the AGND pin.
		(0x1 << 5) | // REFSL. 0x0: VREF pin (external or on-chip). 0x1: VDD pin. 0x2: 1.8V. 0x3: internal voltage reference.
		(0x1F << 0); // ADPWR. Power Up Delay Time. Tpwrtime = ((4 * (ADPWR + 1)) + 2) / (Fadcclk)
	
	ADC0CN2 =
		(0x0 << 7) | // PACEN. 0x0: The ADC accumulator is over-written.  0x1: The ADC accumulator adds to results.
		(0x0 << 0) ; // ADCM. 0x0: ADBUSY, 0x1: TIMER0, 0x2: TIMER2, 0x3: TIMER3, 0x4: CNVSTR, 0x5: CEX5, 0x6: TIMER4, 0x7: TIMER5, 0x8: CLU0, 0x9: CLU1, 0xA: CLU2, 0xB: CLU3

	ADEN=1; // Enable ADC
}

void InitPinADC (unsigned char portno, unsigned char pinno)
{
	unsigned char mask;
	
	mask=1<<pinno;

	SFRPAGE = 0x20;
	switch (portno)
	{
		case 0:
			P0MDIN &= (~mask); // Set pin as analog input
			P0SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		case 1:
			P1MDIN &= (~mask); // Set pin as analog input
			P1SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		case 2:
			P2MDIN &= (~mask); // Set pin as analog input
			P2SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		default:
		break;
	}
	SFRPAGE = 0x00;
}

unsigned int ADC_at_Pin(unsigned char pin)
{
	ADC0MX = pin;   // Select input from pin
	ADINT = 0;
	ADBUSY = 1;     // Convert voltage at the pin
	while (!ADINT); // Wait for conversion to complete
	return (ADC0);
}

float Volts_at_Pin(unsigned char pin)
{
	 return ((ADC_at_Pin(pin)*VDD)/0b_0011_1111_1111_1111);
}

// loser timer setup for pwm signal ?
void Timer4_ISR (void) interrupt INTERRUPT_TIMER4
{
	SFRPAGE=0x10;
	TF4H = 0; 

	pwm_counter4 += 256; // counting steps
	if ( direction == 1) {
	PWMOUT4 = (pwm_counter4 < pwm_duty4) ? 1 : 0;
	}
	if ( direction  == 0 ) {
	PWMOUT4R = (pwm_counter4 < pwm_duty4) ? 1 : 0;
	}
	if ( direction == 3 ) 
	{
	}
}

void Timer2_ISR (void) interrupt INTERRUPT_TIMER2
{
	SFRPAGE=0x0;
	TF2H = 0; // Clear Timer2 interrupt flag
	pwm_counter2 += 256; // counting steps
	if ( direction == 1) {
	PWMOUT2 = (pwm_counter2 < pwm_duty2) ? 1 : 0; ////////////////////////change this to pwm_duty2 later on 
	}
	if (direction == 0 ){
	PWMOUT2R = (pwm_counter2 < pwm_duty2) ? 1 : 0;
	}
	if ( direction == 3 ) 
	{
	}
}


// Measure the period of a square signal at PERIOD_PIN
unsigned long GetPeriod (int n)
{
	unsigned int overflow_count;
	unsigned char i;
	
	TR0=0; // Stop Timer/Counter 0
	TMOD&=0b_1111_0000; // Set the bits of Timer/Counter 0 to zero
	TMOD|=0b_0000_0001; // Timer/Counter 0 used as a 16-bit timer

	// Reset the counter
	TR0=0;
	TL0=0; TH0=0; TF0=0; overflow_count=0;
	TR0=1;

		// Reset the counter
		TR0=0;
		TL0=0; TH0=0; TF0=0; overflow_count=0;
		TR0=1;
		while(PERIOD_PIN!=0) // Wait for the signal to be zero
		{
			if(TF0==1) // Did the 16-bit timer overflow?
			{
				TF0=0;
				overflow_count++;
				if(overflow_count==10) // If it overflows too many times assume no signal is present
				{
					TR0=0;
					return 0; // No signal
				}
			}
		}
		
		// Reset the counter
		TR0=0;
		TL0=0; TH0=0; TF0=0; overflow_count=0;
		TR0=1;
		while(PERIOD_PIN!=1) // Wait for the signal to be one
		{
			if(TF0==1) // Did the 16-bit timer overflow?
			{
				TF0=0;
				overflow_count++;
				if(overflow_count==10) // If it overflows too many times assume no signal is present
				{
					TR0=0;
					return 0; // No signal
				}
			}
		}
		
		// Reset the counter
		TR0=0;
		TL0=0; TH0=0; TF0=0; overflow_count=0;
		TR0=1; // Start the timer
		for(i=0; i<n; i++) // Measure the time of 'n' periods
		{
			while(PERIOD_PIN!=0) // Wait for the signal to be zero
			{
				if(TF0==1) // Did the 16-bit timer overflow?
				{
					TF0=0;
					overflow_count++;
				}
			}
			while(PERIOD_PIN!=1) // Wait for the signal to be one
			{
				if(TF0==1) // Did the 16-bit timer overflow?
				{
					TF0=0;
					overflow_count++;
				}
			}
		}
		TR0=0; // Stop timer 0, the 24-bit number [overflow_count-TH0-TL0] has the period in clock cycles!
	
	return (overflow_count*65536+TH0*256+TL0);
}

void eputs(char *String)
{	
	while(*String)
	{
		putchar(*String);
		String++;
	}
}

void PrintNumber(long int val, int Base, int digits)
{ 
	code const char HexDigit[]="0123456789ABCDEF";
	int j;
	#define NBITS 32
	xdata char buff[NBITS+1];
	buff[NBITS]=0;
	
	if(val<0)
	{
		putchar('-');
		val*=-1;
	}

	j=NBITS-1;
	while ( (val>0) | (digits>0) )
	{
		buff[j--]=HexDigit[val%Base];
		val/=Base;
		if(digits!=0) digits--;
	}
	eputs(&buff[j+1]);
}

unsigned long GetFrequency (long int c)
{
	long int f = 0;

	if(c>0)
	{
			f=(SYSCLK*200.0)/(c*12);
//			eputs(" f");
//			PrintNumber(pin, 10, 1);
//			eputs(" = ");
//			PrintNumber(f, 10, 7);
//			eputs("Hz");
	}
	
	else
	{
		eputs(" NO SIGNAL                     \r");
	}

	return f;
}


////////////////////////////////// LCD //////////////////////////////////////////

void LCD_pulse (void)
{
	LCD_E=1;
	Timer3us(40);
	LCD_E=0;
}

void LCD_byte (unsigned char x)
{
	// The accumulator in the C8051Fxxx is bit addressable!
	ACC=x; //Send high nible
	LCD_D7=ACC_7;
	LCD_D6=ACC_6;
	LCD_D5=ACC_5;
	LCD_D4=ACC_4;
	LCD_pulse();
	Timer3us(40);
	ACC=x; //Send low nible
	LCD_D7=ACC_3;
	LCD_D6=ACC_2;
	LCD_D5=ACC_1;
	LCD_D4=ACC_0;
	LCD_pulse();
}

void WriteData (unsigned char x)
{
	LCD_RS=1;
	LCD_byte(x);
	waitms(2);
}

void WriteCommand (unsigned char x)
{
	LCD_RS=0;
	LCD_byte(x);
	waitms(5);
}

void LCD_4BIT (void)
{
	LCD_E=0; // Resting state of LCD's enable is zero
	// LCD_RW=0; // We are only writing to the LCD in this program
	waitms(20);
	// First make sure the LCD is in 8-bit mode and then change to 4-bit mode
	WriteCommand(0x33);
	WriteCommand(0x33);
	WriteCommand(0x32); // Change to 4-bit mode

	// Configure the LCD
	WriteCommand(0x28);
	WriteCommand(0x0c);
	WriteCommand(0x01); // Clear screen command (takes some time)
	waitms(20); // Wait for clear screen command to finsih.
}

//////////////////////////////////


void servomotion(void)
{
	unsigned char j;
	int i;
	
	WriteCommand(0x40);  // Set CGRAM address
	for(i=0; i<8; i++) {
        
	 	WriteData(customMouth[i]);
	}
	
	WriteCommand(0x48);
	for(i=0; i<8; i++) {
        
	 	WriteData(customEye[i]);
	}
	
	WriteCommand(0x50);  // Set CGRAM address
	for(i=0; i<8; i++) {
        
	 	WriteData(customOpenMouth[i]);
	}
	
	WriteCommand(0x58);  // Set CGRAM address
	for(i=0; i<8; i++) {
        
	 	WriteData(customSparkle[i]);
	}
	
	WriteCommand(0x60);  // Set CGRAM address
	for(i=0; i<8; i++) {
        
	 	WriteData(customMoney[i]);
	}
	
	WriteCommand(0x68);  // Set CGRAM address
	for(i=0; i<8; i++) {
        
	 	WriteData(customHappyMouth[i]);
	}
	
	WriteCommand(0x83);
	WriteData(3);
			
	WriteCommand(0x85);
	WriteData(3);
			
	WriteCommand(0xC4);
	WriteData(2);
	
	waitms(500);
	servo1 = 150;
	waitms(100);
	
	P1_5 = 1;
	

	// sweeping motion of servo2
	for(j=250; j>180; j-=5) 
	{
		servo2 = j;
		waitms(20);
	}
	
	waitms(1000);
	
	// lift servo1 to lift up coin
	for(j=150; j<250; j+=5) 
	{
		servo1 = j;
		waitms(20);
	}
	
	waitms(1000);
	
	// move servo2 to bring arm over coin bucket
	for(j=180; j > 60; j-=5){
		servo2 = j;
		waitms(20);
	}

	// drop coin
	//waitms(1000);
	P1_5 = 0;
	
	waitms(150);
	servo1 = 250;
	servo2 = 250; 
	EMAGNET=0;
	
	WriteCommand(0x83);
	WriteData(1);
			
	WriteCommand(0x85);
	WriteData(1);
			
	WriteCommand(0xC4);
	WriteData(0);

	money_count++;

}
/*
automaticmode
move foward untill dectects per
*/

void automaticmode(float fowardper, float sideper, float freq)
{
	int control = 0;
	
	direction = 3;
	//move fowawrd
	P3_7=1;  //wheel 1
	P3_2=0;	// wheel 1 
	P3_0=0; // wheel 2
	P2_5=1; // wheel 2
	printf("%ld\n\r", freq);
		if ( freq >= 64000)  //100000    63750   65000
	{
		P3_7=0;  //wheel 1
		P3_2=1;	// wheel 1 
		P3_0=1; // wheel 2
		P2_5=0; // wheel 2
		waitms(150);
		P3_7=0;  //wheel 1
		P3_2=0;	// wheel 1 
		P3_0=0; // wheel 2
		P2_5=0; // wheel 2
	servomotion();
	}

	//foward per logic 
	if ( fowardper >= p_thresh)
	{
		P3_7=0;  //wheel 1
		P3_2=1;	// wheel 1 
		P3_0=1; // wheel 2
		P2_5=0; // wheel 2
		waitms(300);
		if ( peggingsidnatu == 0 )
		{
			P3_7=0;  //wheel 1
			P3_2=1;	// wheel 1 
			P3_0=0; // wheel 2
			P2_5=0; // wheel 2
			waitms(750);
			peggingsidnatu = 1;
			return;
			
		}
		if ( peggingsidnatu == 1 )
		{
			P3_7=0;  //wheel 1
			P3_2=0;	// wheel 1 
			P3_0=1; // wheel 2
			P2_5=0; // wheel 2
			waitms(750);
			peggingsidnatu = 0;
			return;
		}
	}
	
	
	if ( sideper >= p_thresh)
	{
		if ( peggingsidnatu == 0 )
		{
			P3_7=0;  //wheel 1
			P3_2=1;	// wheel 1 
			P3_0=0; // wheel 2
			P2_5=0; // wheel 2
			waitms(750);
			control = 1;
			return;
			
		}
		if ( peggingsidnatu == 1 )
		{
			P3_7=0;  //wheel 1
			P3_2=0;	// wheel 1 
			P3_0=1; // wheel 2
			P2_5=0; // wheel 2
			waitms(750);
			control = 0;
			return;
		}
	}
	
	

}

void main (void)
{
    //unsigned int evilcode, evilcode1;
	unsigned int timeout=10000;
	float pulse_width = 20;
	float pulse_width1 = 10;
	int adcwheel1, adcwheel2;
	int which;
 	//char c;

	// initialization for the period code
	long int count, f;
	// initialization for the perimeter code
	float v[2];

	// automatic mode
	//int automatic = 0;
	
	waitms(500);
	printf("\r\nEFM8LB12 JDY-40 Slave Test.\r\n");
	UART1_Init(9600);

	ReceptionOff();

	LCD_4BIT();

	TIMER0_Init(); 

	InitPinADC(2, 1); // Configure P2.1 as analog input
	InitPinADC(2, 3); // Configure P2.1 as analog input
	InitADC();


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
	
	P1_5 = 0;
	while(1)
	{	
		EMAGNET = 0;
		/* PERIOD CODE */
		count = GetPeriod(200);
		f = GetFrequency(count);
//		coinPresent = CoinDecider(f); 
//		if(coinPresent)
//		{
//			sprintf(msg, "%ld", f-55000); // subtracted so that it sends a smaller value
//			sendstr1(msg);
//		}

		/* PERIMETER CODE */
		v[0] = Volts_at_Pin(QFP32_MUX_P2_1);
		v[1] = Volts_at_Pin(QFP32_MUX_P2_3);
		
		
		// printing the voltage at the inductors (if perimeter is reached)
//		printf(" V_P2_1 = %f V_P2_3 = %f %ld\n\r", v[0], v[1], f);
		if(RXU1()) // Something has arrived
		{
		//agartha will rise again
			getstr1(buff, sizeof(buff));
			
			if ( strcmp(buff, "A") == 0 )
			{
				waitms(500);
				while(1)
				{
					waitms(5);
					direction=3; 
//					printf(" V_P2_1 = %f V_P2_3 = %f %ld\n\r", v[0], v[1], f);
								
					v[0] = Volts_at_Pin(QFP32_MUX_P2_1);
					v[1] = Volts_at_Pin(QFP32_MUX_P2_3);
					count = GetPeriod(200);
					f = GetFrequency(count);
					automaticmode(v[0], v[1], f);
			
				if(RXU1())
				{
					getstr1(buff, sizeof(buff));
					if (strcmp(buff, "A") == 0 ) break;
				}
					 
				}
				
					
			}
			
							if ( strcmp(buff, "S") == 0 )	
					{
					
						
						servomotion();
						
						printf("this should be the motor function");
						waitms(500);

					}	

		
			sscanf(buff, "K%uW%uG%d\n", &adcwheel1, &adcwheel2, &which);
		
			if (which == 0 )
			{ 
				P2_5 = 0;
				P3_7=0;
				direction = 1;
			}
			 else 
			{
			 P3_2=0;
			 P3_0=0;
			 direction = 0;
			 
			}
		
			pwm_duty4 = adcwheel2;
			if ( adcwheel1 == 5535 ) adcwheel1 = 65535;
			if ( adcwheel1 == 5086 ) adcwheel1 = 65535;
			if ( adcwheel1 == 535 ) adcwheel1 = 65535;
			if ( adcwheel1 == 86 ) adcwheel1 = 65535;
			pwm_duty2 = adcwheel1;

			
    //		printf("pwm_duty4 = %u pwm_duty3 = %u dir = %d \n", pwm_duty4, pwm_duty2, which);
	//	    printf("%ld\n\r", f);
	//		PrintNumber(pwm_duty2, 10, 6);
	//		eputs("\n\r");
			waitms(5); // The radio seems to need this delay...

		}


	}
}
