;--------------------------------------------------------
; File Created by C51
; Version 1.0.0 #1170 (Feb 16 2022) (MSVC)
; This file was generated Wed Mar 26 13:25:18 2025
;--------------------------------------------------------
$name EFM8_JDY40_test
$optc51 --model-small
	R_DSEG    segment data
	R_CSEG    segment code
	R_BSEG    segment bit
	R_XSEG    segment xdata
	R_PSEG    segment xdata
	R_ISEG    segment idata
	R_OSEG    segment data overlay
	BIT_BANK  segment data overlay
	R_HOME    segment code
	R_GSINIT  segment code
	R_IXSEG   segment xdata
	R_CONST   segment code
	R_XINIT   segment code
	R_DINIT   segment code

;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	public _InitPinADC_PARM_2
	public _PrintNumber_HexDigit_1_160
	public _main
	public _CoinDecider
	public _GetFrequency
	public _PrintNumber
	public _eputs
	public _GetPeriod
	public _ADCsteeringRatio
	public _ADCtoPWM
	public _Timer2_ISR
	public _Timer4_ISR
	public _Volts_at_Pin
	public _ADC_at_Pin
	public _InitPinADC
	public _InitADC
	public _TIMER0_Init
	public _ReceptionOff
	public _SendATCommand
	public _waitms_or_RI1
	public _RXU1
	public _getstr1
	public _getchar1_with_timeout
	public _getchar1
	public _sendstr1
	public _putchar1
	public _UART1_Init
	public _waitms
	public _Timer3us
	public __c51_external_startup
	public _msg
	public _buff
	public _GetFrequency_PARM_2
	public _PrintNumber_PARM_3
	public _PrintNumber_PARM_2
	public _ADCsteeringRatio_PARM_4
	public _ADCsteeringRatio_PARM_3
	public _ADCsteeringRatio_PARM_2
	public _getstr1_PARM_2
	public _overflow_count
	public _direction
	public _pwm_duty2
	public _pwm_counter2
	public _pwm_duty4
	public _pwm_counter4
;--------------------------------------------------------
; Special Function Registers
;--------------------------------------------------------
_ACC            DATA 0xe0
_ADC0ASAH       DATA 0xb6
_ADC0ASAL       DATA 0xb5
_ADC0ASCF       DATA 0xa1
_ADC0ASCT       DATA 0xc7
_ADC0CF0        DATA 0xbc
_ADC0CF1        DATA 0xb9
_ADC0CF2        DATA 0xdf
_ADC0CN0        DATA 0xe8
_ADC0CN1        DATA 0xb2
_ADC0CN2        DATA 0xb3
_ADC0GTH        DATA 0xc4
_ADC0GTL        DATA 0xc3
_ADC0H          DATA 0xbe
_ADC0L          DATA 0xbd
_ADC0LTH        DATA 0xc6
_ADC0LTL        DATA 0xc5
_ADC0MX         DATA 0xbb
_B              DATA 0xf0
_CKCON0         DATA 0x8e
_CKCON1         DATA 0xa6
_CLEN0          DATA 0xc6
_CLIE0          DATA 0xc7
_CLIF0          DATA 0xe8
_CLKSEL         DATA 0xa9
_CLOUT0         DATA 0xd1
_CLU0CF         DATA 0xb1
_CLU0FN         DATA 0xaf
_CLU0MX         DATA 0x84
_CLU1CF         DATA 0xb3
_CLU1FN         DATA 0xb2
_CLU1MX         DATA 0x85
_CLU2CF         DATA 0xb6
_CLU2FN         DATA 0xb5
_CLU2MX         DATA 0x91
_CLU3CF         DATA 0xbf
_CLU3FN         DATA 0xbe
_CLU3MX         DATA 0xae
_CMP0CN0        DATA 0x9b
_CMP0CN1        DATA 0x99
_CMP0MD         DATA 0x9d
_CMP0MX         DATA 0x9f
_CMP1CN0        DATA 0xbf
_CMP1CN1        DATA 0xac
_CMP1MD         DATA 0xab
_CMP1MX         DATA 0xaa
_CRC0CN0        DATA 0xce
_CRC0CN1        DATA 0x86
_CRC0CNT        DATA 0xd3
_CRC0DAT        DATA 0xcb
_CRC0FLIP       DATA 0xcf
_CRC0IN         DATA 0xca
_CRC0ST         DATA 0xd2
_DAC0CF0        DATA 0x91
_DAC0CF1        DATA 0x92
_DAC0H          DATA 0x85
_DAC0L          DATA 0x84
_DAC1CF0        DATA 0x93
_DAC1CF1        DATA 0x94
_DAC1H          DATA 0x8a
_DAC1L          DATA 0x89
_DAC2CF0        DATA 0x95
_DAC2CF1        DATA 0x96
_DAC2H          DATA 0x8c
_DAC2L          DATA 0x8b
_DAC3CF0        DATA 0x9a
_DAC3CF1        DATA 0x9c
_DAC3H          DATA 0x8e
_DAC3L          DATA 0x8d
_DACGCF0        DATA 0x88
_DACGCF1        DATA 0x98
_DACGCF2        DATA 0xa2
_DERIVID        DATA 0xad
_DEVICEID       DATA 0xb5
_DPH            DATA 0x83
_DPL            DATA 0x82
_EIE1           DATA 0xe6
_EIE2           DATA 0xf3
_EIP1           DATA 0xbb
_EIP1H          DATA 0xee
_EIP2           DATA 0xed
_EIP2H          DATA 0xf6
_EMI0CN         DATA 0xe7
_FLKEY          DATA 0xb7
_HFO0CAL        DATA 0xc7
_HFO1CAL        DATA 0xd6
_HFOCN          DATA 0xef
_I2C0ADM        DATA 0xff
_I2C0CN0        DATA 0xba
_I2C0DIN        DATA 0xbc
_I2C0DOUT       DATA 0xbb
_I2C0FCN0       DATA 0xad
_I2C0FCN1       DATA 0xab
_I2C0FCT        DATA 0xf5
_I2C0SLAD       DATA 0xbd
_I2C0STAT       DATA 0xb9
_IE             DATA 0xa8
_IP             DATA 0xb8
_IPH            DATA 0xf2
_IT01CF         DATA 0xe4
_LFO0CN         DATA 0xb1
_P0             DATA 0x80
_P0MASK         DATA 0xfe
_P0MAT          DATA 0xfd
_P0MDIN         DATA 0xf1
_P0MDOUT        DATA 0xa4
_P0SKIP         DATA 0xd4
_P1             DATA 0x90
_P1MASK         DATA 0xee
_P1MAT          DATA 0xed
_P1MDIN         DATA 0xf2
_P1MDOUT        DATA 0xa5
_P1SKIP         DATA 0xd5
_P2             DATA 0xa0
_P2MASK         DATA 0xfc
_P2MAT          DATA 0xfb
_P2MDIN         DATA 0xf3
_P2MDOUT        DATA 0xa6
_P2SKIP         DATA 0xcc
_P3             DATA 0xb0
_P3MDIN         DATA 0xf4
_P3MDOUT        DATA 0x9c
_PCA0CENT       DATA 0x9e
_PCA0CLR        DATA 0x9c
_PCA0CN0        DATA 0xd8
_PCA0CPH0       DATA 0xfc
_PCA0CPH1       DATA 0xea
_PCA0CPH2       DATA 0xec
_PCA0CPH3       DATA 0xf5
_PCA0CPH4       DATA 0x85
_PCA0CPH5       DATA 0xde
_PCA0CPL0       DATA 0xfb
_PCA0CPL1       DATA 0xe9
_PCA0CPL2       DATA 0xeb
_PCA0CPL3       DATA 0xf4
_PCA0CPL4       DATA 0x84
_PCA0CPL5       DATA 0xdd
_PCA0CPM0       DATA 0xda
_PCA0CPM1       DATA 0xdb
_PCA0CPM2       DATA 0xdc
_PCA0CPM3       DATA 0xae
_PCA0CPM4       DATA 0xaf
_PCA0CPM5       DATA 0xcc
_PCA0H          DATA 0xfa
_PCA0L          DATA 0xf9
_PCA0MD         DATA 0xd9
_PCA0POL        DATA 0x96
_PCA0PWM        DATA 0xf7
_PCON0          DATA 0x87
_PCON1          DATA 0xcd
_PFE0CN         DATA 0xc1
_PRTDRV         DATA 0xf6
_PSCTL          DATA 0x8f
_PSTAT0         DATA 0xaa
_PSW            DATA 0xd0
_REF0CN         DATA 0xd1
_REG0CN         DATA 0xc9
_REVID          DATA 0xb6
_RSTSRC         DATA 0xef
_SBCON1         DATA 0x94
_SBRLH1         DATA 0x96
_SBRLL1         DATA 0x95
_SBUF           DATA 0x99
_SBUF0          DATA 0x99
_SBUF1          DATA 0x92
_SCON           DATA 0x98
_SCON0          DATA 0x98
_SCON1          DATA 0xc8
_SFRPAGE        DATA 0xa7
_SFRPGCN        DATA 0xbc
_SFRSTACK       DATA 0xd7
_SMB0ADM        DATA 0xd6
_SMB0ADR        DATA 0xd7
_SMB0CF         DATA 0xc1
_SMB0CN0        DATA 0xc0
_SMB0DAT        DATA 0xc2
_SMB0FCN0       DATA 0xc3
_SMB0FCN1       DATA 0xc4
_SMB0FCT        DATA 0xef
_SMB0RXLN       DATA 0xc5
_SMB0TC         DATA 0xac
_SMOD1          DATA 0x93
_SP             DATA 0x81
_SPI0CFG        DATA 0xa1
_SPI0CKR        DATA 0xa2
_SPI0CN0        DATA 0xf8
_SPI0DAT        DATA 0xa3
_SPI0FCN0       DATA 0x9a
_SPI0FCN1       DATA 0x9b
_SPI0FCT        DATA 0xf7
_SPI0PCF        DATA 0xdf
_TCON           DATA 0x88
_TH0            DATA 0x8c
_TH1            DATA 0x8d
_TL0            DATA 0x8a
_TL1            DATA 0x8b
_TMOD           DATA 0x89
_TMR2CN0        DATA 0xc8
_TMR2CN1        DATA 0xfd
_TMR2H          DATA 0xcf
_TMR2L          DATA 0xce
_TMR2RLH        DATA 0xcb
_TMR2RLL        DATA 0xca
_TMR3CN0        DATA 0x91
_TMR3CN1        DATA 0xfe
_TMR3H          DATA 0x95
_TMR3L          DATA 0x94
_TMR3RLH        DATA 0x93
_TMR3RLL        DATA 0x92
_TMR4CN0        DATA 0x98
_TMR4CN1        DATA 0xff
_TMR4H          DATA 0xa5
_TMR4L          DATA 0xa4
_TMR4RLH        DATA 0xa3
_TMR4RLL        DATA 0xa2
_TMR5CN0        DATA 0xc0
_TMR5CN1        DATA 0xf1
_TMR5H          DATA 0xd5
_TMR5L          DATA 0xd4
_TMR5RLH        DATA 0xd3
_TMR5RLL        DATA 0xd2
_UART0PCF       DATA 0xd9
_UART1FCN0      DATA 0x9d
_UART1FCN1      DATA 0xd8
_UART1FCT       DATA 0xfa
_UART1LIN       DATA 0x9e
_UART1PCF       DATA 0xda
_VDM0CN         DATA 0xff
_WDTCN          DATA 0x97
_XBR0           DATA 0xe1
_XBR1           DATA 0xe2
_XBR2           DATA 0xe3
_XOSC0CN        DATA 0x86
_DPTR           DATA 0x8382
_TMR2RL         DATA 0xcbca
_TMR3RL         DATA 0x9392
_TMR4RL         DATA 0xa3a2
_TMR5RL         DATA 0xd3d2
_TMR0           DATA 0x8c8a
_TMR1           DATA 0x8d8b
_TMR2           DATA 0xcfce
_TMR3           DATA 0x9594
_TMR4           DATA 0xa5a4
_TMR5           DATA 0xd5d4
_SBRL1          DATA 0x9695
_PCA0           DATA 0xfaf9
_PCA0CP0        DATA 0xfcfb
_PCA0CP1        DATA 0xeae9
_PCA0CP2        DATA 0xeceb
_PCA0CP3        DATA 0xf5f4
_PCA0CP4        DATA 0x8584
_PCA0CP5        DATA 0xdedd
_ADC0ASA        DATA 0xb6b5
_ADC0GT         DATA 0xc4c3
_ADC0           DATA 0xbebd
_ADC0LT         DATA 0xc6c5
_DAC0           DATA 0x8584
_DAC1           DATA 0x8a89
_DAC2           DATA 0x8c8b
_DAC3           DATA 0x8e8d
;--------------------------------------------------------
; special function bits
;--------------------------------------------------------
_ACC_0          BIT 0xe0
_ACC_1          BIT 0xe1
_ACC_2          BIT 0xe2
_ACC_3          BIT 0xe3
_ACC_4          BIT 0xe4
_ACC_5          BIT 0xe5
_ACC_6          BIT 0xe6
_ACC_7          BIT 0xe7
_TEMPE          BIT 0xe8
_ADGN0          BIT 0xe9
_ADGN1          BIT 0xea
_ADWINT         BIT 0xeb
_ADBUSY         BIT 0xec
_ADINT          BIT 0xed
_IPOEN          BIT 0xee
_ADEN           BIT 0xef
_B_0            BIT 0xf0
_B_1            BIT 0xf1
_B_2            BIT 0xf2
_B_3            BIT 0xf3
_B_4            BIT 0xf4
_B_5            BIT 0xf5
_B_6            BIT 0xf6
_B_7            BIT 0xf7
_C0FIF          BIT 0xe8
_C0RIF          BIT 0xe9
_C1FIF          BIT 0xea
_C1RIF          BIT 0xeb
_C2FIF          BIT 0xec
_C2RIF          BIT 0xed
_C3FIF          BIT 0xee
_C3RIF          BIT 0xef
_D1SRC0         BIT 0x88
_D1SRC1         BIT 0x89
_D1AMEN         BIT 0x8a
_D01REFSL       BIT 0x8b
_D3SRC0         BIT 0x8c
_D3SRC1         BIT 0x8d
_D3AMEN         BIT 0x8e
_D23REFSL       BIT 0x8f
_D0UDIS         BIT 0x98
_D1UDIS         BIT 0x99
_D2UDIS         BIT 0x9a
_D3UDIS         BIT 0x9b
_EX0            BIT 0xa8
_ET0            BIT 0xa9
_EX1            BIT 0xaa
_ET1            BIT 0xab
_ES0            BIT 0xac
_ET2            BIT 0xad
_ESPI0          BIT 0xae
_EA             BIT 0xaf
_PX0            BIT 0xb8
_PT0            BIT 0xb9
_PX1            BIT 0xba
_PT1            BIT 0xbb
_PS0            BIT 0xbc
_PT2            BIT 0xbd
_PSPI0          BIT 0xbe
_P0_0           BIT 0x80
_P0_1           BIT 0x81
_P0_2           BIT 0x82
_P0_3           BIT 0x83
_P0_4           BIT 0x84
_P0_5           BIT 0x85
_P0_6           BIT 0x86
_P0_7           BIT 0x87
_P1_0           BIT 0x90
_P1_1           BIT 0x91
_P1_2           BIT 0x92
_P1_3           BIT 0x93
_P1_4           BIT 0x94
_P1_5           BIT 0x95
_P1_6           BIT 0x96
_P1_7           BIT 0x97
_P2_0           BIT 0xa0
_P2_1           BIT 0xa1
_P2_2           BIT 0xa2
_P2_3           BIT 0xa3
_P2_4           BIT 0xa4
_P2_5           BIT 0xa5
_P2_6           BIT 0xa6
_P3_0           BIT 0xb0
_P3_1           BIT 0xb1
_P3_2           BIT 0xb2
_P3_3           BIT 0xb3
_P3_4           BIT 0xb4
_P3_7           BIT 0xb7
_CCF0           BIT 0xd8
_CCF1           BIT 0xd9
_CCF2           BIT 0xda
_CCF3           BIT 0xdb
_CCF4           BIT 0xdc
_CCF5           BIT 0xdd
_CR             BIT 0xde
_CF             BIT 0xdf
_PARITY         BIT 0xd0
_F1             BIT 0xd1
_OV             BIT 0xd2
_RS0            BIT 0xd3
_RS1            BIT 0xd4
_F0             BIT 0xd5
_AC             BIT 0xd6
_CY             BIT 0xd7
_RI             BIT 0x98
_TI             BIT 0x99
_RB8            BIT 0x9a
_TB8            BIT 0x9b
_REN            BIT 0x9c
_CE             BIT 0x9d
_SMODE          BIT 0x9e
_RI1            BIT 0xc8
_TI1            BIT 0xc9
_RBX1           BIT 0xca
_TBX1           BIT 0xcb
_REN1           BIT 0xcc
_PERR1          BIT 0xcd
_OVR1           BIT 0xce
_SI             BIT 0xc0
_ACK            BIT 0xc1
_ARBLOST        BIT 0xc2
_ACKRQ          BIT 0xc3
_STO            BIT 0xc4
_STA            BIT 0xc5
_TXMODE         BIT 0xc6
_MASTER         BIT 0xc7
_SPIEN          BIT 0xf8
_TXNF           BIT 0xf9
_NSSMD0         BIT 0xfa
_NSSMD1         BIT 0xfb
_RXOVRN         BIT 0xfc
_MODF           BIT 0xfd
_WCOL           BIT 0xfe
_SPIF           BIT 0xff
_IT0            BIT 0x88
_IE0            BIT 0x89
_IT1            BIT 0x8a
_IE1            BIT 0x8b
_TR0            BIT 0x8c
_TF0            BIT 0x8d
_TR1            BIT 0x8e
_TF1            BIT 0x8f
_T2XCLK0        BIT 0xc8
_T2XCLK1        BIT 0xc9
_TR2            BIT 0xca
_T2SPLIT        BIT 0xcb
_TF2CEN         BIT 0xcc
_TF2LEN         BIT 0xcd
_TF2L           BIT 0xce
_TF2H           BIT 0xcf
_T4XCLK0        BIT 0x98
_T4XCLK1        BIT 0x99
_TR4            BIT 0x9a
_T4SPLIT        BIT 0x9b
_TF4CEN         BIT 0x9c
_TF4LEN         BIT 0x9d
_TF4L           BIT 0x9e
_TF4H           BIT 0x9f
_T5XCLK0        BIT 0xc0
_T5XCLK1        BIT 0xc1
_TR5            BIT 0xc2
_T5SPLIT        BIT 0xc3
_TF5CEN         BIT 0xc4
_TF5LEN         BIT 0xc5
_TF5L           BIT 0xc6
_TF5H           BIT 0xc7
_RIE            BIT 0xd8
_RXTO0          BIT 0xd9
_RXTO1          BIT 0xda
_RFRQ           BIT 0xdb
_TIE            BIT 0xdc
_TXHOLD         BIT 0xdd
_TXNF1          BIT 0xde
_TFRQ           BIT 0xdf
;--------------------------------------------------------
; overlayable register banks
;--------------------------------------------------------
	rbank0 segment data overlay
;--------------------------------------------------------
; internal ram data
;--------------------------------------------------------
	rseg R_DSEG
_pwm_counter4:
	ds 2
_pwm_duty4:
	ds 2
_pwm_counter2:
	ds 2
_pwm_duty2:
	ds 2
_direction:
	ds 2
_overflow_count:
	ds 1
_getstr1_PARM_2:
	ds 1
_ADCsteeringRatio_PARM_2:
	ds 2
_ADCsteeringRatio_PARM_3:
	ds 3
_ADCsteeringRatio_PARM_4:
	ds 3
_ADCsteeringRatio_steeringFactor_1_141:
	ds 4
_PrintNumber_PARM_2:
	ds 2
_PrintNumber_PARM_3:
	ds 2
_PrintNumber_val_1_159:
	ds 4
_PrintNumber_j_1_160:
	ds 2
_PrintNumber_sloc0_1_0:
	ds 2
_PrintNumber_sloc1_1_0:
	ds 4
_GetFrequency_PARM_2:
	ds 2
_main_speed_1_175:
	ds 2
_main_steering_1_175:
	ds 2
_main_adcwheel1_1_175:
	ds 2
_main_adcwheel2_1_175:
	ds 2
_main_v_1_175:
	ds 8
;--------------------------------------------------------
; overlayable items in internal ram 
;--------------------------------------------------------
	rseg	R_OSEG
	rseg	R_OSEG
	rseg	R_OSEG
	rseg	R_OSEG
_InitPinADC_PARM_2:
	ds 1
	rseg	R_OSEG
	rseg	R_OSEG
;--------------------------------------------------------
; indirectly addressable internal ram data
;--------------------------------------------------------
	rseg R_ISEG
_buff:
	ds 20
_msg:
	ds 20
;--------------------------------------------------------
; absolute internal ram data
;--------------------------------------------------------
	DSEG
;--------------------------------------------------------
; bit data
;--------------------------------------------------------
	rseg R_BSEG
;--------------------------------------------------------
; paged external ram data
;--------------------------------------------------------
	rseg R_PSEG
;--------------------------------------------------------
; external ram data
;--------------------------------------------------------
	rseg R_XSEG
_PrintNumber_buff_1_160:
	ds 33
;--------------------------------------------------------
; absolute external ram data
;--------------------------------------------------------
	XSEG
;--------------------------------------------------------
; external initialized ram data
;--------------------------------------------------------
	rseg R_IXSEG
	rseg R_HOME
	rseg R_GSINIT
	rseg R_CSEG
;--------------------------------------------------------
; Reset entry point and interrupt vectors
;--------------------------------------------------------
	CSEG at 0x0000
	ljmp	_crt0
	CSEG at 0x002b
	ljmp	_Timer2_ISR
	CSEG at 0x008b
	ljmp	_Timer4_ISR
;--------------------------------------------------------
; global & static initialisations
;--------------------------------------------------------
	rseg R_HOME
	rseg R_GSINIT
	rseg R_GSINIT
;--------------------------------------------------------
; data variables initialization
;--------------------------------------------------------
	rseg R_DINIT
;	EFM8_JDY40_test.c:21: volatile unsigned int pwm_counter4=0;
	clr	a
	mov	_pwm_counter4,a
	mov	(_pwm_counter4 + 1),a
;	EFM8_JDY40_test.c:22: volatile unsigned int pwm_duty4=65535; //(0�65535)
	mov	_pwm_duty4,#0xFF
	mov	(_pwm_duty4 + 1),#0xFF
;	EFM8_JDY40_test.c:28: volatile unsigned int pwm_counter2=0;
	clr	a
	mov	_pwm_counter2,a
	mov	(_pwm_counter2 + 1),a
;	EFM8_JDY40_test.c:29: volatile unsigned int pwm_duty2=65535; //(0�65535)
	mov	_pwm_duty2,#0xFF
	mov	(_pwm_duty2 + 1),#0xFF
;	EFM8_JDY40_test.c:30: volatile int direction=0;
	clr	a
	mov	_direction,a
	mov	(_direction + 1),a
	; The linker places a 'ret' at the end of segment R_DINIT.
;--------------------------------------------------------
; code
;--------------------------------------------------------
	rseg R_CSEG
;------------------------------------------------------------
;Allocation info for local variables in function '_c51_external_startup'
;------------------------------------------------------------
;------------------------------------------------------------
;	EFM8_JDY40_test.c:42: char _c51_external_startup (void)
;	-----------------------------------------
;	 function _c51_external_startup
;	-----------------------------------------
__c51_external_startup:
	using	0
;	EFM8_JDY40_test.c:45: SFRPAGE = 0x00;
	mov	_SFRPAGE,#0x00
;	EFM8_JDY40_test.c:46: WDTCN = 0xDE; //First key
	mov	_WDTCN,#0xDE
;	EFM8_JDY40_test.c:47: WDTCN = 0xAD; //Second key
	mov	_WDTCN,#0xAD
;	EFM8_JDY40_test.c:49: VDM0CN=0x80;       // enable VDD monitor
	mov	_VDM0CN,#0x80
;	EFM8_JDY40_test.c:50: RSTSRC=0x02|0x04;  // Enable reset on missing clock detector and VDD
	mov	_RSTSRC,#0x06
;	EFM8_JDY40_test.c:57: SFRPAGE = 0x10;
	mov	_SFRPAGE,#0x10
;	EFM8_JDY40_test.c:58: PFE0CN  = 0x20; // SYSCLK < 75 MHz.
	mov	_PFE0CN,#0x20
;	EFM8_JDY40_test.c:59: SFRPAGE = 0x00;
	mov	_SFRPAGE,#0x00
;	EFM8_JDY40_test.c:80: CLKSEL = 0x00;
	mov	_CLKSEL,#0x00
;	EFM8_JDY40_test.c:81: CLKSEL = 0x00;
	mov	_CLKSEL,#0x00
;	EFM8_JDY40_test.c:82: while ((CLKSEL & 0x80) == 0);
L002001?:
	mov	a,_CLKSEL
	jnb	acc.7,L002001?
;	EFM8_JDY40_test.c:83: CLKSEL = 0x03;
	mov	_CLKSEL,#0x03
;	EFM8_JDY40_test.c:84: CLKSEL = 0x03;
	mov	_CLKSEL,#0x03
;	EFM8_JDY40_test.c:85: while ((CLKSEL & 0x80) == 0);
L002004?:
	mov	a,_CLKSEL
	jnb	acc.7,L002004?
;	EFM8_JDY40_test.c:90: P0MDOUT |= 0x11; // Enable UART0 TX (P0.4) and UART1 TX (P0.0) as push-pull outputs
	orl	_P0MDOUT,#0x11
;	EFM8_JDY40_test.c:91: P2MDOUT |= 0x01; // P2.0 in push-pull mode
	orl	_P2MDOUT,#0x01
;	EFM8_JDY40_test.c:92: XBR0     = 0x01; // Enable UART0 on P0.4(TX) and P0.5(RX)                     
	mov	_XBR0,#0x01
;	EFM8_JDY40_test.c:93: XBR1     = 0X00;
	mov	_XBR1,#0x00
;	EFM8_JDY40_test.c:94: XBR2     = 0x41; // Enable crossbar and uart 1
	mov	_XBR2,#0x41
;	EFM8_JDY40_test.c:96: P3MDOUT |= 0b10000101;
	orl	_P3MDOUT,#0x85
;	EFM8_JDY40_test.c:97: XBR2     = 0x41; // existing line
	mov	_XBR2,#0x41
;	EFM8_JDY40_test.c:98: P3MDOUT |= 0b10000101;  // <== Add this
	orl	_P3MDOUT,#0x85
;	EFM8_JDY40_test.c:104: SCON0 = 0x10;
	mov	_SCON0,#0x10
;	EFM8_JDY40_test.c:105: TH1 = 0x100-((SYSCLK/BAUDRATE)/(2L*12L));
	mov	_TH1,#0xE6
;	EFM8_JDY40_test.c:106: TL1 = TH1;      // Init Timer1
	mov	_TL1,_TH1
;	EFM8_JDY40_test.c:107: TMOD &= ~0xf0;  // TMOD: timer 1 in 8-bit auto-reload
	anl	_TMOD,#0x0F
;	EFM8_JDY40_test.c:108: TMOD |=  0x20;                       
	orl	_TMOD,#0x20
;	EFM8_JDY40_test.c:109: TR1 = 1; // START Timer1
	setb	_TR1
;	EFM8_JDY40_test.c:110: TI = 1;  // Indicate TX0 ready
	setb	_TI
;	EFM8_JDY40_test.c:114: SFRPAGE=0x10;
	mov	_SFRPAGE,#0x10
;	EFM8_JDY40_test.c:115: TMR4CN0=0x00;   // Stop Timer4; Clear TF4; WARNING: lives in SFR page 0x10
	mov	_TMR4CN0,#0x00
;	EFM8_JDY40_test.c:116: CKCON1|=0b_0000_0001; // Timer 4 uses the system clock
	orl	_CKCON1,#0x01
;	EFM8_JDY40_test.c:117: TMR4RL = TIMER4_RELOAD;
	mov	_TMR4RL,#0xA8
	mov	(_TMR4RL >> 8),#0xFD
;	EFM8_JDY40_test.c:118: TMR4=0xffff;   // Set to reload immediately
	mov	_TMR4,#0xFF
	mov	(_TMR4 >> 8),#0xFF
;	EFM8_JDY40_test.c:119: EIE2|=0b_0000_0100;     // Enable Timer4 interrupts
	orl	_EIE2,#0x04
;	EFM8_JDY40_test.c:120: TR4=1;
	setb	_TR4
;	EFM8_JDY40_test.c:121: EA=1;
	setb	_EA
;	EFM8_JDY40_test.c:124: TMR2CN0=0x00;   // Stop Timer2; Clear TF2;
	mov	_TMR2CN0,#0x00
;	EFM8_JDY40_test.c:125: CKCON0|=0b_0001_0000; // Timer 2 uses the system clock
	orl	_CKCON0,#0x10
;	EFM8_JDY40_test.c:126: TMR2RL=TIMER2_RELOAD; // Initialize reload value
	mov	_TMR2RL,#0xA8
	mov	(_TMR2RL >> 8),#0xFD
;	EFM8_JDY40_test.c:127: TMR2=0xffff;   // Set to reload immediately
	mov	_TMR2,#0xFF
	mov	(_TMR2 >> 8),#0xFF
;	EFM8_JDY40_test.c:128: ET2=1;         // Enable Timer2 interrupts
	setb	_ET2
;	EFM8_JDY40_test.c:129: TR2=1;         // Start Timer2 (TMR2CN is bit addressable)
	setb	_TR2
;	EFM8_JDY40_test.c:130: SFRPAGE=0x00;
	mov	_SFRPAGE,#0x00
;	EFM8_JDY40_test.c:131: return 0;
	mov	dpl,#0x00
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'Timer3us'
;------------------------------------------------------------
;us                        Allocated to registers r2 
;i                         Allocated to registers r3 
;------------------------------------------------------------
;	EFM8_JDY40_test.c:135: void Timer3us(unsigned char us)
;	-----------------------------------------
;	 function Timer3us
;	-----------------------------------------
_Timer3us:
	mov	r2,dpl
;	EFM8_JDY40_test.c:140: CKCON0|=0b_0100_0000;
	orl	_CKCON0,#0x40
;	EFM8_JDY40_test.c:142: TMR3RL = (-(SYSCLK)/1000000L); // Set Timer3 to overflow in 1us.
	mov	_TMR3RL,#0xB8
	mov	(_TMR3RL >> 8),#0xFF
;	EFM8_JDY40_test.c:143: TMR3 = TMR3RL;                 // Initialize Timer3 for first overflow
	mov	_TMR3,_TMR3RL
	mov	(_TMR3 >> 8),(_TMR3RL >> 8)
;	EFM8_JDY40_test.c:145: TMR3CN0 = 0x04;                 // Sart Timer3 and clear overflow flag
	mov	_TMR3CN0,#0x04
;	EFM8_JDY40_test.c:146: for (i = 0; i < us; i++)       // Count <us> overflows
	mov	r3,#0x00
L003004?:
	clr	c
	mov	a,r3
	subb	a,r2
	jnc	L003007?
;	EFM8_JDY40_test.c:148: while (!(TMR3CN0 & 0x80));  // Wait for overflow
L003001?:
	mov	a,_TMR3CN0
	jnb	acc.7,L003001?
;	EFM8_JDY40_test.c:149: TMR3CN0 &= ~(0x80);         // Clear overflow indicator
	anl	_TMR3CN0,#0x7F
;	EFM8_JDY40_test.c:146: for (i = 0; i < us; i++)       // Count <us> overflows
	inc	r3
	sjmp	L003004?
L003007?:
;	EFM8_JDY40_test.c:151: TMR3CN0 = 0 ;                   // Stop Timer3 and clear overflow flag
	mov	_TMR3CN0,#0x00
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'waitms'
;------------------------------------------------------------
;ms                        Allocated to registers r2 r3 
;j                         Allocated to registers r4 r5 
;k                         Allocated to registers r6 
;------------------------------------------------------------
;	EFM8_JDY40_test.c:154: void waitms (unsigned int ms)
;	-----------------------------------------
;	 function waitms
;	-----------------------------------------
_waitms:
	mov	r2,dpl
	mov	r3,dph
;	EFM8_JDY40_test.c:158: for(j=0; j<ms; j++)
	mov	r4,#0x00
	mov	r5,#0x00
L004005?:
	clr	c
	mov	a,r4
	subb	a,r2
	mov	a,r5
	subb	a,r3
	jnc	L004009?
;	EFM8_JDY40_test.c:159: for (k=0; k<4; k++) Timer3us(250);
	mov	r6,#0x00
L004001?:
	cjne	r6,#0x04,L004018?
L004018?:
	jnc	L004007?
	mov	dpl,#0xFA
	push	ar2
	push	ar3
	push	ar4
	push	ar5
	push	ar6
	lcall	_Timer3us
	pop	ar6
	pop	ar5
	pop	ar4
	pop	ar3
	pop	ar2
	inc	r6
	sjmp	L004001?
L004007?:
;	EFM8_JDY40_test.c:158: for(j=0; j<ms; j++)
	inc	r4
	cjne	r4,#0x00,L004005?
	inc	r5
	sjmp	L004005?
L004009?:
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'UART1_Init'
;------------------------------------------------------------
;baudrate                  Allocated to registers r2 r3 r4 r5 
;------------------------------------------------------------
;	EFM8_JDY40_test.c:162: void UART1_Init (unsigned long baudrate)
;	-----------------------------------------
;	 function UART1_Init
;	-----------------------------------------
_UART1_Init:
	mov	r2,dpl
	mov	r3,dph
	mov	r4,b
	mov	r5,a
;	EFM8_JDY40_test.c:164: SFRPAGE = 0x20;
	mov	_SFRPAGE,#0x20
;	EFM8_JDY40_test.c:165: SMOD1 = 0x0C; // no parity, 8 data bits, 1 stop bit
	mov	_SMOD1,#0x0C
;	EFM8_JDY40_test.c:166: SCON1 = 0x10;
	mov	_SCON1,#0x10
;	EFM8_JDY40_test.c:167: SBCON1 =0x00;   // disable baud rate generator
	mov	_SBCON1,#0x00
;	EFM8_JDY40_test.c:168: SBRL1 = 0x10000L-((SYSCLK/baudrate)/(12L*2L));
	mov	__divulong_PARM_2,r2
	mov	(__divulong_PARM_2 + 1),r3
	mov	(__divulong_PARM_2 + 2),r4
	mov	(__divulong_PARM_2 + 3),r5
	mov	dptr,#0xA200
	mov	b,#0x4A
	mov	a,#0x04
	lcall	__divulong
	mov	r2,dpl
	mov	r3,dph
	mov	r4,b
	mov	r5,a
	mov	__divulong_PARM_2,#0x18
	clr	a
	mov	(__divulong_PARM_2 + 1),a
	mov	(__divulong_PARM_2 + 2),a
	mov	(__divulong_PARM_2 + 3),a
	mov	dpl,r2
	mov	dph,r3
	mov	b,r4
	mov	a,r5
	lcall	__divulong
	mov	r2,dpl
	mov	r3,dph
	mov	r4,b
	mov	r5,a
	clr	a
	clr	c
	subb	a,r2
	mov	r2,a
	clr	a
	subb	a,r3
	mov	r3,a
	mov	a,#0x01
	subb	a,r4
	clr	a
	subb	a,r5
	mov	_SBRL1,r2
	mov	(_SBRL1 >> 8),r3
;	EFM8_JDY40_test.c:169: TI1 = 1; // indicate ready for TX
	setb	_TI1
;	EFM8_JDY40_test.c:170: SBCON1 |= 0x40;   // enable baud rate generator
	orl	_SBCON1,#0x40
;	EFM8_JDY40_test.c:171: SFRPAGE = 0x00;
	mov	_SFRPAGE,#0x00
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'putchar1'
;------------------------------------------------------------
;c                         Allocated to registers r2 
;------------------------------------------------------------
;	EFM8_JDY40_test.c:174: void putchar1 (char c) 
;	-----------------------------------------
;	 function putchar1
;	-----------------------------------------
_putchar1:
	mov	r2,dpl
;	EFM8_JDY40_test.c:176: SFRPAGE = 0x20;
	mov	_SFRPAGE,#0x20
;	EFM8_JDY40_test.c:177: while (!TI1);
L006001?:
;	EFM8_JDY40_test.c:178: TI1=0;
	jbc	_TI1,L006008?
	sjmp	L006001?
L006008?:
;	EFM8_JDY40_test.c:179: SBUF1 = c;
	mov	_SBUF1,r2
;	EFM8_JDY40_test.c:180: SFRPAGE = 0x00;
	mov	_SFRPAGE,#0x00
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'sendstr1'
;------------------------------------------------------------
;s                         Allocated to registers r2 r3 r4 
;------------------------------------------------------------
;	EFM8_JDY40_test.c:183: void sendstr1 (char * s)
;	-----------------------------------------
;	 function sendstr1
;	-----------------------------------------
_sendstr1:
	mov	r2,dpl
	mov	r3,dph
	mov	r4,b
;	EFM8_JDY40_test.c:185: while(*s)
L007001?:
	mov	dpl,r2
	mov	dph,r3
	mov	b,r4
	lcall	__gptrget
	mov	r5,a
	jz	L007004?
;	EFM8_JDY40_test.c:187: putchar1(*s);
	mov	dpl,r5
	push	ar2
	push	ar3
	push	ar4
	lcall	_putchar1
	pop	ar4
	pop	ar3
	pop	ar2
;	EFM8_JDY40_test.c:188: s++;	
	inc	r2
	cjne	r2,#0x00,L007001?
	inc	r3
	sjmp	L007001?
L007004?:
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'getchar1'
;------------------------------------------------------------
;c                         Allocated to registers 
;------------------------------------------------------------
;	EFM8_JDY40_test.c:192: char getchar1 (void)
;	-----------------------------------------
;	 function getchar1
;	-----------------------------------------
_getchar1:
;	EFM8_JDY40_test.c:195: SFRPAGE = 0x20;
	mov	_SFRPAGE,#0x20
;	EFM8_JDY40_test.c:196: while (!RI1);
L008001?:
;	EFM8_JDY40_test.c:197: RI1=0;
	jbc	_RI1,L008008?
	sjmp	L008001?
L008008?:
;	EFM8_JDY40_test.c:199: SCON1&=0b_0011_1111;
	anl	_SCON1,#0x3F
;	EFM8_JDY40_test.c:200: c = SBUF1;
	mov	dpl,_SBUF1
;	EFM8_JDY40_test.c:201: SFRPAGE = 0x00;
	mov	_SFRPAGE,#0x00
;	EFM8_JDY40_test.c:202: return (c);
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'getchar1_with_timeout'
;------------------------------------------------------------
;c                         Allocated to registers 
;timeout                   Allocated to registers r2 r3 
;------------------------------------------------------------
;	EFM8_JDY40_test.c:205: char getchar1_with_timeout (void)
;	-----------------------------------------
;	 function getchar1_with_timeout
;	-----------------------------------------
_getchar1_with_timeout:
;	EFM8_JDY40_test.c:209: SFRPAGE = 0x20;
	mov	_SFRPAGE,#0x20
;	EFM8_JDY40_test.c:211: while (!RI1)
	mov	r2,#0x00
	mov	r3,#0x00
L009003?:
	jb	_RI1,L009005?
;	EFM8_JDY40_test.c:213: SFRPAGE = 0x00;
	mov	_SFRPAGE,#0x00
;	EFM8_JDY40_test.c:214: Timer3us(20);
	mov	dpl,#0x14
	push	ar2
	push	ar3
	lcall	_Timer3us
	pop	ar3
	pop	ar2
;	EFM8_JDY40_test.c:215: SFRPAGE = 0x20;
	mov	_SFRPAGE,#0x20
;	EFM8_JDY40_test.c:216: timeout++;
	inc	r2
	cjne	r2,#0x00,L009012?
	inc	r3
L009012?:
;	EFM8_JDY40_test.c:217: if(timeout==25000)
	cjne	r2,#0xA8,L009003?
	cjne	r3,#0x61,L009003?
;	EFM8_JDY40_test.c:219: SFRPAGE = 0x00;
	mov	_SFRPAGE,#0x00
;	EFM8_JDY40_test.c:220: return ('\n'); // Timeout after half second
	mov	dpl,#0x0A
	ret
L009005?:
;	EFM8_JDY40_test.c:223: RI1=0;
	clr	_RI1
;	EFM8_JDY40_test.c:225: SCON1&=0b_0011_1111;
	anl	_SCON1,#0x3F
;	EFM8_JDY40_test.c:226: c = SBUF1;
	mov	dpl,_SBUF1
;	EFM8_JDY40_test.c:227: SFRPAGE = 0x00;
	mov	_SFRPAGE,#0x00
;	EFM8_JDY40_test.c:228: return (c);
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'getstr1'
;------------------------------------------------------------
;n                         Allocated with name '_getstr1_PARM_2'
;s                         Allocated to registers r2 r3 r4 
;c                         Allocated to registers r1 
;cnt                       Allocated to registers r5 
;------------------------------------------------------------
;	EFM8_JDY40_test.c:231: void getstr1 (char * s, unsigned char n)
;	-----------------------------------------
;	 function getstr1
;	-----------------------------------------
_getstr1:
	mov	r2,dpl
	mov	r3,dph
	mov	r4,b
;	EFM8_JDY40_test.c:237: while(1)
	mov	r5,#0x00
	mov	ar6,r2
	mov	ar7,r3
	mov	ar0,r4
L010007?:
;	EFM8_JDY40_test.c:239: c=getchar1_with_timeout();
	push	ar2
	push	ar3
	push	ar4
	push	ar5
	push	ar6
	push	ar7
	push	ar0
	lcall	_getchar1_with_timeout
	mov	r1,dpl
	pop	ar0
	pop	ar7
	pop	ar6
	pop	ar5
	pop	ar4
	pop	ar3
	pop	ar2
;	EFM8_JDY40_test.c:240: if(c=='\n')
	cjne	r1,#0x0A,L010002?
;	EFM8_JDY40_test.c:242: *s=0;
	mov	dpl,r2
	mov	dph,r3
	mov	b,r4
	clr	a
;	EFM8_JDY40_test.c:243: return;
	ljmp	__gptrput
L010002?:
;	EFM8_JDY40_test.c:246: if (cnt<n)
	clr	c
	mov	a,r5
	subb	a,_getstr1_PARM_2
	jnc	L010004?
;	EFM8_JDY40_test.c:248: cnt++;
	inc	r5
;	EFM8_JDY40_test.c:249: *s=c;
	mov	dpl,r6
	mov	dph,r7
	mov	b,r0
	mov	a,r1
	lcall	__gptrput
	inc	dptr
	mov	r6,dpl
	mov	r7,dph
;	EFM8_JDY40_test.c:250: s++;
	mov	ar2,r6
	mov	ar3,r7
	mov	ar4,r0
	sjmp	L010007?
L010004?:
;	EFM8_JDY40_test.c:254: *s=0;
	mov	dpl,r2
	mov	dph,r3
	mov	b,r4
	clr	a
;	EFM8_JDY40_test.c:255: return;
	ljmp	__gptrput
;------------------------------------------------------------
;Allocation info for local variables in function 'RXU1'
;------------------------------------------------------------
;------------------------------------------------------------
;	EFM8_JDY40_test.c:261: bit RXU1 (void)
;	-----------------------------------------
;	 function RXU1
;	-----------------------------------------
_RXU1:
;	EFM8_JDY40_test.c:264: SFRPAGE = 0x20;
	mov	_SFRPAGE,#0x20
;	EFM8_JDY40_test.c:265: mybit=RI1;
	mov	c,_RI1
;	EFM8_JDY40_test.c:266: SFRPAGE = 0x00;
	mov	_SFRPAGE,#0x00
;	EFM8_JDY40_test.c:267: return mybit;
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'waitms_or_RI1'
;------------------------------------------------------------
;ms                        Allocated to registers r2 r3 
;j                         Allocated to registers r4 r5 
;k                         Allocated to registers r6 
;------------------------------------------------------------
;	EFM8_JDY40_test.c:270: void waitms_or_RI1 (unsigned int ms)
;	-----------------------------------------
;	 function waitms_or_RI1
;	-----------------------------------------
_waitms_or_RI1:
	mov	r2,dpl
	mov	r3,dph
;	EFM8_JDY40_test.c:274: for(j=0; j<ms; j++)
	mov	r4,#0x00
	mov	r5,#0x00
L012007?:
	clr	c
	mov	a,r4
	subb	a,r2
	mov	a,r5
	subb	a,r3
	jnc	L012011?
;	EFM8_JDY40_test.c:276: for (k=0; k<4; k++)
	mov	r6,#0x00
L012003?:
	cjne	r6,#0x04,L012019?
L012019?:
	jnc	L012009?
;	EFM8_JDY40_test.c:278: if(RXU1()) return;
	push	ar2
	push	ar3
	push	ar4
	push	ar5
	push	ar6
	lcall	_RXU1
	clr	a
	rlc	a
	pop	ar6
	pop	ar5
	pop	ar4
	pop	ar3
	pop	ar2
	jz	L012002?
	ret
L012002?:
;	EFM8_JDY40_test.c:279: Timer3us(250);
	mov	dpl,#0xFA
	push	ar2
	push	ar3
	push	ar4
	push	ar5
	push	ar6
	lcall	_Timer3us
	pop	ar6
	pop	ar5
	pop	ar4
	pop	ar3
	pop	ar2
;	EFM8_JDY40_test.c:276: for (k=0; k<4; k++)
	inc	r6
	sjmp	L012003?
L012009?:
;	EFM8_JDY40_test.c:274: for(j=0; j<ms; j++)
	inc	r4
	cjne	r4,#0x00,L012007?
	inc	r5
	sjmp	L012007?
L012011?:
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'SendATCommand'
;------------------------------------------------------------
;s                         Allocated to registers r2 r3 r4 
;------------------------------------------------------------
;	EFM8_JDY40_test.c:284: void SendATCommand (char * s)
;	-----------------------------------------
;	 function SendATCommand
;	-----------------------------------------
_SendATCommand:
	mov	r2,dpl
	mov	r3,dph
	mov	r4,b
;	EFM8_JDY40_test.c:286: printf("Command: %s", s);
	push	ar2
	push	ar3
	push	ar4
	push	ar2
	push	ar3
	push	ar4
	mov	a,#__str_0
	push	acc
	mov	a,#(__str_0 >> 8)
	push	acc
	mov	a,#0x80
	push	acc
	lcall	_printf
	mov	a,sp
	add	a,#0xfa
	mov	sp,a
;	EFM8_JDY40_test.c:287: P2_0=0; // 'set' pin to 0 is 'AT' mode.
	clr	_P2_0
;	EFM8_JDY40_test.c:288: waitms(5);
	mov	dptr,#0x0005
	lcall	_waitms
	pop	ar4
	pop	ar3
	pop	ar2
;	EFM8_JDY40_test.c:289: sendstr1(s);
	mov	dpl,r2
	mov	dph,r3
	mov	b,r4
	lcall	_sendstr1
;	EFM8_JDY40_test.c:290: getstr1(buff, sizeof(buff)-1);
	mov	_getstr1_PARM_2,#0x13
	mov	dptr,#_buff
	mov	b,#0x40
	lcall	_getstr1
;	EFM8_JDY40_test.c:291: waitms(10);
	mov	dptr,#0x000A
	lcall	_waitms
;	EFM8_JDY40_test.c:292: P2_0=1; // 'set' pin to 1 is normal operation mode.
	setb	_P2_0
;	EFM8_JDY40_test.c:293: printf("Response: %s\r\n", buff);
	mov	a,#_buff
	push	acc
	mov	a,#(_buff >> 8)
	push	acc
	mov	a,#0x40
	push	acc
	mov	a,#__str_1
	push	acc
	mov	a,#(__str_1 >> 8)
	push	acc
	mov	a,#0x80
	push	acc
	lcall	_printf
	mov	a,sp
	add	a,#0xfa
	mov	sp,a
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'ReceptionOff'
;------------------------------------------------------------
;------------------------------------------------------------
;	EFM8_JDY40_test.c:296: void ReceptionOff (void)
;	-----------------------------------------
;	 function ReceptionOff
;	-----------------------------------------
_ReceptionOff:
;	EFM8_JDY40_test.c:298: P2_0=0; // 'set' pin to 0 is 'AT' mode.
	clr	_P2_0
;	EFM8_JDY40_test.c:299: waitms(10);
	mov	dptr,#0x000A
	lcall	_waitms
;	EFM8_JDY40_test.c:300: sendstr1("AT+DVID0000\r\n"); // Some unused id, so that we get nothing in RXD1.
	mov	dptr,#__str_2
	mov	b,#0x80
	lcall	_sendstr1
;	EFM8_JDY40_test.c:301: waitms(10);
	mov	dptr,#0x000A
	lcall	_waitms
;	EFM8_JDY40_test.c:303: SCON1&=0b_0011_1111;
	anl	_SCON1,#0x3F
;	EFM8_JDY40_test.c:304: P2_0=1; // 'set' pin to 1 is normal operation mode.
	setb	_P2_0
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'TIMER0_Init'
;------------------------------------------------------------
;------------------------------------------------------------
;	EFM8_JDY40_test.c:308: void TIMER0_Init(void)
;	-----------------------------------------
;	 function TIMER0_Init
;	-----------------------------------------
_TIMER0_Init:
;	EFM8_JDY40_test.c:310: TMOD&=0b_1111_0000; // Set the bits of Timer/Counter 0 to zero
	anl	_TMOD,#0xF0
;	EFM8_JDY40_test.c:311: TMOD|=0b_0000_0001; // Timer/Counter 0 used as a 16-bit timer
	orl	_TMOD,#0x01
;	EFM8_JDY40_test.c:312: TR0=0; // Stop Timer/Counter 0
	clr	_TR0
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'InitADC'
;------------------------------------------------------------
;------------------------------------------------------------
;	EFM8_JDY40_test.c:316: void InitADC (void)
;	-----------------------------------------
;	 function InitADC
;	-----------------------------------------
_InitADC:
;	EFM8_JDY40_test.c:318: SFRPAGE = 0x00;
	mov	_SFRPAGE,#0x00
;	EFM8_JDY40_test.c:319: ADEN=0; // Disable ADC
	clr	_ADEN
;	EFM8_JDY40_test.c:324: (0x0 << 0) ; // Accumulate n conversions: 0x0: 1, 0x1:4, 0x2:8, 0x3:16, 0x4:32
	mov	_ADC0CN1,#0x80
;	EFM8_JDY40_test.c:328: (0x0 << 2); // 0:SYSCLK ADCCLK = SYSCLK. 1:HFOSC0 ADCCLK = HFOSC0.
	mov	_ADC0CF0,#0x20
;	EFM8_JDY40_test.c:332: (0x1E << 0); // Conversion Tracking Time. Tadtk = ADTK / (Fsarclk)
	mov	_ADC0CF1,#0x1E
;	EFM8_JDY40_test.c:341: (0x0 << 0) ; // TEMPE. 0: Disable the Temperature Sensor. 1: Enable the Temperature Sensor.
	mov	_ADC0CN0,#0x00
;	EFM8_JDY40_test.c:346: (0x1F << 0); // ADPWR. Power Up Delay Time. Tpwrtime = ((4 * (ADPWR + 1)) + 2) / (Fadcclk)
	mov	_ADC0CF2,#0x3F
;	EFM8_JDY40_test.c:350: (0x0 << 0) ; // ADCM. 0x0: ADBUSY, 0x1: TIMER0, 0x2: TIMER2, 0x3: TIMER3, 0x4: CNVSTR, 0x5: CEX5, 0x6: TIMER4, 0x7: TIMER5, 0x8: CLU0, 0x9: CLU1, 0xA: CLU2, 0xB: CLU3
	mov	_ADC0CN2,#0x00
;	EFM8_JDY40_test.c:352: ADEN=1; // Enable ADC
	setb	_ADEN
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'InitPinADC'
;------------------------------------------------------------
;pinno                     Allocated with name '_InitPinADC_PARM_2'
;portno                    Allocated to registers r2 
;mask                      Allocated to registers r3 
;------------------------------------------------------------
;	EFM8_JDY40_test.c:355: void InitPinADC (unsigned char portno, unsigned char pinno)
;	-----------------------------------------
;	 function InitPinADC
;	-----------------------------------------
_InitPinADC:
	mov	r2,dpl
;	EFM8_JDY40_test.c:359: mask=1<<pinno;
	mov	b,_InitPinADC_PARM_2
	inc	b
	mov	a,#0x01
	sjmp	L017013?
L017011?:
	add	a,acc
L017013?:
	djnz	b,L017011?
	mov	r3,a
;	EFM8_JDY40_test.c:361: SFRPAGE = 0x20;
	mov	_SFRPAGE,#0x20
;	EFM8_JDY40_test.c:362: switch (portno)
	cjne	r2,#0x00,L017014?
	sjmp	L017001?
L017014?:
	cjne	r2,#0x01,L017015?
	sjmp	L017002?
L017015?:
;	EFM8_JDY40_test.c:364: case 0:
	cjne	r2,#0x02,L017005?
	sjmp	L017003?
L017001?:
;	EFM8_JDY40_test.c:365: P0MDIN &= (~mask); // Set pin as analog input
	mov	a,r3
	cpl	a
	mov	r2,a
	anl	_P0MDIN,a
;	EFM8_JDY40_test.c:366: P0SKIP |= mask; // Skip Crossbar decoding for this pin
	mov	a,r3
	orl	_P0SKIP,a
;	EFM8_JDY40_test.c:367: break;
;	EFM8_JDY40_test.c:368: case 1:
	sjmp	L017005?
L017002?:
;	EFM8_JDY40_test.c:369: P1MDIN &= (~mask); // Set pin as analog input
	mov	a,r3
	cpl	a
	mov	r2,a
	anl	_P1MDIN,a
;	EFM8_JDY40_test.c:370: P1SKIP |= mask; // Skip Crossbar decoding for this pin
	mov	a,r3
	orl	_P1SKIP,a
;	EFM8_JDY40_test.c:371: break;
;	EFM8_JDY40_test.c:372: case 2:
	sjmp	L017005?
L017003?:
;	EFM8_JDY40_test.c:373: P2MDIN &= (~mask); // Set pin as analog input
	mov	a,r3
	cpl	a
	mov	r2,a
	anl	_P2MDIN,a
;	EFM8_JDY40_test.c:374: P2SKIP |= mask; // Skip Crossbar decoding for this pin
	mov	a,r3
	orl	_P2SKIP,a
;	EFM8_JDY40_test.c:378: }
L017005?:
;	EFM8_JDY40_test.c:379: SFRPAGE = 0x00;
	mov	_SFRPAGE,#0x00
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'ADC_at_Pin'
;------------------------------------------------------------
;pin                       Allocated to registers 
;------------------------------------------------------------
;	EFM8_JDY40_test.c:382: unsigned int ADC_at_Pin(unsigned char pin)
;	-----------------------------------------
;	 function ADC_at_Pin
;	-----------------------------------------
_ADC_at_Pin:
	mov	_ADC0MX,dpl
;	EFM8_JDY40_test.c:385: ADINT = 0;
	clr	_ADINT
;	EFM8_JDY40_test.c:386: ADBUSY = 1;     // Convert voltage at the pin
	setb	_ADBUSY
;	EFM8_JDY40_test.c:387: while (!ADINT); // Wait for conversion to complete
L018001?:
	jnb	_ADINT,L018001?
;	EFM8_JDY40_test.c:388: return (ADC0);
	mov	dpl,_ADC0
	mov	dph,(_ADC0 >> 8)
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'Volts_at_Pin'
;------------------------------------------------------------
;pin                       Allocated to registers r2 
;------------------------------------------------------------
;	EFM8_JDY40_test.c:391: float Volts_at_Pin(unsigned char pin)
;	-----------------------------------------
;	 function Volts_at_Pin
;	-----------------------------------------
_Volts_at_Pin:
;	EFM8_JDY40_test.c:393: return ((ADC_at_Pin(pin)*VDD)/0b_0011_1111_1111_1111);
	lcall	_ADC_at_Pin
	lcall	___uint2fs
	mov	r2,dpl
	mov	r3,dph
	mov	r4,b
	mov	r5,a
	push	ar2
	push	ar3
	push	ar4
	push	ar5
	mov	dptr,#0x6C8B
	mov	b,#0x53
	mov	a,#0x40
	lcall	___fsmul
	mov	r2,dpl
	mov	r3,dph
	mov	r4,b
	mov	r5,a
	mov	a,sp
	add	a,#0xfc
	mov	sp,a
	clr	a
	push	acc
	mov	a,#0xFC
	push	acc
	mov	a,#0x7F
	push	acc
	mov	a,#0x46
	push	acc
	mov	dpl,r2
	mov	dph,r3
	mov	b,r4
	mov	a,r5
	lcall	___fsdiv
	mov	r2,dpl
	mov	r3,dph
	mov	r4,b
	mov	r5,a
	mov	a,sp
	add	a,#0xfc
	mov	sp,a
	mov	dpl,r2
	mov	dph,r3
	mov	b,r4
	mov	a,r5
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'Timer4_ISR'
;------------------------------------------------------------
;------------------------------------------------------------
;	EFM8_JDY40_test.c:397: void Timer4_ISR (void) interrupt INTERRUPT_TIMER4
;	-----------------------------------------
;	 function Timer4_ISR
;	-----------------------------------------
_Timer4_ISR:
	push	acc
	push	psw
	mov	psw,#0x00
;	EFM8_JDY40_test.c:399: SFRPAGE=0x10;
	mov	_SFRPAGE,#0x10
;	EFM8_JDY40_test.c:400: TF4H = 0; 
	clr	_TF4H
;	EFM8_JDY40_test.c:402: pwm_counter4 += 256; // counting steps
	mov	_pwm_counter4,_pwm_counter4
	mov	a,#0x01
	add	a,(_pwm_counter4 + 1)
	mov	(_pwm_counter4 + 1),a
;	EFM8_JDY40_test.c:403: if ( direction == 1) {
	mov	a,#0x01
	cjne	a,_direction,L020009?
	clr	a
	cjne	a,(_direction + 1),L020009?
	sjmp	L020010?
L020009?:
	sjmp	L020002?
L020010?:
;	EFM8_JDY40_test.c:404: PWMOUT4 = (pwm_counter4 < pwm_duty4) ? 1 : 0;
	clr	c
	mov	a,_pwm_counter4
	subb	a,_pwm_duty4
	mov	a,(_pwm_counter4 + 1)
	subb	a,(_pwm_duty4 + 1)
	mov	_P3_0,c
L020002?:
;	EFM8_JDY40_test.c:406: if ( direction  == 0 ) {
	mov	a,_direction
	orl	a,(_direction + 1)
	jnz	L020005?
;	EFM8_JDY40_test.c:407: PWMOUT4R = (pwm_counter4 < pwm_duty4) ? 1 : 0;
	clr	c
	mov	a,_pwm_counter4
	subb	a,_pwm_duty4
	mov	a,(_pwm_counter4 + 1)
	subb	a,(_pwm_duty4 + 1)
	mov	_P2_5,c
L020005?:
	pop	psw
	pop	acc
	reti
;	eliminated unneeded push/pop dpl
;	eliminated unneeded push/pop dph
;	eliminated unneeded push/pop b
;------------------------------------------------------------
;Allocation info for local variables in function 'Timer2_ISR'
;------------------------------------------------------------
;------------------------------------------------------------
;	EFM8_JDY40_test.c:411: void Timer2_ISR (void) interrupt INTERRUPT_TIMER2
;	-----------------------------------------
;	 function Timer2_ISR
;	-----------------------------------------
_Timer2_ISR:
	push	acc
	push	psw
	mov	psw,#0x00
;	EFM8_JDY40_test.c:413: SFRPAGE=0x0;
	mov	_SFRPAGE,#0x00
;	EFM8_JDY40_test.c:414: TF2H = 0; // Clear Timer2 interrupt flag
	clr	_TF2H
;	EFM8_JDY40_test.c:415: pwm_counter2 += 256; // counting steps
	mov	_pwm_counter2,_pwm_counter2
	mov	a,#0x01
	add	a,(_pwm_counter2 + 1)
	mov	(_pwm_counter2 + 1),a
;	EFM8_JDY40_test.c:416: if ( direction == 1) {
	mov	a,#0x01
	cjne	a,_direction,L021009?
	clr	a
	cjne	a,(_direction + 1),L021009?
	sjmp	L021010?
L021009?:
	sjmp	L021002?
L021010?:
;	EFM8_JDY40_test.c:417: PWMOUT2 = (pwm_counter2 < pwm_duty2) ? 1 : 0; ////////////////////////change this to pwm_duty2 later on 
	clr	c
	mov	a,_pwm_counter2
	subb	a,_pwm_duty2
	mov	a,(_pwm_counter2 + 1)
	subb	a,(_pwm_duty2 + 1)
	mov	_P3_2,c
L021002?:
;	EFM8_JDY40_test.c:419: if (direction == 0 ){
	mov	a,_direction
	orl	a,(_direction + 1)
	jnz	L021005?
;	EFM8_JDY40_test.c:420: PWMOUT2R = (pwm_counter2 < pwm_duty2) ? 1 : 0;
	clr	c
	mov	a,_pwm_counter2
	subb	a,_pwm_duty2
	mov	a,(_pwm_counter2 + 1)
	subb	a,(_pwm_duty2 + 1)
	mov	_P3_7,c
L021005?:
	pop	psw
	pop	acc
	reti
;	eliminated unneeded push/pop dpl
;	eliminated unneeded push/pop dph
;	eliminated unneeded push/pop b
;------------------------------------------------------------
;Allocation info for local variables in function 'ADCtoPWM'
;------------------------------------------------------------
;adc_value                 Allocated to registers r2 r3 
;------------------------------------------------------------
;	EFM8_JDY40_test.c:424: unsigned int ADCtoPWM(int adc_value)
;	-----------------------------------------
;	 function ADCtoPWM
;	-----------------------------------------
_ADCtoPWM:
	mov	r2,dpl
	mov	r3,dph
;	EFM8_JDY40_test.c:429: return (unsigned int)((adc_value * 65535UL) / 1008UL);
	mov	__mullong_PARM_2,r2
	mov	a,r3
	mov	(__mullong_PARM_2 + 1),a
	rlc	a
	subb	a,acc
	mov	(__mullong_PARM_2 + 2),a
	mov	(__mullong_PARM_2 + 3),a
	mov	dptr,#0xFFFF
	clr	a
	mov	b,a
	lcall	__mullong
	mov	__divulong_PARM_2,#0xF0
	mov	(__divulong_PARM_2 + 1),#0x03
	mov	(__divulong_PARM_2 + 2),#0x00
	mov	(__divulong_PARM_2 + 3),#0x00
	ljmp	__divulong
;------------------------------------------------------------
;Allocation info for local variables in function 'ADCsteeringRatio'
;------------------------------------------------------------
;steering                  Allocated with name '_ADCsteeringRatio_PARM_2'
;ADCwheel1                 Allocated with name '_ADCsteeringRatio_PARM_3'
;ADCwheel2                 Allocated with name '_ADCsteeringRatio_PARM_4'
;speed                     Allocated to registers r2 r3 
;centersteering            Allocated to registers r4 r5 
;centerspeed               Allocated to registers 
;steeringFactor            Allocated with name '_ADCsteeringRatio_steeringFactor_1_141'
;baseSpeed                 Allocated to registers r6 r7 
;baseSteer                 Allocated to registers r0 r1 
;wheel1Speed               Allocated to registers r6 r7 
;wheel2Speed               Allocated to registers r2 r3 
;------------------------------------------------------------
;	EFM8_JDY40_test.c:437: void ADCsteeringRatio(int speed, int steering, int *ADCwheel1, int *ADCwheel2) 
;	-----------------------------------------
;	 function ADCsteeringRatio
;	-----------------------------------------
_ADCsteeringRatio:
	mov	r2,dpl
	mov	r3,dph
;	EFM8_JDY40_test.c:441: int centersteering = steering - 508;
	mov	a,_ADCsteeringRatio_PARM_2
	add	a,#0x04
	mov	r4,a
	mov	a,(_ADCsteeringRatio_PARM_2 + 1)
	addc	a,#0xfe
	mov	r5,a
;	EFM8_JDY40_test.c:449: centerspeed = speed - 504;
	mov	a,r2
	add	a,#0x08
	mov	dpl,a
	mov	a,r3
	addc	a,#0xfe
	mov	dph,a
;	EFM8_JDY40_test.c:452: baseSpeed = abs(centerspeed);
	push	ar2
	push	ar3
	push	ar4
	push	ar5
	lcall	_abs
	mov	r6,dpl
	mov	r7,dph
	pop	ar5
	pop	ar4
;	EFM8_JDY40_test.c:453: baseSteer = abs(centersteering);
	mov	dpl,r4
	mov	dph,r5
	push	ar4
	push	ar5
	push	ar6
	push	ar7
	lcall	_abs
	mov	r0,dpl
	mov	r1,dph
	pop	ar7
	pop	ar6
	pop	ar5
	pop	ar4
	pop	ar3
	pop	ar2
;	EFM8_JDY40_test.c:454: if ( baseSpeed < 5 && baseSteer < 5 ) 
	clr	c
	mov	a,r6
	subb	a,#0x05
	mov	a,r7
	xrl	a,#0x80
	subb	a,#0x80
	jnc	L023002?
	clr	c
	mov	a,r0
	subb	a,#0x05
	mov	a,r1
	xrl	a,#0x80
	subb	a,#0x80
	jnc	L023002?
;	EFM8_JDY40_test.c:456: *ADCwheel1 = 0;
	mov	r6,_ADCsteeringRatio_PARM_3
	mov	r7,(_ADCsteeringRatio_PARM_3 + 1)
	mov	r0,(_ADCsteeringRatio_PARM_3 + 2)
	mov	dpl,r6
	mov	dph,r7
	mov	b,r0
	clr	a
	lcall	__gptrput
	inc	dptr
	clr	a
	lcall	__gptrput
;	EFM8_JDY40_test.c:457: *ADCwheel2 = 0;
	mov	r6,_ADCsteeringRatio_PARM_4
	mov	r7,(_ADCsteeringRatio_PARM_4 + 1)
	mov	r0,(_ADCsteeringRatio_PARM_4 + 2)
	mov	dpl,r6
	mov	dph,r7
	mov	b,r0
	clr	a
	lcall	__gptrput
	inc	dptr
	clr	a
;	EFM8_JDY40_test.c:458: return;
	ljmp	__gptrput
L023002?:
;	EFM8_JDY40_test.c:462: steeringFactor = (float)centersteering / 508; // ranges from -1.0 (full left) to +1.0 (full right)
	mov	dpl,r4
	mov	dph,r5
	push	ar2
	push	ar3
	lcall	___sint2fs
	mov	r4,dpl
	mov	r5,dph
	mov	r6,b
	mov	r7,a
	clr	a
	push	acc
	push	acc
	mov	a,#0xFE
	push	acc
	mov	a,#0x43
	push	acc
	mov	dpl,r4
	mov	dph,r5
	mov	b,r6
	mov	a,r7
	lcall	___fsdiv
	mov	_ADCsteeringRatio_steeringFactor_1_141,dpl
	mov	(_ADCsteeringRatio_steeringFactor_1_141 + 1),dph
	mov	(_ADCsteeringRatio_steeringFactor_1_141 + 2),b
	mov	(_ADCsteeringRatio_steeringFactor_1_141 + 3),a
	mov	a,sp
	add	a,#0xfc
	mov	sp,a
;	EFM8_JDY40_test.c:464: if ( steeringFactor > 1 ) steeringFactor = 1;
	clr	a
	push	acc
	push	acc
	mov	a,#0x80
	push	acc
	mov	a,#0x3F
	push	acc
	mov	dpl,_ADCsteeringRatio_steeringFactor_1_141
	mov	dph,(_ADCsteeringRatio_steeringFactor_1_141 + 1)
	mov	b,(_ADCsteeringRatio_steeringFactor_1_141 + 2)
	mov	a,(_ADCsteeringRatio_steeringFactor_1_141 + 3)
	lcall	___fsgt
	mov	r0,dpl
	mov	a,sp
	add	a,#0xfc
	mov	sp,a
	pop	ar3
	pop	ar2
	mov	a,r0
	jz	L023005?
	mov	_ADCsteeringRatio_steeringFactor_1_141,#0x00
	mov	(_ADCsteeringRatio_steeringFactor_1_141 + 1),#0x00
	mov	(_ADCsteeringRatio_steeringFactor_1_141 + 2),#0x80
	mov	(_ADCsteeringRatio_steeringFactor_1_141 + 3),#0x3F
L023005?:
;	EFM8_JDY40_test.c:468: wheel1Speed = speed + (int)(speed * steeringFactor);
	mov	dpl,r2
	mov	dph,r3
	push	ar2
	push	ar3
	lcall	___sint2fs
	mov	r0,dpl
	mov	r1,dph
	mov	r4,b
	mov	r5,a
	push	_ADCsteeringRatio_steeringFactor_1_141
	push	(_ADCsteeringRatio_steeringFactor_1_141 + 1)
	push	(_ADCsteeringRatio_steeringFactor_1_141 + 2)
	push	(_ADCsteeringRatio_steeringFactor_1_141 + 3)
	mov	dpl,r0
	mov	dph,r1
	mov	b,r4
	mov	a,r5
	lcall	___fsmul
	mov	r4,dpl
	mov	r5,dph
	mov	r6,b
	mov	r7,a
	mov	a,sp
	add	a,#0xfc
	mov	sp,a
	mov	dpl,r4
	mov	dph,r5
	mov	b,r6
	mov	a,r7
	lcall	___fs2sint
	mov	r4,dpl
	mov	r5,dph
	pop	ar3
	pop	ar2
	mov	a,r4
	add	a,r2
	mov	r6,a
	mov	a,r5
	addc	a,r3
	mov	r7,a
;	EFM8_JDY40_test.c:469: wheel2Speed = speed - (int)(speed * steeringFactor);
	mov	a,r2
	clr	c
	subb	a,r4
	mov	r2,a
	mov	a,r3
	subb	a,r5
	mov	r3,a
;	EFM8_JDY40_test.c:470: if (wheel1Speed > 1008) wheel1Speed = 1008;
	clr	c
	mov	a,#0xF0
	subb	a,r6
	mov	a,#(0x03 ^ 0x80)
	mov	b,r7
	xrl	b,#0x80
	subb	a,b
	jnc	L023007?
	mov	r6,#0xF0
	mov	r7,#0x03
L023007?:
;	EFM8_JDY40_test.c:471: if (wheel1Speed < 0) wheel1Speed = 0;
	mov	a,r7
	jnb	acc.7,L023009?
	mov	r6,#0x00
	mov	r7,#0x00
L023009?:
;	EFM8_JDY40_test.c:473: if (wheel2Speed > 1008) wheel2Speed = 1008;
	clr	c
	mov	a,#0xF0
	subb	a,r2
	mov	a,#(0x03 ^ 0x80)
	mov	b,r3
	xrl	b,#0x80
	subb	a,b
	jnc	L023011?
	mov	r2,#0xF0
	mov	r3,#0x03
L023011?:
;	EFM8_JDY40_test.c:474: if (wheel2Speed < 0) wheel2Speed = 0;
	mov	a,r3
	jnb	acc.7,L023013?
	mov	r2,#0x00
	mov	r3,#0x00
L023013?:
;	EFM8_JDY40_test.c:476: *ADCwheel1 = (unsigned int)(wheel1Speed);
	mov	r4,_ADCsteeringRatio_PARM_3
	mov	r5,(_ADCsteeringRatio_PARM_3 + 1)
	mov	r0,(_ADCsteeringRatio_PARM_3 + 2)
	mov	dpl,r4
	mov	dph,r5
	mov	b,r0
	mov	a,r6
	lcall	__gptrput
	inc	dptr
	mov	a,r7
	lcall	__gptrput
;	EFM8_JDY40_test.c:477: *ADCwheel2 = (unsigned int)(wheel2Speed);	
	mov	r4,_ADCsteeringRatio_PARM_4
	mov	r5,(_ADCsteeringRatio_PARM_4 + 1)
	mov	r6,(_ADCsteeringRatio_PARM_4 + 2)
	mov	dpl,r4
	mov	dph,r5
	mov	b,r6
	mov	a,r2
	lcall	__gptrput
	inc	dptr
	mov	a,r3
	ljmp	__gptrput
;------------------------------------------------------------
;Allocation info for local variables in function 'GetPeriod'
;------------------------------------------------------------
;n                         Allocated to registers r2 r3 
;overflow_count            Allocated to registers r4 r5 
;i                         Allocated to registers r6 
;------------------------------------------------------------
;	EFM8_JDY40_test.c:480: unsigned long GetPeriod (int n)
;	-----------------------------------------
;	 function GetPeriod
;	-----------------------------------------
_GetPeriod:
	mov	r2,dpl
	mov	r3,dph
;	EFM8_JDY40_test.c:485: TR0=0; // Stop Timer/Counter 0
	clr	_TR0
;	EFM8_JDY40_test.c:486: TMOD&=0b_1111_0000; // Set the bits of Timer/Counter 0 to zero
	anl	_TMOD,#0xF0
;	EFM8_JDY40_test.c:487: TMOD|=0b_0000_0001; // Timer/Counter 0 used as a 16-bit timer
	orl	_TMOD,#0x01
;	EFM8_JDY40_test.c:490: TR0=0;
	clr	_TR0
;	EFM8_JDY40_test.c:491: TL0=0; TH0=0; TF0=0; overflow_count=0;
	mov	_TL0,#0x00
	mov	_TH0,#0x00
	clr	_TF0
;	EFM8_JDY40_test.c:492: TR0=1;
	setb	_TR0
;	EFM8_JDY40_test.c:495: TR0=0;
	clr	_TR0
;	EFM8_JDY40_test.c:496: TL0=0; TH0=0; TF0=0; overflow_count=0;
	mov	_TL0,#0x00
	mov	_TH0,#0x00
	clr	_TF0
;	EFM8_JDY40_test.c:497: TR0=1;
	setb	_TR0
;	EFM8_JDY40_test.c:498: while(PERIOD_PIN!=0) // Wait for the signal to be zero
	mov	r4,#0x00
	mov	r5,#0x00
L024005?:
	jnb	_P2_6,L024007?
;	EFM8_JDY40_test.c:500: if(TF0==1) // Did the 16-bit timer overflow?
;	EFM8_JDY40_test.c:502: TF0=0;
	jbc	_TF0,L024050?
	sjmp	L024005?
L024050?:
;	EFM8_JDY40_test.c:503: overflow_count++;
	inc	r4
	cjne	r4,#0x00,L024051?
	inc	r5
L024051?:
;	EFM8_JDY40_test.c:504: if(overflow_count==10) // If it overflows too many times assume no signal is present
	cjne	r4,#0x0A,L024005?
	cjne	r5,#0x00,L024005?
;	EFM8_JDY40_test.c:506: TR0=0;
	clr	_TR0
;	EFM8_JDY40_test.c:507: return 0; // No signal
	mov	dptr,#(0x00&0x00ff)
	clr	a
	mov	b,a
	ret
L024007?:
;	EFM8_JDY40_test.c:513: TR0=0;
	clr	_TR0
;	EFM8_JDY40_test.c:514: TL0=0; TH0=0; TF0=0; overflow_count=0;
	mov	_TL0,#0x00
	mov	_TH0,#0x00
	clr	_TF0
;	EFM8_JDY40_test.c:515: TR0=1;
	setb	_TR0
;	EFM8_JDY40_test.c:516: while(PERIOD_PIN!=1) // Wait for the signal to be one
	mov	r4,#0x00
	mov	r5,#0x00
L024012?:
	jb	_P2_6,L024014?
;	EFM8_JDY40_test.c:518: if(TF0==1) // Did the 16-bit timer overflow?
;	EFM8_JDY40_test.c:520: TF0=0;
	jbc	_TF0,L024055?
	sjmp	L024012?
L024055?:
;	EFM8_JDY40_test.c:521: overflow_count++;
	inc	r4
	cjne	r4,#0x00,L024056?
	inc	r5
L024056?:
;	EFM8_JDY40_test.c:522: if(overflow_count==10) // If it overflows too many times assume no signal is present
	cjne	r4,#0x0A,L024012?
	cjne	r5,#0x00,L024012?
;	EFM8_JDY40_test.c:524: TR0=0;
	clr	_TR0
;	EFM8_JDY40_test.c:525: return 0; // No signal
	mov	dptr,#(0x00&0x00ff)
	clr	a
	mov	b,a
	ret
L024014?:
;	EFM8_JDY40_test.c:531: TR0=0;
	clr	_TR0
;	EFM8_JDY40_test.c:532: TL0=0; TH0=0; TF0=0; overflow_count=0;
	mov	_TL0,#0x00
	mov	_TH0,#0x00
	clr	_TF0
	mov	r4,#0x00
	mov	r5,#0x00
;	EFM8_JDY40_test.c:533: TR0=1; // Start the timer
	setb	_TR0
;	EFM8_JDY40_test.c:534: for(i=0; i<n; i++) // Measure the time of 'n' periods
	mov	r6,#0x00
L024025?:
	mov	ar7,r6
	mov	r0,#0x00
	clr	c
	mov	a,r7
	subb	a,r2
	mov	a,r0
	xrl	a,#0x80
	mov	b,r3
	xrl	b,#0x80
	subb	a,b
	jnc	L024028?
;	EFM8_JDY40_test.c:536: while(PERIOD_PIN!=0) // Wait for the signal to be zero
	mov	ar7,r4
	mov	ar0,r5
L024017?:
	jnb	_P2_6,L024043?
;	EFM8_JDY40_test.c:538: if(TF0==1) // Did the 16-bit timer overflow?
;	EFM8_JDY40_test.c:540: TF0=0;
	jbc	_TF0,L024061?
	sjmp	L024017?
L024061?:
;	EFM8_JDY40_test.c:541: overflow_count++;
	inc	r7
	cjne	r7,#0x00,L024017?
	inc	r0
;	EFM8_JDY40_test.c:544: while(PERIOD_PIN!=1) // Wait for the signal to be one
	sjmp	L024017?
L024043?:
L024022?:
	jb	_P2_6,L024047?
;	EFM8_JDY40_test.c:546: if(TF0==1) // Did the 16-bit timer overflow?
;	EFM8_JDY40_test.c:548: TF0=0;
	jbc	_TF0,L024063?
	sjmp	L024022?
L024063?:
;	EFM8_JDY40_test.c:549: overflow_count++;
	inc	r7
	cjne	r7,#0x00,L024022?
	inc	r0
	sjmp	L024022?
L024047?:
	mov	ar4,r7
	mov	ar5,r0
;	EFM8_JDY40_test.c:534: for(i=0; i<n; i++) // Measure the time of 'n' periods
	inc	r6
	sjmp	L024025?
L024028?:
;	EFM8_JDY40_test.c:553: TR0=0; // Stop timer 0, the 24-bit number [overflow_count-TH0-TL0] has the period in clock cycles!
	clr	_TR0
;	EFM8_JDY40_test.c:555: return (overflow_count*65536+TH0*256+TL0);
	mov	ar3,r5
	mov	ar2,r4
	mov	r5,#0x00
	mov	r4,#0x00
	mov	r7,_TH0
	mov	r6,#0x00
	mov	a,r7
	rlc	a
	subb	a,acc
	mov	r0,a
	mov	r1,a
	mov	a,r6
	add	a,r4
	mov	r4,a
	mov	a,r7
	addc	a,r5
	mov	r5,a
	mov	a,r0
	addc	a,r2
	mov	r2,a
	mov	a,r1
	addc	a,r3
	mov	r3,a
	mov	r6,_TL0
	clr	a
	mov	r7,a
	rlc	a
	subb	a,acc
	mov	r0,a
	mov	r1,a
	mov	a,r6
	add	a,r4
	mov	r4,a
	mov	a,r7
	addc	a,r5
	mov	r5,a
	mov	a,r0
	addc	a,r2
	mov	r2,a
	mov	a,r1
	addc	a,r3
	mov	dpl,r4
	mov	dph,r5
	mov	b,r2
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'eputs'
;------------------------------------------------------------
;String                    Allocated to registers r2 r3 r4 
;------------------------------------------------------------
;	EFM8_JDY40_test.c:558: void eputs(char *String)
;	-----------------------------------------
;	 function eputs
;	-----------------------------------------
_eputs:
	mov	r2,dpl
	mov	r3,dph
	mov	r4,b
;	EFM8_JDY40_test.c:560: while(*String)
L025001?:
	mov	dpl,r2
	mov	dph,r3
	mov	b,r4
	lcall	__gptrget
	mov	r5,a
	jz	L025004?
;	EFM8_JDY40_test.c:562: putchar(*String);
	mov	dpl,r5
	push	ar2
	push	ar3
	push	ar4
	lcall	_putchar
	pop	ar4
	pop	ar3
	pop	ar2
;	EFM8_JDY40_test.c:563: String++;
	inc	r2
	cjne	r2,#0x00,L025001?
	inc	r3
	sjmp	L025001?
L025004?:
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'PrintNumber'
;------------------------------------------------------------
;Base                      Allocated with name '_PrintNumber_PARM_2'
;digits                    Allocated with name '_PrintNumber_PARM_3'
;val                       Allocated with name '_PrintNumber_val_1_159'
;j                         Allocated with name '_PrintNumber_j_1_160'
;sloc0                     Allocated with name '_PrintNumber_sloc0_1_0'
;sloc1                     Allocated with name '_PrintNumber_sloc1_1_0'
;buff                      Allocated with name '_PrintNumber_buff_1_160'
;------------------------------------------------------------
;	EFM8_JDY40_test.c:567: void PrintNumber(long int val, int Base, int digits)
;	-----------------------------------------
;	 function PrintNumber
;	-----------------------------------------
_PrintNumber:
	mov	_PrintNumber_val_1_159,dpl
	mov	(_PrintNumber_val_1_159 + 1),dph
	mov	(_PrintNumber_val_1_159 + 2),b
	mov	(_PrintNumber_val_1_159 + 3),a
;	EFM8_JDY40_test.c:573: buff[NBITS]=0;
	mov	dptr,#(_PrintNumber_buff_1_160 + 0x0020)
	clr	a
	movx	@dptr,a
;	EFM8_JDY40_test.c:575: if(val<0)
	mov	a,(_PrintNumber_val_1_159 + 3)
	jnb	acc.7,L026012?
;	EFM8_JDY40_test.c:577: putchar('-');
	mov	dpl,#0x2D
	lcall	_putchar
;	EFM8_JDY40_test.c:578: val*=-1;
	clr	c
	clr	a
	subb	a,_PrintNumber_val_1_159
	mov	_PrintNumber_val_1_159,a
	clr	a
	subb	a,(_PrintNumber_val_1_159 + 1)
	mov	(_PrintNumber_val_1_159 + 1),a
	clr	a
	subb	a,(_PrintNumber_val_1_159 + 2)
	mov	(_PrintNumber_val_1_159 + 2),a
	clr	a
	subb	a,(_PrintNumber_val_1_159 + 3)
	mov	(_PrintNumber_val_1_159 + 3),a
;	EFM8_JDY40_test.c:582: while ( (val>0) | (digits>0) )
L026012?:
	mov	_PrintNumber_j_1_160,#0x1F
	clr	a
	mov	(_PrintNumber_j_1_160 + 1),a
	mov	r0,_PrintNumber_PARM_3
	mov	r1,(_PrintNumber_PARM_3 + 1)
L026005?:
	clr	c
	clr	a
	subb	a,_PrintNumber_val_1_159
	clr	a
	subb	a,(_PrintNumber_val_1_159 + 1)
	clr	a
	subb	a,(_PrintNumber_val_1_159 + 2)
	clr	a
	xrl	a,#0x80
	mov	b,(_PrintNumber_val_1_159 + 3)
	xrl	b,#0x80
	subb	a,b
	clr	a
	rlc	a
	mov	r6,a
	clr	c
	clr	a
	subb	a,r0
	clr	a
	xrl	a,#0x80
	mov	b,r1
	xrl	b,#0x80
	subb	a,b
	clr	a
	rlc	a
	mov	r7,a
	orl	a,r6
	jnz	L026016?
	ljmp	L026007?
L026016?:
;	EFM8_JDY40_test.c:584: buff[j--]=HexDigit[val%Base];
	mov	r6,_PrintNumber_j_1_160
	mov	r7,(_PrintNumber_j_1_160 + 1)
	dec	_PrintNumber_j_1_160
	mov	a,#0xff
	cjne	a,_PrintNumber_j_1_160,L026017?
	dec	(_PrintNumber_j_1_160 + 1)
L026017?:
	mov	a,r6
	add	a,#_PrintNumber_buff_1_160
	mov	_PrintNumber_sloc0_1_0,a
	mov	a,r7
	addc	a,#(_PrintNumber_buff_1_160 >> 8)
	mov	(_PrintNumber_sloc0_1_0 + 1),a
	mov	_PrintNumber_sloc1_1_0,_PrintNumber_PARM_2
	mov	a,(_PrintNumber_PARM_2 + 1)
	mov	(_PrintNumber_sloc1_1_0 + 1),a
	rlc	a
	subb	a,acc
	mov	(_PrintNumber_sloc1_1_0 + 2),a
	mov	(_PrintNumber_sloc1_1_0 + 3),a
	mov	__modslong_PARM_2,_PrintNumber_sloc1_1_0
	mov	(__modslong_PARM_2 + 1),(_PrintNumber_sloc1_1_0 + 1)
	mov	(__modslong_PARM_2 + 2),(_PrintNumber_sloc1_1_0 + 2)
	mov	(__modslong_PARM_2 + 3),(_PrintNumber_sloc1_1_0 + 3)
	mov	dpl,_PrintNumber_val_1_159
	mov	dph,(_PrintNumber_val_1_159 + 1)
	mov	b,(_PrintNumber_val_1_159 + 2)
	mov	a,(_PrintNumber_val_1_159 + 3)
	push	ar0
	push	ar1
	lcall	__modslong
	mov	r6,dpl
	mov	r7,dph
	mov	a,r6
	add	a,#_PrintNumber_HexDigit_1_160
	mov	dpl,a
	mov	a,r7
	addc	a,#(_PrintNumber_HexDigit_1_160 >> 8)
	mov	dph,a
	clr	a
	movc	a,@a+dptr
	mov	dpl,_PrintNumber_sloc0_1_0
	mov	dph,(_PrintNumber_sloc0_1_0 + 1)
	movx	@dptr,a
;	EFM8_JDY40_test.c:585: val/=Base;
	mov	__divslong_PARM_2,_PrintNumber_sloc1_1_0
	mov	(__divslong_PARM_2 + 1),(_PrintNumber_sloc1_1_0 + 1)
	mov	(__divslong_PARM_2 + 2),(_PrintNumber_sloc1_1_0 + 2)
	mov	(__divslong_PARM_2 + 3),(_PrintNumber_sloc1_1_0 + 3)
	mov	dpl,_PrintNumber_val_1_159
	mov	dph,(_PrintNumber_val_1_159 + 1)
	mov	b,(_PrintNumber_val_1_159 + 2)
	mov	a,(_PrintNumber_val_1_159 + 3)
	lcall	__divslong
	mov	_PrintNumber_val_1_159,dpl
	mov	(_PrintNumber_val_1_159 + 1),dph
	mov	(_PrintNumber_val_1_159 + 2),b
	mov	(_PrintNumber_val_1_159 + 3),a
	pop	ar1
	pop	ar0
;	EFM8_JDY40_test.c:586: if(digits!=0) digits--;
	mov	a,r0
	orl	a,r1
	jnz	L026018?
	ljmp	L026005?
L026018?:
	dec	r0
	cjne	r0,#0xff,L026019?
	dec	r1
L026019?:
	ljmp	L026005?
L026007?:
;	EFM8_JDY40_test.c:588: eputs(&buff[j+1]);
	mov	a,_PrintNumber_j_1_160
	inc	a
	add	a,#_PrintNumber_buff_1_160
	mov	r2,a
	clr	a
	addc	a,#(_PrintNumber_buff_1_160 >> 8)
	mov	r3,a
	mov	r4,#0x00
	mov	dpl,r2
	mov	dph,r3
	mov	b,r4
	ljmp	_eputs
;------------------------------------------------------------
;Allocation info for local variables in function 'GetFrequency'
;------------------------------------------------------------
;pin                       Allocated with name '_GetFrequency_PARM_2'
;c                         Allocated to registers r2 r3 r4 r5 
;f                         Allocated to registers r6 r7 r0 r1 
;------------------------------------------------------------
;	EFM8_JDY40_test.c:591: unsigned long GetFrequency (long int c, int pin)
;	-----------------------------------------
;	 function GetFrequency
;	-----------------------------------------
_GetFrequency:
	mov	r2,dpl
	mov	r3,dph
	mov	r4,b
	mov	r5,a
;	EFM8_JDY40_test.c:593: long int f = 0;
	mov	r6,#0x00
	mov	r7,#0x00
	mov	r0,#0x00
	mov	r1,#0x00
;	EFM8_JDY40_test.c:595: if(c>0)
	clr	c
	clr	a
	subb	a,r2
	clr	a
	subb	a,r3
	clr	a
	subb	a,r4
	clr	a
	xrl	a,#0x80
	mov	b,r5
	xrl	b,#0x80
	subb	a,b
	jc	L027006?
	ljmp	L027002?
L027006?:
;	EFM8_JDY40_test.c:597: f=(SYSCLK*200.0)/(c*12);
	mov	__mullong_PARM_2,r2
	mov	(__mullong_PARM_2 + 1),r3
	mov	(__mullong_PARM_2 + 2),r4
	mov	(__mullong_PARM_2 + 3),r5
	mov	dptr,#(0x0C&0x00ff)
	clr	a
	mov	b,a
	lcall	__mullong
	lcall	___slong2fs
	mov	r2,dpl
	mov	r3,dph
	mov	r4,b
	mov	r5,a
	push	ar2
	push	ar3
	push	ar4
	push	ar5
	mov	dptr,#0x93A4
	mov	b,#0x56
	mov	a,#0x50
	lcall	___fsdiv
	mov	r2,dpl
	mov	r3,dph
	mov	r4,b
	mov	r5,a
	mov	a,sp
	add	a,#0xfc
	mov	sp,a
	mov	dpl,r2
	mov	dph,r3
	mov	b,r4
	mov	a,r5
	lcall	___fs2slong
	mov	r6,dpl
	mov	r7,dph
	mov	r0,b
	mov	r1,a
;	EFM8_JDY40_test.c:598: eputs(" f");
	mov	dptr,#__str_4
	mov	b,#0x80
	push	ar6
	push	ar7
	push	ar0
	push	ar1
	lcall	_eputs
;	EFM8_JDY40_test.c:599: PrintNumber(pin, 10, 1);
	mov	r2,_GetFrequency_PARM_2
	mov	a,(_GetFrequency_PARM_2 + 1)
	mov	r3,a
	rlc	a
	subb	a,acc
	mov	r4,a
	mov	r5,a
	mov	_PrintNumber_PARM_2,#0x0A
	clr	a
	mov	(_PrintNumber_PARM_2 + 1),a
	mov	_PrintNumber_PARM_3,#0x01
	clr	a
	mov	(_PrintNumber_PARM_3 + 1),a
	mov	dpl,r2
	mov	dph,r3
	mov	b,r4
	mov	a,r5
	lcall	_PrintNumber
;	EFM8_JDY40_test.c:600: eputs(" = ");
	mov	dptr,#__str_5
	mov	b,#0x80
	lcall	_eputs
	pop	ar1
	pop	ar0
	pop	ar7
	pop	ar6
;	EFM8_JDY40_test.c:601: PrintNumber(f, 10, 7);
	mov	_PrintNumber_PARM_2,#0x0A
	clr	a
	mov	(_PrintNumber_PARM_2 + 1),a
	mov	_PrintNumber_PARM_3,#0x07
	clr	a
	mov	(_PrintNumber_PARM_3 + 1),a
	mov	dpl,r6
	mov	dph,r7
	mov	b,r0
	mov	a,r1
	push	ar6
	push	ar7
	push	ar0
	push	ar1
	lcall	_PrintNumber
;	EFM8_JDY40_test.c:602: eputs("Hz");
	mov	dptr,#__str_6
	mov	b,#0x80
	lcall	_eputs
	pop	ar1
	pop	ar0
	pop	ar7
	pop	ar6
L027002?:
;	EFM8_JDY40_test.c:610: return f;
	mov	dpl,r6
	mov	dph,r7
	mov	b,r0
	mov	a,r1
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'CoinDecider'
;------------------------------------------------------------
;freq                      Allocated to registers r2 r3 r4 r5 
;------------------------------------------------------------
;	EFM8_JDY40_test.c:613: int CoinDecider(long int freq)
;	-----------------------------------------
;	 function CoinDecider
;	-----------------------------------------
_CoinDecider:
	mov	r2,dpl
	mov	r3,dph
	mov	r4,b
	mov	r5,a
;	EFM8_JDY40_test.c:615: if(freq>=56300) // detects a coin
	clr	c
	mov	a,r2
	subb	a,#0xEC
	mov	a,r3
	subb	a,#0xDB
	mov	a,r4
	subb	a,#0x00
	mov	a,r5
	xrl	a,#0x80
	subb	a,#0x80
	jnc	L028018?
	ljmp	L028010?
L028018?:
;	EFM8_JDY40_test.c:618: if((freq >= 56200) && (freq < 56400))
	clr	c
	mov	a,r2
	subb	a,#0x88
	mov	a,r3
	subb	a,#0xDB
	mov	a,r4
	subb	a,#0x00
	mov	a,r5
	xrl	a,#0x80
	subb	a,#0x80
	jc	L028006?
	mov	a,r2
	subb	a,#0x50
	mov	a,r3
	subb	a,#0xDC
	mov	a,r4
	subb	a,#0x00
	mov	a,r5
	xrl	a,#0x80
	subb	a,#0x80
	jnc	L028006?
;	EFM8_JDY40_test.c:620: eputs(" DIME");
	mov	dptr,#__str_7
	mov	b,#0x80
	lcall	_eputs
	sjmp	L028007?
L028006?:
;	EFM8_JDY40_test.c:624: else if ((freq >= 56400) && (freq < 56700))
	clr	c
	mov	a,r2
	subb	a,#0x50
	mov	a,r3
	subb	a,#0xDC
	mov	a,r4
	subb	a,#0x00
	mov	a,r5
	xrl	a,#0x80
	subb	a,#0x80
	jc	L028002?
	mov	a,r2
	subb	a,#0x7C
	mov	a,r3
	subb	a,#0xDD
	mov	a,r4
	subb	a,#0x00
	mov	a,r5
	xrl	a,#0x80
	subb	a,#0x80
	jnc	L028002?
;	EFM8_JDY40_test.c:626: eputs(" NICKEL");
	mov	dptr,#__str_8
	mov	b,#0x80
	lcall	_eputs
	sjmp	L028007?
L028002?:
;	EFM8_JDY40_test.c:632: eputs(" LOONIE");
	mov	dptr,#__str_9
	mov	b,#0x80
	lcall	_eputs
L028007?:
;	EFM8_JDY40_test.c:635: return 1;
	mov	dptr,#0x0001
;	EFM8_JDY40_test.c:643: return 0;
	ret
L028010?:
	mov	dptr,#0x0000
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'main'
;------------------------------------------------------------
;evilcode                  Allocated with name '_main_evilcode_1_175'
;evilcode1                 Allocated with name '_main_evilcode1_1_175'
;timeout                   Allocated to registers 
;pulse_width               Allocated to registers 
;pulse_width1              Allocated to registers 
;speed                     Allocated with name '_main_speed_1_175'
;steering                  Allocated with name '_main_steering_1_175'
;adcwheel1                 Allocated with name '_main_adcwheel1_1_175'
;adcwheel2                 Allocated with name '_main_adcwheel2_1_175'
;count                     Allocated to registers r2 r3 r4 r5 
;f                         Allocated to registers r2 r3 r4 r5 
;coinPresent               Allocated to registers 
;p_thresh                  Allocated to registers 
;v                         Allocated with name '_main_v_1_175'
;------------------------------------------------------------
;	EFM8_JDY40_test.c:646: void main (void)
;	-----------------------------------------
;	 function main
;	-----------------------------------------
_main:
;	EFM8_JDY40_test.c:664: waitms(500);
	mov	dptr,#0x01F4
	lcall	_waitms
;	EFM8_JDY40_test.c:665: printf("\r\nEFM8LB12 JDY-40 Slave Test.\r\n");
	mov	a,#__str_10
	push	acc
	mov	a,#(__str_10 >> 8)
	push	acc
	mov	a,#0x80
	push	acc
	lcall	_printf
	dec	sp
	dec	sp
	dec	sp
;	EFM8_JDY40_test.c:666: UART1_Init(9600);
	mov	dptr,#0x2580
	clr	a
	mov	b,a
	lcall	_UART1_Init
;	EFM8_JDY40_test.c:668: ReceptionOff();
	lcall	_ReceptionOff
;	EFM8_JDY40_test.c:670: TIMER0_Init(); 
	lcall	_TIMER0_Init
;	EFM8_JDY40_test.c:672: InitPinADC(2, 1); // Configure P2.1 as analog input
	mov	_InitPinADC_PARM_2,#0x01
	mov	dpl,#0x02
	lcall	_InitPinADC
;	EFM8_JDY40_test.c:673: InitPinADC(2, 2); // Configure P2.2 as analog input
	mov	_InitPinADC_PARM_2,#0x02
	mov	dpl,#0x02
	lcall	_InitPinADC
;	EFM8_JDY40_test.c:674: InitADC();
	lcall	_InitADC
;	EFM8_JDY40_test.c:677: SendATCommand("AT+VER\r\n");
	mov	dptr,#__str_11
	mov	b,#0x80
	lcall	_SendATCommand
;	EFM8_JDY40_test.c:678: SendATCommand("AT+BAUD\r\n");
	mov	dptr,#__str_12
	mov	b,#0x80
	lcall	_SendATCommand
;	EFM8_JDY40_test.c:679: SendATCommand("AT+RFID\r\n");
	mov	dptr,#__str_13
	mov	b,#0x80
	lcall	_SendATCommand
;	EFM8_JDY40_test.c:680: SendATCommand("AT+DVID\r\n");
	mov	dptr,#__str_14
	mov	b,#0x80
	lcall	_SendATCommand
;	EFM8_JDY40_test.c:681: SendATCommand("AT+RFC120\r\n");
	mov	dptr,#__str_15
	mov	b,#0x80
	lcall	_SendATCommand
;	EFM8_JDY40_test.c:682: SendATCommand("AT+POWE\r\n");
	mov	dptr,#__str_16
	mov	b,#0x80
	lcall	_SendATCommand
;	EFM8_JDY40_test.c:683: SendATCommand("AT+CLSS\r\n");
	mov	dptr,#__str_17
	mov	b,#0x80
	lcall	_SendATCommand
;	EFM8_JDY40_test.c:687: SendATCommand("AT+DVIDFFFF\r\n");  
	mov	dptr,#__str_18
	mov	b,#0x80
	lcall	_SendATCommand
;	EFM8_JDY40_test.c:689: while(1)
L029009?:
;	EFM8_JDY40_test.c:692: count = GetPeriod(200);
	mov	dptr,#0x00C8
	lcall	_GetPeriod
	mov	r2,dpl
	mov	r3,dph
	mov	r4,b
	mov	r5,a
;	EFM8_JDY40_test.c:693: f = GetFrequency(count, 1);
	mov	_GetFrequency_PARM_2,#0x01
	clr	a
	mov	(_GetFrequency_PARM_2 + 1),a
	mov	dpl,r2
	mov	dph,r3
	mov	b,r4
	mov	a,r5
	lcall	_GetFrequency
;	EFM8_JDY40_test.c:694: coinPresent = CoinDecider(f); 
	mov	r2,dpl
	mov	r3,dph
	mov	r4,b
	mov	r5,a
	push	ar2
	push	ar3
	push	ar4
	push	ar5
	lcall	_CoinDecider
	mov	a,dpl
	mov	b,dph
	pop	ar5
	pop	ar4
	pop	ar3
	pop	ar2
;	EFM8_JDY40_test.c:695: if(coinPresent)
	orl	a,b
	jz	L029002?
;	EFM8_JDY40_test.c:697: sprintf(msg, "%ld", f);
	push	ar2
	push	ar3
	push	ar4
	push	ar5
	mov	a,#__str_19
	push	acc
	mov	a,#(__str_19 >> 8)
	push	acc
	mov	a,#0x80
	push	acc
	mov	a,#_msg
	push	acc
	mov	a,#(_msg >> 8)
	push	acc
	mov	a,#0x40
	push	acc
	lcall	_sprintf
	mov	a,sp
	add	a,#0xf6
	mov	sp,a
;	EFM8_JDY40_test.c:698: sendstr1(msg);
	mov	dptr,#_msg
	mov	b,#0x40
	lcall	_sendstr1
L029002?:
;	EFM8_JDY40_test.c:702: v[0] = Volts_at_Pin(QFP32_MUX_P2_1);
	mov	dpl,#0x0E
	lcall	_Volts_at_Pin
	mov	r2,dpl
	mov	r3,dph
	mov	r4,b
	mov	r5,a
	mov	_main_v_1_175,r2
	mov	(_main_v_1_175 + 1),r3
	mov	(_main_v_1_175 + 2),r4
	mov	(_main_v_1_175 + 3),r5
;	EFM8_JDY40_test.c:703: v[1] = Volts_at_Pin(QFP32_MUX_P2_2);
	mov	dpl,#0x0F
	lcall	_Volts_at_Pin
	mov	r2,dpl
	mov	r3,dph
	mov	r4,b
	mov	r5,a
	mov	(_main_v_1_175 + 0x0004),r2
	mov	((_main_v_1_175 + 0x0004) + 1),r3
	mov	((_main_v_1_175 + 0x0004) + 2),r4
	mov	((_main_v_1_175 + 0x0004) + 3),r5
;	EFM8_JDY40_test.c:726: if(RXU1()) // Something has arrived
	lcall	_RXU1
	jc	L029018?
	ljmp	L029009?
L029018?:
;	EFM8_JDY40_test.c:729: getstr1(buff, sizeof(buff));
	mov	_getstr1_PARM_2,#0x14
	mov	dptr,#_buff
	mov	b,#0x40
	lcall	_getstr1
;	EFM8_JDY40_test.c:730: sscanf(buff, "S%dT%d", &speed, &steering);
	mov	a,#_main_steering_1_175
	push	acc
	mov	a,#(_main_steering_1_175 >> 8)
	push	acc
	mov	a,#0x40
	push	acc
	mov	a,#_main_speed_1_175
	push	acc
	mov	a,#(_main_speed_1_175 >> 8)
	push	acc
	mov	a,#0x40
	push	acc
	mov	a,#__str_20
	push	acc
	mov	a,#(__str_20 >> 8)
	push	acc
	mov	a,#0x80
	push	acc
	mov	a,#_buff
	push	acc
	mov	a,#(_buff >> 8)
	push	acc
	mov	a,#0x40
	push	acc
	lcall	_sscanf
	mov	a,sp
	add	a,#0xf4
	mov	sp,a
;	EFM8_JDY40_test.c:731: if (speed < 503 )
	clr	c
	mov	a,_main_speed_1_175
	subb	a,#0xF7
	mov	a,(_main_speed_1_175 + 1)
	xrl	a,#0x80
	subb	a,#0x81
	jnc	L029004?
;	EFM8_JDY40_test.c:733: P2_5 = 0;
	clr	_P2_5
;	EFM8_JDY40_test.c:734: P3_7=0;
	clr	_P3_7
;	EFM8_JDY40_test.c:735: direction = 1;
	mov	_direction,#0x01
	clr	a
	mov	(_direction + 1),a
	sjmp	L029005?
L029004?:
;	EFM8_JDY40_test.c:739: P3_2=0;
	clr	_P3_2
;	EFM8_JDY40_test.c:740: P3_0=0;
	clr	_P3_0
;	EFM8_JDY40_test.c:741: direction = 0;
	clr	a
	mov	_direction,a
	mov	(_direction + 1),a
L029005?:
;	EFM8_JDY40_test.c:744: ADCsteeringRatio(speed, steering, &adcwheel1, &adcwheel2);
	mov	_ADCsteeringRatio_PARM_3,#_main_adcwheel1_1_175
	mov	(_ADCsteeringRatio_PARM_3 + 1),#0x00
	mov	(_ADCsteeringRatio_PARM_3 + 2),#0x40
	mov	_ADCsteeringRatio_PARM_4,#_main_adcwheel2_1_175
	mov	(_ADCsteeringRatio_PARM_4 + 1),#0x00
	mov	(_ADCsteeringRatio_PARM_4 + 2),#0x40
	mov	_ADCsteeringRatio_PARM_2,_main_steering_1_175
	mov	(_ADCsteeringRatio_PARM_2 + 1),(_main_steering_1_175 + 1)
	mov	dpl,_main_speed_1_175
	mov	dph,(_main_speed_1_175 + 1)
	lcall	_ADCsteeringRatio
;	EFM8_JDY40_test.c:746: pwm_duty4 = ADCtoPWM(adcwheel1);
	mov	dpl,_main_adcwheel1_1_175
	mov	dph,(_main_adcwheel1_1_175 + 1)
	lcall	_ADCtoPWM
	mov	_pwm_duty4,dpl
	mov	(_pwm_duty4 + 1),dph
;	EFM8_JDY40_test.c:747: pwm_duty2 = ADCtoPWM(adcwheel2);
	mov	dpl,_main_adcwheel2_1_175
	mov	dph,(_main_adcwheel2_1_175 + 1)
	lcall	_ADCtoPWM
	mov	_pwm_duty2,dpl
	mov	(_pwm_duty2 + 1),dph
;	EFM8_JDY40_test.c:749: printf("duty4= %u duty2 = %u buff=%s speed=%u steering=%u\n\r", pwm_duty4, pwm_duty2, buff, adcwheel1, adcwheel2);
	push	_main_adcwheel2_1_175
	push	(_main_adcwheel2_1_175 + 1)
	push	_main_adcwheel1_1_175
	push	(_main_adcwheel1_1_175 + 1)
	mov	a,#_buff
	push	acc
	mov	a,#(_buff >> 8)
	push	acc
	mov	a,#0x40
	push	acc
	push	_pwm_duty2
	push	(_pwm_duty2 + 1)
	push	_pwm_duty4
	push	(_pwm_duty4 + 1)
	mov	a,#__str_21
	push	acc
	mov	a,#(__str_21 >> 8)
	push	acc
	mov	a,#0x80
	push	acc
	lcall	_printf
	mov	a,sp
	add	a,#0xf2
	mov	sp,a
;	EFM8_JDY40_test.c:752: waitms(5); // The radio seems to need this delay...
	mov	dptr,#0x0005
	lcall	_waitms
	ljmp	L029009?
	rseg R_CSEG

	rseg R_XINIT

	rseg R_CONST
__str_0:
	db 'Command: %s'
	db 0x00
__str_1:
	db 'Response: %s'
	db 0x0D
	db 0x0A
	db 0x00
__str_2:
	db 'AT+DVID0000'
	db 0x0D
	db 0x0A
	db 0x00
_PrintNumber_HexDigit_1_160:
	db '0123456789ABCDEF'
	db 0x00
__str_4:
	db ' f'
	db 0x00
__str_5:
	db ' = '
	db 0x00
__str_6:
	db 'Hz'
	db 0x00
__str_7:
	db ' DIME'
	db 0x00
__str_8:
	db ' NICKEL'
	db 0x00
__str_9:
	db ' LOONIE'
	db 0x00
__str_10:
	db 0x0D
	db 0x0A
	db 'EFM8LB12 JDY-40 Slave Test.'
	db 0x0D
	db 0x0A
	db 0x00
__str_11:
	db 'AT+VER'
	db 0x0D
	db 0x0A
	db 0x00
__str_12:
	db 'AT+BAUD'
	db 0x0D
	db 0x0A
	db 0x00
__str_13:
	db 'AT+RFID'
	db 0x0D
	db 0x0A
	db 0x00
__str_14:
	db 'AT+DVID'
	db 0x0D
	db 0x0A
	db 0x00
__str_15:
	db 'AT+RFC120'
	db 0x0D
	db 0x0A
	db 0x00
__str_16:
	db 'AT+POWE'
	db 0x0D
	db 0x0A
	db 0x00
__str_17:
	db 'AT+CLSS'
	db 0x0D
	db 0x0A
	db 0x00
__str_18:
	db 'AT+DVIDFFFF'
	db 0x0D
	db 0x0A
	db 0x00
__str_19:
	db '%ld'
	db 0x00
__str_20:
	db 'S%dT%d'
	db 0x00
__str_21:
	db 'duty4= %u duty2 = %u buff=%s speed=%u steering=%u'
	db 0x0A
	db 0x0D
	db 0x00

	CSEG

end
