;--------------------------------------------------------
; File Created by C51
; Version 1.0.0 #1170 (Feb 16 2022) (MSVC)
; This file was generated Thu Apr 03 13:58:20 2025
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
	public _PrintNumber_HexDigit_1_164
	public _customHappyMouth
	public _customMoney
	public _customSparkle
	public _customOpenMouth
	public _customEye
	public _customMouth
	public _main
	public _automaticmode
	public _servomotion
	public _LCD_4BIT
	public _WriteCommand
	public _WriteData
	public _LCD_byte
	public _LCD_pulse
	public _GetFrequency
	public _PrintNumber
	public _eputs
	public _GetPeriod
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
	public _Timer5_ISR
	public __c51_external_startup
	public _msg
	public _buff
	public _automaticmode_PARM_3
	public _automaticmode_PARM_2
	public _PrintNumber_PARM_3
	public _PrintNumber_PARM_2
	public _getstr1_PARM_2
	public _overflow_count
	public _cointcount
	public _servo2
	public _servo1
	public _servo_counter
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
_servo_counter:
	ds 2
_servo1:
	ds 1
_servo2:
	ds 1
_cointcount:
	ds 2
_overflow_count:
	ds 1
_getstr1_PARM_2:
	ds 1
_PrintNumber_PARM_2:
	ds 2
_PrintNumber_PARM_3:
	ds 2
_PrintNumber_val_1_163:
	ds 4
_PrintNumber_j_1_164:
	ds 2
_PrintNumber_sloc0_1_0:
	ds 2
_PrintNumber_sloc1_1_0:
	ds 4
_automaticmode_PARM_2:
	ds 4
_automaticmode_PARM_3:
	ds 4
_main_adcwheel1_1_199:
	ds 2
_main_adcwheel2_1_199:
	ds 2
_main_which_1_199:
	ds 2
_main_f_1_199:
	ds 4
_main_v_1_199:
	ds 8
_main_sloc0_1_0:
	ds 4
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
_PrintNumber_buff_1_164:
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
	CSEG at 0x0093
	ljmp	_Timer5_ISR
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
;	EFM8_JDY40_test.c:23: volatile unsigned int pwm_counter4=0;
	clr	a
	mov	_pwm_counter4,a
	mov	(_pwm_counter4 + 1),a
;	EFM8_JDY40_test.c:24: volatile unsigned int pwm_duty4=65535; //(0?65535)
	mov	_pwm_duty4,#0xFF
	mov	(_pwm_duty4 + 1),#0xFF
;	EFM8_JDY40_test.c:29: volatile unsigned int pwm_counter2=0;
	clr	a
	mov	_pwm_counter2,a
	mov	(_pwm_counter2 + 1),a
;	EFM8_JDY40_test.c:30: volatile unsigned int pwm_duty2=65535; //(0?65535)
	mov	_pwm_duty2,#0xFF
	mov	(_pwm_duty2 + 1),#0xFF
;	EFM8_JDY40_test.c:31: volatile int direction=0;
	clr	a
	mov	_direction,a
	mov	(_direction + 1),a
;	EFM8_JDY40_test.c:41: volatile unsigned int servo_counter=0;
	clr	a
	mov	_servo_counter,a
	mov	(_servo_counter + 1),a
;	EFM8_JDY40_test.c:42: volatile unsigned char servo1=250, servo2=250;
	mov	_servo1,#0xFA
;	EFM8_JDY40_test.c:42: volatile unsigned int cointcount =0;
	mov	_servo2,#0xFA
;	EFM8_JDY40_test.c:43: /////////////////////////////LCD//////////////////////////
	clr	a
	mov	_cointcount,a
	mov	(_cointcount + 1),a
	; The linker places a 'ret' at the end of segment R_DINIT.
;--------------------------------------------------------
; code
;--------------------------------------------------------
	rseg R_CSEG
;------------------------------------------------------------
;Allocation info for local variables in function '_c51_external_startup'
;------------------------------------------------------------
;------------------------------------------------------------
;	EFM8_JDY40_test.c:70: char _c51_external_startup (void)
;	-----------------------------------------
;	 function _c51_external_startup
;	-----------------------------------------
__c51_external_startup:
	using	0
;	EFM8_JDY40_test.c:73: SFRPAGE = 0x00;
	mov	_SFRPAGE,#0x00
;	EFM8_JDY40_test.c:74: WDTCN = 0xDE; //First key
	mov	_WDTCN,#0xDE
;	EFM8_JDY40_test.c:75: WDTCN = 0xAD; //Second key
	mov	_WDTCN,#0xAD
;	EFM8_JDY40_test.c:77: VDM0CN=0x80;       // enable VDD monitor
	mov	_VDM0CN,#0x80
;	EFM8_JDY40_test.c:78: RSTSRC=0x02|0x04;  // Enable reset on missing clock detector and VDD
	mov	_RSTSRC,#0x06
;	EFM8_JDY40_test.c:85: SFRPAGE = 0x10;
	mov	_SFRPAGE,#0x10
;	EFM8_JDY40_test.c:86: PFE0CN  = 0x20; // SYSCLK < 75 MHz.
	mov	_PFE0CN,#0x20
;	EFM8_JDY40_test.c:87: SFRPAGE = 0x00;
	mov	_SFRPAGE,#0x00
;	EFM8_JDY40_test.c:108: CLKSEL = 0x00;
	mov	_CLKSEL,#0x00
;	EFM8_JDY40_test.c:109: CLKSEL = 0x00;
	mov	_CLKSEL,#0x00
;	EFM8_JDY40_test.c:110: while ((CLKSEL & 0x80) == 0);
L002001?:
	mov	a,_CLKSEL
	jnb	acc.7,L002001?
;	EFM8_JDY40_test.c:111: CLKSEL = 0x03;
	mov	_CLKSEL,#0x03
;	EFM8_JDY40_test.c:112: CLKSEL = 0x03;
	mov	_CLKSEL,#0x03
;	EFM8_JDY40_test.c:113: while ((CLKSEL & 0x80) == 0);
L002004?:
	mov	a,_CLKSEL
	jnb	acc.7,L002004?
;	EFM8_JDY40_test.c:118: P0MDOUT |= 0x11; // Enable UART0 TX (P0.4) and UART1 TX (P0.0) as push-pull outputs
	orl	_P0MDOUT,#0x11
;	EFM8_JDY40_test.c:119: P2MDOUT |= 0x01; // P2.0 in push-pull mode
	orl	_P2MDOUT,#0x01
;	EFM8_JDY40_test.c:120: XBR0     = 0x01; // Enable UART0 on P0.4(TX) and P0.5(RX)                     
	mov	_XBR0,#0x01
;	EFM8_JDY40_test.c:121: XBR1     = 0X00;
	mov	_XBR1,#0x00
;	EFM8_JDY40_test.c:122: XBR2     = 0x41; // Enable crossbar and uart 1
	mov	_XBR2,#0x41
;	EFM8_JDY40_test.c:124: P3MDOUT |= 0b10000101;
	orl	_P3MDOUT,#0x85
;	EFM8_JDY40_test.c:125: XBR2     = 0x41; // existing line
	mov	_XBR2,#0x41
;	EFM8_JDY40_test.c:126: P3MDOUT |= 0b10000101;  // <== Add this
	orl	_P3MDOUT,#0x85
;	EFM8_JDY40_test.c:132: SCON0 = 0x10;
	mov	_SCON0,#0x10
;	EFM8_JDY40_test.c:133: TH1 = 0x100-((SYSCLK/BAUDRATE)/(2L*12L));
	mov	_TH1,#0xE6
;	EFM8_JDY40_test.c:134: TL1 = TH1;      // Init Timer1
	mov	_TL1,_TH1
;	EFM8_JDY40_test.c:135: TMOD &= ~0xf0;  // TMOD: timer 1 in 8-bit auto-reload
	anl	_TMOD,#0x0F
;	EFM8_JDY40_test.c:136: TMOD |=  0x20;                       
	orl	_TMOD,#0x20
;	EFM8_JDY40_test.c:137: TR1 = 1; // START Timer1
	setb	_TR1
;	EFM8_JDY40_test.c:138: TI = 1;  // Indicate TX0 ready
	setb	_TI
;	EFM8_JDY40_test.c:142: SFRPAGE=0x10;
	mov	_SFRPAGE,#0x10
;	EFM8_JDY40_test.c:143: TMR4CN0=0x00;   // Stop Timer4; Clear TF4; WARNING: lives in SFR page 0x10
	mov	_TMR4CN0,#0x00
;	EFM8_JDY40_test.c:144: CKCON1|=0b_0000_0001; // Timer 4 uses the system clock
	orl	_CKCON1,#0x01
;	EFM8_JDY40_test.c:145: TMR4RL = TIMER4_RELOAD;
	mov	_TMR4RL,#0xA8
	mov	(_TMR4RL >> 8),#0xFD
;	EFM8_JDY40_test.c:146: TMR4=0xffff;   // Set to reload immediately
	mov	_TMR4,#0xFF
	mov	(_TMR4 >> 8),#0xFF
;	EFM8_JDY40_test.c:147: EIE2|=0b_0000_0100;     // Enable Timer4 interrupts
	orl	_EIE2,#0x04
;	EFM8_JDY40_test.c:148: TR4=1;
	setb	_TR4
;	EFM8_JDY40_test.c:149: EA=1;
	setb	_EA
;	EFM8_JDY40_test.c:152: TMR2CN0=0x00;   // Stop Timer2; Clear TF2;
	mov	_TMR2CN0,#0x00
;	EFM8_JDY40_test.c:153: CKCON0|=0b_0001_0000; // Timer 2 uses the system clock
	orl	_CKCON0,#0x10
;	EFM8_JDY40_test.c:154: TMR2RL=TIMER2_RELOAD; // Initialize reload value
	mov	_TMR2RL,#0xA8
	mov	(_TMR2RL >> 8),#0xFD
;	EFM8_JDY40_test.c:155: TMR2=0xffff;   // Set to reload immediately
	mov	_TMR2,#0xFF
	mov	(_TMR2 >> 8),#0xFF
;	EFM8_JDY40_test.c:156: ET2=1;         // Enable Timer2 interrupts
	setb	_ET2
;	EFM8_JDY40_test.c:157: TR2=1;         // Start Timer2 (TMR2CN is bit addressable)
	setb	_TR2
;	EFM8_JDY40_test.c:158: SFRPAGE=0x00;
	mov	_SFRPAGE,#0x00
;	EFM8_JDY40_test.c:161: SFRPAGE=0x10;
	mov	_SFRPAGE,#0x10
;	EFM8_JDY40_test.c:162: TMR5CN0=0x00;
	mov	_TMR5CN0,#0x00
;	EFM8_JDY40_test.c:163: TMR5=0xffff;   // Set to reload immediately
	mov	_TMR5,#0xFF
	mov	(_TMR5 >> 8),#0xFF
;	EFM8_JDY40_test.c:164: EIE2|=0b_0000_1000; // Enable Timer5 interrupts
	orl	_EIE2,#0x08
;	EFM8_JDY40_test.c:165: TR5=1;         // Start Timer5 (TMR5CN0 is bit addressable)
	setb	_TR5
;	EFM8_JDY40_test.c:167: EA=1;
	setb	_EA
;	EFM8_JDY40_test.c:169: SFRPAGE=0x00;
	mov	_SFRPAGE,#0x00
;	EFM8_JDY40_test.c:172: return 0;
	mov	dpl,#0x00
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'Timer5_ISR'
;------------------------------------------------------------
;------------------------------------------------------------
;	EFM8_JDY40_test.c:175: void Timer5_ISR (void) interrupt INTERRUPT_TIMER5
;	-----------------------------------------
;	 function Timer5_ISR
;	-----------------------------------------
_Timer5_ISR:
	push	acc
	push	ar2
	push	ar3
	push	psw
	mov	psw,#0x00
;	EFM8_JDY40_test.c:177: SFRPAGE=0x10;
	mov	_SFRPAGE,#0x10
;	EFM8_JDY40_test.c:178: TF5H = 0; // Clear Timer5 interrupt flag
	clr	_TF5H
;	EFM8_JDY40_test.c:179: TMR5RL=(0x10000L-(SYSCLK/(12L*100000L)));
	mov	_TMR5RL,#0xC4
	mov	(_TMR5RL >> 8),#0xFF
;	EFM8_JDY40_test.c:180: servo_counter++;
	mov	a,#0x01
	add	a,_servo_counter
	mov	_servo_counter,a
	clr	a
	addc	a,(_servo_counter + 1)
	mov	(_servo_counter + 1),a
;	EFM8_JDY40_test.c:181: if(servo_counter==2000)
	mov	a,#0xD0
	cjne	a,_servo_counter,L003002?
	mov	a,#0x07
	cjne	a,(_servo_counter + 1),L003002?
;	EFM8_JDY40_test.c:183: servo_counter=0;
	clr	a
	mov	_servo_counter,a
	mov	(_servo_counter + 1),a
L003002?:
;	EFM8_JDY40_test.c:185: if(servo1>=servo_counter)
	mov	r2,_servo1
	mov	r3,#0x00
	clr	c
	mov	a,r2
	subb	a,_servo_counter
	mov	a,r3
	subb	a,(_servo_counter + 1)
	jc	L003004?
;	EFM8_JDY40_test.c:187: SERVO1=1;
	setb	_P1_3
	sjmp	L003005?
L003004?:
;	EFM8_JDY40_test.c:191: SERVO1=0;
	clr	_P1_3
L003005?:
;	EFM8_JDY40_test.c:193: if(servo2>=servo_counter)
	mov	r2,_servo2
	mov	r3,#0x00
	clr	c
	mov	a,r2
	subb	a,_servo_counter
	mov	a,r3
	subb	a,(_servo_counter + 1)
	jc	L003007?
;	EFM8_JDY40_test.c:195: SERVO2=1;
	setb	_P1_4
	sjmp	L003009?
L003007?:
;	EFM8_JDY40_test.c:199: SERVO2=0;
	clr	_P1_4
L003009?:
	pop	psw
	pop	ar3
	pop	ar2
	pop	acc
	reti
;	eliminated unneeded push/pop dpl
;	eliminated unneeded push/pop dph
;	eliminated unneeded push/pop b
;------------------------------------------------------------
;Allocation info for local variables in function 'Timer3us'
;------------------------------------------------------------
;us                        Allocated to registers r2 
;i                         Allocated to registers r3 
;------------------------------------------------------------
;	EFM8_JDY40_test.c:204: void Timer3us(unsigned char us)
;	-----------------------------------------
;	 function Timer3us
;	-----------------------------------------
_Timer3us:
	mov	r2,dpl
;	EFM8_JDY40_test.c:209: CKCON0|=0b_0100_0000;
	orl	_CKCON0,#0x40
;	EFM8_JDY40_test.c:211: TMR3RL = (-(SYSCLK)/1000000L); // Set Timer3 to overflow in 1us.
	mov	_TMR3RL,#0xB8
	mov	(_TMR3RL >> 8),#0xFF
;	EFM8_JDY40_test.c:212: TMR3 = TMR3RL;                 // Initialize Timer3 for first overflow
	mov	_TMR3,_TMR3RL
	mov	(_TMR3 >> 8),(_TMR3RL >> 8)
;	EFM8_JDY40_test.c:214: TMR3CN0 = 0x04;                 // Sart Timer3 and clear overflow flag
	mov	_TMR3CN0,#0x04
;	EFM8_JDY40_test.c:215: for (i = 0; i < us; i++)       // Count <us> overflows
	mov	r3,#0x00
L004004?:
	clr	c
	mov	a,r3
	subb	a,r2
	jnc	L004007?
;	EFM8_JDY40_test.c:217: while (!(TMR3CN0 & 0x80));  // Wait for overflow
L004001?:
	mov	a,_TMR3CN0
	jnb	acc.7,L004001?
;	EFM8_JDY40_test.c:218: TMR3CN0 &= ~(0x80);         // Clear overflow indicator
	anl	_TMR3CN0,#0x7F
;	EFM8_JDY40_test.c:215: for (i = 0; i < us; i++)       // Count <us> overflows
	inc	r3
	sjmp	L004004?
L004007?:
;	EFM8_JDY40_test.c:220: TMR3CN0 = 0 ;                   // Stop Timer3 and clear overflow flag
	mov	_TMR3CN0,#0x00
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'waitms'
;------------------------------------------------------------
;ms                        Allocated to registers r2 r3 
;j                         Allocated to registers r4 r5 
;k                         Allocated to registers r6 
;------------------------------------------------------------
;	EFM8_JDY40_test.c:224: void waitms (unsigned int ms)
;	-----------------------------------------
;	 function waitms
;	-----------------------------------------
_waitms:
	mov	r2,dpl
	mov	r3,dph
;	EFM8_JDY40_test.c:228: for(j=0; j<ms; j++)
	mov	r4,#0x00
	mov	r5,#0x00
L005005?:
	clr	c
	mov	a,r4
	subb	a,r2
	mov	a,r5
	subb	a,r3
	jnc	L005009?
;	EFM8_JDY40_test.c:229: for (k=0; k<4; k++) Timer3us(250);
	mov	r6,#0x00
L005001?:
	cjne	r6,#0x04,L005018?
L005018?:
	jnc	L005007?
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
	sjmp	L005001?
L005007?:
;	EFM8_JDY40_test.c:228: for(j=0; j<ms; j++)
	inc	r4
	cjne	r4,#0x00,L005005?
	inc	r5
	sjmp	L005005?
L005009?:
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'UART1_Init'
;------------------------------------------------------------
;baudrate                  Allocated to registers r2 r3 r4 r5 
;------------------------------------------------------------
;	EFM8_JDY40_test.c:232: void UART1_Init (unsigned long baudrate)
;	-----------------------------------------
;	 function UART1_Init
;	-----------------------------------------
_UART1_Init:
	mov	r2,dpl
	mov	r3,dph
	mov	r4,b
	mov	r5,a
;	EFM8_JDY40_test.c:234: SFRPAGE = 0x20;
	mov	_SFRPAGE,#0x20
;	EFM8_JDY40_test.c:235: SMOD1 = 0x0C; // no parity, 8 data bits, 1 stop bit
	mov	_SMOD1,#0x0C
;	EFM8_JDY40_test.c:236: SCON1 = 0x10;
	mov	_SCON1,#0x10
;	EFM8_JDY40_test.c:237: SBCON1 =0x00;   // disable baud rate generator
	mov	_SBCON1,#0x00
;	EFM8_JDY40_test.c:238: SBRL1 = 0x10000L-((SYSCLK/baudrate)/(12L*2L));
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
;	EFM8_JDY40_test.c:239: TI1 = 1; // indicate ready for TX
	setb	_TI1
;	EFM8_JDY40_test.c:240: SBCON1 |= 0x40;   // enable baud rate generator
	orl	_SBCON1,#0x40
;	EFM8_JDY40_test.c:241: SFRPAGE = 0x00;
	mov	_SFRPAGE,#0x00
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'putchar1'
;------------------------------------------------------------
;c                         Allocated to registers r2 
;------------------------------------------------------------
;	EFM8_JDY40_test.c:244: void putchar1 (char c) 
;	-----------------------------------------
;	 function putchar1
;	-----------------------------------------
_putchar1:
	mov	r2,dpl
;	EFM8_JDY40_test.c:246: SFRPAGE = 0x20;
	mov	_SFRPAGE,#0x20
;	EFM8_JDY40_test.c:247: while (!TI1);
L007001?:
;	EFM8_JDY40_test.c:248: TI1=0;
	jbc	_TI1,L007008?
	sjmp	L007001?
L007008?:
;	EFM8_JDY40_test.c:249: SBUF1 = c;
	mov	_SBUF1,r2
;	EFM8_JDY40_test.c:250: SFRPAGE = 0x00;
	mov	_SFRPAGE,#0x00
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'sendstr1'
;------------------------------------------------------------
;s                         Allocated to registers r2 r3 r4 
;------------------------------------------------------------
;	EFM8_JDY40_test.c:253: void sendstr1 (char * s)
;	-----------------------------------------
;	 function sendstr1
;	-----------------------------------------
_sendstr1:
	mov	r2,dpl
	mov	r3,dph
	mov	r4,b
;	EFM8_JDY40_test.c:255: while(*s)
L008001?:
	mov	dpl,r2
	mov	dph,r3
	mov	b,r4
	lcall	__gptrget
	mov	r5,a
	jz	L008004?
;	EFM8_JDY40_test.c:257: putchar1(*s);
	mov	dpl,r5
	push	ar2
	push	ar3
	push	ar4
	lcall	_putchar1
	pop	ar4
	pop	ar3
	pop	ar2
;	EFM8_JDY40_test.c:258: s++;	
	inc	r2
	cjne	r2,#0x00,L008001?
	inc	r3
	sjmp	L008001?
L008004?:
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'getchar1'
;------------------------------------------------------------
;c                         Allocated to registers 
;------------------------------------------------------------
;	EFM8_JDY40_test.c:262: char getchar1 (void)
;	-----------------------------------------
;	 function getchar1
;	-----------------------------------------
_getchar1:
;	EFM8_JDY40_test.c:265: SFRPAGE = 0x20;
	mov	_SFRPAGE,#0x20
;	EFM8_JDY40_test.c:266: while (!RI1);
L009001?:
;	EFM8_JDY40_test.c:267: RI1=0;
	jbc	_RI1,L009008?
	sjmp	L009001?
L009008?:
;	EFM8_JDY40_test.c:269: SCON1&=0b_0011_1111;
	anl	_SCON1,#0x3F
;	EFM8_JDY40_test.c:270: c = SBUF1;
	mov	dpl,_SBUF1
;	EFM8_JDY40_test.c:271: SFRPAGE = 0x00;
	mov	_SFRPAGE,#0x00
;	EFM8_JDY40_test.c:272: return (c);
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'getchar1_with_timeout'
;------------------------------------------------------------
;c                         Allocated to registers 
;timeout                   Allocated to registers r2 r3 
;------------------------------------------------------------
;	EFM8_JDY40_test.c:275: char getchar1_with_timeout (void)
;	-----------------------------------------
;	 function getchar1_with_timeout
;	-----------------------------------------
_getchar1_with_timeout:
;	EFM8_JDY40_test.c:279: SFRPAGE = 0x20;
	mov	_SFRPAGE,#0x20
;	EFM8_JDY40_test.c:281: while (!RI1)
	mov	r2,#0x00
	mov	r3,#0x00
L010003?:
	jb	_RI1,L010005?
;	EFM8_JDY40_test.c:283: SFRPAGE = 0x00;
	mov	_SFRPAGE,#0x00
;	EFM8_JDY40_test.c:284: Timer3us(20);
	mov	dpl,#0x14
	push	ar2
	push	ar3
	lcall	_Timer3us
	pop	ar3
	pop	ar2
;	EFM8_JDY40_test.c:285: SFRPAGE = 0x20;
	mov	_SFRPAGE,#0x20
;	EFM8_JDY40_test.c:286: timeout++;
	inc	r2
	cjne	r2,#0x00,L010012?
	inc	r3
L010012?:
;	EFM8_JDY40_test.c:287: if(timeout==25000)
	cjne	r2,#0xA8,L010003?
	cjne	r3,#0x61,L010003?
;	EFM8_JDY40_test.c:289: SFRPAGE = 0x00;
	mov	_SFRPAGE,#0x00
;	EFM8_JDY40_test.c:290: return ('\n'); // Timeout after half second
	mov	dpl,#0x0A
	ret
L010005?:
;	EFM8_JDY40_test.c:293: RI1=0;
	clr	_RI1
;	EFM8_JDY40_test.c:295: SCON1&=0b_0011_1111;
	anl	_SCON1,#0x3F
;	EFM8_JDY40_test.c:296: c = SBUF1;
	mov	dpl,_SBUF1
;	EFM8_JDY40_test.c:297: SFRPAGE = 0x00;
	mov	_SFRPAGE,#0x00
;	EFM8_JDY40_test.c:298: return (c);
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'getstr1'
;------------------------------------------------------------
;n                         Allocated with name '_getstr1_PARM_2'
;s                         Allocated to registers r2 r3 r4 
;c                         Allocated to registers r1 
;cnt                       Allocated to registers r5 
;------------------------------------------------------------
;	EFM8_JDY40_test.c:301: void getstr1 (char * s, unsigned char n)
;	-----------------------------------------
;	 function getstr1
;	-----------------------------------------
_getstr1:
	mov	r2,dpl
	mov	r3,dph
	mov	r4,b
;	EFM8_JDY40_test.c:307: while(1)
	mov	r5,#0x00
	mov	ar6,r2
	mov	ar7,r3
	mov	ar0,r4
L011007?:
;	EFM8_JDY40_test.c:309: c=getchar1_with_timeout();
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
;	EFM8_JDY40_test.c:310: if(c=='\n')
	cjne	r1,#0x0A,L011002?
;	EFM8_JDY40_test.c:312: *s=0;
	mov	dpl,r2
	mov	dph,r3
	mov	b,r4
	clr	a
;	EFM8_JDY40_test.c:313: return;
	ljmp	__gptrput
L011002?:
;	EFM8_JDY40_test.c:316: if (cnt<n)
	clr	c
	mov	a,r5
	subb	a,_getstr1_PARM_2
	jnc	L011004?
;	EFM8_JDY40_test.c:318: cnt++;
	inc	r5
;	EFM8_JDY40_test.c:319: *s=c;
	mov	dpl,r6
	mov	dph,r7
	mov	b,r0
	mov	a,r1
	lcall	__gptrput
	inc	dptr
	mov	r6,dpl
	mov	r7,dph
;	EFM8_JDY40_test.c:320: s++;
	mov	ar2,r6
	mov	ar3,r7
	mov	ar4,r0
	sjmp	L011007?
L011004?:
;	EFM8_JDY40_test.c:324: *s=0;
	mov	dpl,r2
	mov	dph,r3
	mov	b,r4
	clr	a
;	EFM8_JDY40_test.c:325: return;
	ljmp	__gptrput
;------------------------------------------------------------
;Allocation info for local variables in function 'RXU1'
;------------------------------------------------------------
;------------------------------------------------------------
;	EFM8_JDY40_test.c:331: bit RXU1 (void)
;	-----------------------------------------
;	 function RXU1
;	-----------------------------------------
_RXU1:
;	EFM8_JDY40_test.c:334: SFRPAGE = 0x20;
	mov	_SFRPAGE,#0x20
;	EFM8_JDY40_test.c:335: mybit=RI1;
	mov	c,_RI1
;	EFM8_JDY40_test.c:336: SFRPAGE = 0x00;
	mov	_SFRPAGE,#0x00
;	EFM8_JDY40_test.c:337: return mybit;
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'waitms_or_RI1'
;------------------------------------------------------------
;ms                        Allocated to registers r2 r3 
;j                         Allocated to registers r4 r5 
;k                         Allocated to registers r6 
;------------------------------------------------------------
;	EFM8_JDY40_test.c:340: void waitms_or_RI1 (unsigned int ms)
;	-----------------------------------------
;	 function waitms_or_RI1
;	-----------------------------------------
_waitms_or_RI1:
	mov	r2,dpl
	mov	r3,dph
;	EFM8_JDY40_test.c:344: for(j=0; j<ms; j++)
	mov	r4,#0x00
	mov	r5,#0x00
L013007?:
	clr	c
	mov	a,r4
	subb	a,r2
	mov	a,r5
	subb	a,r3
	jnc	L013011?
;	EFM8_JDY40_test.c:346: for (k=0; k<4; k++)
	mov	r6,#0x00
L013003?:
	cjne	r6,#0x04,L013019?
L013019?:
	jnc	L013009?
;	EFM8_JDY40_test.c:348: if(RXU1()) return;
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
	jz	L013002?
	ret
L013002?:
;	EFM8_JDY40_test.c:349: Timer3us(250);
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
;	EFM8_JDY40_test.c:346: for (k=0; k<4; k++)
	inc	r6
	sjmp	L013003?
L013009?:
;	EFM8_JDY40_test.c:344: for(j=0; j<ms; j++)
	inc	r4
	cjne	r4,#0x00,L013007?
	inc	r5
	sjmp	L013007?
L013011?:
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'SendATCommand'
;------------------------------------------------------------
;s                         Allocated to registers r2 r3 r4 
;------------------------------------------------------------
;	EFM8_JDY40_test.c:354: void SendATCommand (char * s)
;	-----------------------------------------
;	 function SendATCommand
;	-----------------------------------------
_SendATCommand:
	mov	r2,dpl
	mov	r3,dph
	mov	r4,b
;	EFM8_JDY40_test.c:356: printf("Command: %s", s);
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
;	EFM8_JDY40_test.c:357: P2_0=0; // 'set' pin to 0 is 'AT' mode.
	clr	_P2_0
;	EFM8_JDY40_test.c:358: waitms(5);
	mov	dptr,#0x0005
	lcall	_waitms
	pop	ar4
	pop	ar3
	pop	ar2
;	EFM8_JDY40_test.c:359: sendstr1(s);
	mov	dpl,r2
	mov	dph,r3
	mov	b,r4
	lcall	_sendstr1
;	EFM8_JDY40_test.c:360: getstr1(buff, sizeof(buff)-1);
	mov	_getstr1_PARM_2,#0x13
	mov	dptr,#_buff
	mov	b,#0x40
	lcall	_getstr1
;	EFM8_JDY40_test.c:361: waitms(10);
	mov	dptr,#0x000A
	lcall	_waitms
;	EFM8_JDY40_test.c:362: P2_0=1; // 'set' pin to 1 is normal operation mode.
	setb	_P2_0
;	EFM8_JDY40_test.c:363: printf("Response: %s\r\n", buff);
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
;	EFM8_JDY40_test.c:366: void ReceptionOff (void)
;	-----------------------------------------
;	 function ReceptionOff
;	-----------------------------------------
_ReceptionOff:
;	EFM8_JDY40_test.c:368: P2_0=0; // 'set' pin to 0 is 'AT' mode.
	clr	_P2_0
;	EFM8_JDY40_test.c:369: waitms(10);
	mov	dptr,#0x000A
	lcall	_waitms
;	EFM8_JDY40_test.c:370: sendstr1("AT+DVID0000\r\n"); // Some unused id, so that we get nothing in RXD1.
	mov	dptr,#__str_2
	mov	b,#0x80
	lcall	_sendstr1
;	EFM8_JDY40_test.c:371: waitms(10);
	mov	dptr,#0x000A
	lcall	_waitms
;	EFM8_JDY40_test.c:373: SCON1&=0b_0011_1111;
	anl	_SCON1,#0x3F
;	EFM8_JDY40_test.c:374: P2_0=1; // 'set' pin to 1 is normal operation mode.
	setb	_P2_0
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'TIMER0_Init'
;------------------------------------------------------------
;------------------------------------------------------------
;	EFM8_JDY40_test.c:378: void TIMER0_Init(void)
;	-----------------------------------------
;	 function TIMER0_Init
;	-----------------------------------------
_TIMER0_Init:
;	EFM8_JDY40_test.c:380: TMOD&=0b_1111_0000; // Set the bits of Timer/Counter 0 to zero
	anl	_TMOD,#0xF0
;	EFM8_JDY40_test.c:381: TMOD|=0b_0000_0001; // Timer/Counter 0 used as a 16-bit timer
	orl	_TMOD,#0x01
;	EFM8_JDY40_test.c:382: TR0=0; // Stop Timer/Counter 0
	clr	_TR0
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'InitADC'
;------------------------------------------------------------
;------------------------------------------------------------
;	EFM8_JDY40_test.c:386: void InitADC (void)
;	-----------------------------------------
;	 function InitADC
;	-----------------------------------------
_InitADC:
;	EFM8_JDY40_test.c:388: SFRPAGE = 0x00;
	mov	_SFRPAGE,#0x00
;	EFM8_JDY40_test.c:389: ADEN=0; // Disable ADC
	clr	_ADEN
;	EFM8_JDY40_test.c:394: (0x0 << 0) ; // Accumulate n conversions: 0x0: 1, 0x1:4, 0x2:8, 0x3:16, 0x4:32
	mov	_ADC0CN1,#0x80
;	EFM8_JDY40_test.c:398: (0x0 << 2); // 0:SYSCLK ADCCLK = SYSCLK. 1:HFOSC0 ADCCLK = HFOSC0.
	mov	_ADC0CF0,#0x20
;	EFM8_JDY40_test.c:402: (0x1E << 0); // Conversion Tracking Time. Tadtk = ADTK / (Fsarclk)
	mov	_ADC0CF1,#0x1E
;	EFM8_JDY40_test.c:411: (0x0 << 0) ; // TEMPE. 0: Disable the Temperature Sensor. 1: Enable the Temperature Sensor.
	mov	_ADC0CN0,#0x00
;	EFM8_JDY40_test.c:416: (0x1F << 0); // ADPWR. Power Up Delay Time. Tpwrtime = ((4 * (ADPWR + 1)) + 2) / (Fadcclk)
	mov	_ADC0CF2,#0x3F
;	EFM8_JDY40_test.c:420: (0x0 << 0) ; // ADCM. 0x0: ADBUSY, 0x1: TIMER0, 0x2: TIMER2, 0x3: TIMER3, 0x4: CNVSTR, 0x5: CEX5, 0x6: TIMER4, 0x7: TIMER5, 0x8: CLU0, 0x9: CLU1, 0xA: CLU2, 0xB: CLU3
	mov	_ADC0CN2,#0x00
;	EFM8_JDY40_test.c:422: ADEN=1; // Enable ADC
	setb	_ADEN
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'InitPinADC'
;------------------------------------------------------------
;pinno                     Allocated with name '_InitPinADC_PARM_2'
;portno                    Allocated to registers r2 
;mask                      Allocated to registers r3 
;------------------------------------------------------------
;	EFM8_JDY40_test.c:425: void InitPinADC (unsigned char portno, unsigned char pinno)
;	-----------------------------------------
;	 function InitPinADC
;	-----------------------------------------
_InitPinADC:
	mov	r2,dpl
;	EFM8_JDY40_test.c:429: mask=1<<pinno;
	mov	b,_InitPinADC_PARM_2
	inc	b
	mov	a,#0x01
	sjmp	L018013?
L018011?:
	add	a,acc
L018013?:
	djnz	b,L018011?
	mov	r3,a
;	EFM8_JDY40_test.c:431: SFRPAGE = 0x20;
	mov	_SFRPAGE,#0x20
;	EFM8_JDY40_test.c:432: switch (portno)
	cjne	r2,#0x00,L018014?
	sjmp	L018001?
L018014?:
	cjne	r2,#0x01,L018015?
	sjmp	L018002?
L018015?:
;	EFM8_JDY40_test.c:434: case 0:
	cjne	r2,#0x02,L018005?
	sjmp	L018003?
L018001?:
;	EFM8_JDY40_test.c:435: P0MDIN &= (~mask); // Set pin as analog input
	mov	a,r3
	cpl	a
	mov	r2,a
	anl	_P0MDIN,a
;	EFM8_JDY40_test.c:436: P0SKIP |= mask; // Skip Crossbar decoding for this pin
	mov	a,r3
	orl	_P0SKIP,a
;	EFM8_JDY40_test.c:437: break;
;	EFM8_JDY40_test.c:438: case 1:
	sjmp	L018005?
L018002?:
;	EFM8_JDY40_test.c:439: P1MDIN &= (~mask); // Set pin as analog input
	mov	a,r3
	cpl	a
	mov	r2,a
	anl	_P1MDIN,a
;	EFM8_JDY40_test.c:440: P1SKIP |= mask; // Skip Crossbar decoding for this pin
	mov	a,r3
	orl	_P1SKIP,a
;	EFM8_JDY40_test.c:441: break;
;	EFM8_JDY40_test.c:442: case 2:
	sjmp	L018005?
L018003?:
;	EFM8_JDY40_test.c:443: P2MDIN &= (~mask); // Set pin as analog input
	mov	a,r3
	cpl	a
	mov	r2,a
	anl	_P2MDIN,a
;	EFM8_JDY40_test.c:444: P2SKIP |= mask; // Skip Crossbar decoding for this pin
	mov	a,r3
	orl	_P2SKIP,a
;	EFM8_JDY40_test.c:448: }
L018005?:
;	EFM8_JDY40_test.c:449: SFRPAGE = 0x00;
	mov	_SFRPAGE,#0x00
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'ADC_at_Pin'
;------------------------------------------------------------
;pin                       Allocated to registers 
;------------------------------------------------------------
;	EFM8_JDY40_test.c:452: unsigned int ADC_at_Pin(unsigned char pin)
;	-----------------------------------------
;	 function ADC_at_Pin
;	-----------------------------------------
_ADC_at_Pin:
	mov	_ADC0MX,dpl
;	EFM8_JDY40_test.c:455: ADINT = 0;
	clr	_ADINT
;	EFM8_JDY40_test.c:456: ADBUSY = 1;     // Convert voltage at the pin
	setb	_ADBUSY
;	EFM8_JDY40_test.c:457: while (!ADINT); // Wait for conversion to complete
L019001?:
	jnb	_ADINT,L019001?
;	EFM8_JDY40_test.c:458: return (ADC0);
	mov	dpl,_ADC0
	mov	dph,(_ADC0 >> 8)
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'Volts_at_Pin'
;------------------------------------------------------------
;pin                       Allocated to registers r2 
;------------------------------------------------------------
;	EFM8_JDY40_test.c:461: float Volts_at_Pin(unsigned char pin)
;	-----------------------------------------
;	 function Volts_at_Pin
;	-----------------------------------------
_Volts_at_Pin:
;	EFM8_JDY40_test.c:463: return ((ADC_at_Pin(pin)*VDD)/0b_0011_1111_1111_1111);
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
;	EFM8_JDY40_test.c:467: void Timer4_ISR (void) interrupt INTERRUPT_TIMER4
;	-----------------------------------------
;	 function Timer4_ISR
;	-----------------------------------------
_Timer4_ISR:
	push	acc
	push	psw
	mov	psw,#0x00
;	EFM8_JDY40_test.c:469: SFRPAGE=0x10;
	mov	_SFRPAGE,#0x10
;	EFM8_JDY40_test.c:470: TF4H = 0; 
	clr	_TF4H
;	EFM8_JDY40_test.c:472: pwm_counter4 += 256; // counting steps
	mov	_pwm_counter4,_pwm_counter4
	mov	a,#0x01
	add	a,(_pwm_counter4 + 1)
	mov	(_pwm_counter4 + 1),a
;	EFM8_JDY40_test.c:473: if ( direction == 1) {
	mov	a,#0x01
	cjne	a,_direction,L021009?
	clr	a
	cjne	a,(_direction + 1),L021009?
	sjmp	L021010?
L021009?:
	sjmp	L021002?
L021010?:
;	EFM8_JDY40_test.c:474: PWMOUT4 = (pwm_counter4 < pwm_duty4) ? 1 : 0;
	clr	c
	mov	a,_pwm_counter4
	subb	a,_pwm_duty4
	mov	a,(_pwm_counter4 + 1)
	subb	a,(_pwm_duty4 + 1)
	mov	_P3_0,c
L021002?:
;	EFM8_JDY40_test.c:476: if ( direction  == 0 ) {
	mov	a,_direction
	orl	a,(_direction + 1)
	jnz	L021004?
;	EFM8_JDY40_test.c:477: PWMOUT4R = (pwm_counter4 < pwm_duty4) ? 1 : 0;
	clr	c
	mov	a,_pwm_counter4
	subb	a,_pwm_duty4
	mov	a,(_pwm_counter4 + 1)
	subb	a,(_pwm_duty4 + 1)
	mov	_P2_5,c
L021004?:
;	EFM8_JDY40_test.c:479: if ( direction == 3 ) 
	mov	a,_direction
	mov	a,(_direction + 1)
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
;	EFM8_JDY40_test.c:484: void Timer2_ISR (void) interrupt INTERRUPT_TIMER2
;	-----------------------------------------
;	 function Timer2_ISR
;	-----------------------------------------
_Timer2_ISR:
	push	acc
	push	psw
	mov	psw,#0x00
;	EFM8_JDY40_test.c:486: SFRPAGE=0x0;
	mov	_SFRPAGE,#0x00
;	EFM8_JDY40_test.c:487: TF2H = 0; // Clear Timer2 interrupt flag
	clr	_TF2H
;	EFM8_JDY40_test.c:488: pwm_counter2 += 256; // counting steps
	mov	_pwm_counter2,_pwm_counter2
	mov	a,#0x01
	add	a,(_pwm_counter2 + 1)
	mov	(_pwm_counter2 + 1),a
;	EFM8_JDY40_test.c:489: if ( direction == 1) {
	mov	a,#0x01
	cjne	a,_direction,L022009?
	clr	a
	cjne	a,(_direction + 1),L022009?
	sjmp	L022010?
L022009?:
	sjmp	L022002?
L022010?:
;	EFM8_JDY40_test.c:490: PWMOUT2 = (pwm_counter2 < pwm_duty2) ? 1 : 0; ////////////////////////change this to pwm_duty2 later on 
	clr	c
	mov	a,_pwm_counter2
	subb	a,_pwm_duty2
	mov	a,(_pwm_counter2 + 1)
	subb	a,(_pwm_duty2 + 1)
	mov	_P3_2,c
L022002?:
;	EFM8_JDY40_test.c:492: if (direction == 0 ){
	mov	a,_direction
	orl	a,(_direction + 1)
	jnz	L022004?
;	EFM8_JDY40_test.c:493: PWMOUT2R = (pwm_counter2 < pwm_duty2) ? 1 : 0;
	clr	c
	mov	a,_pwm_counter2
	subb	a,_pwm_duty2
	mov	a,(_pwm_counter2 + 1)
	subb	a,(_pwm_duty2 + 1)
	mov	_P3_7,c
L022004?:
;	EFM8_JDY40_test.c:495: if ( direction == 3 ) 
	mov	a,_direction
	mov	a,(_direction + 1)
	pop	psw
	pop	acc
	reti
;	eliminated unneeded push/pop dpl
;	eliminated unneeded push/pop dph
;	eliminated unneeded push/pop b
;------------------------------------------------------------
;Allocation info for local variables in function 'GetPeriod'
;------------------------------------------------------------
;n                         Allocated to registers r2 r3 
;overflow_count            Allocated to registers r4 r5 
;i                         Allocated to registers r6 
;------------------------------------------------------------
;	EFM8_JDY40_test.c:502: unsigned long GetPeriod (int n)
;	-----------------------------------------
;	 function GetPeriod
;	-----------------------------------------
_GetPeriod:
	mov	r2,dpl
	mov	r3,dph
;	EFM8_JDY40_test.c:507: TR0=0; // Stop Timer/Counter 0
	clr	_TR0
;	EFM8_JDY40_test.c:508: TMOD&=0b_1111_0000; // Set the bits of Timer/Counter 0 to zero
	anl	_TMOD,#0xF0
;	EFM8_JDY40_test.c:509: TMOD|=0b_0000_0001; // Timer/Counter 0 used as a 16-bit timer
	orl	_TMOD,#0x01
;	EFM8_JDY40_test.c:512: TR0=0;
	clr	_TR0
;	EFM8_JDY40_test.c:513: TL0=0; TH0=0; TF0=0; overflow_count=0;
	mov	_TL0,#0x00
	mov	_TH0,#0x00
	clr	_TF0
;	EFM8_JDY40_test.c:514: TR0=1;
	setb	_TR0
;	EFM8_JDY40_test.c:517: TR0=0;
	clr	_TR0
;	EFM8_JDY40_test.c:518: TL0=0; TH0=0; TF0=0; overflow_count=0;
	mov	_TL0,#0x00
	mov	_TH0,#0x00
	clr	_TF0
;	EFM8_JDY40_test.c:519: TR0=1;
	setb	_TR0
;	EFM8_JDY40_test.c:520: while(PERIOD_PIN!=0) // Wait for the signal to be zero
	mov	r4,#0x00
	mov	r5,#0x00
L023005?:
	jnb	_P0_6,L023007?
;	EFM8_JDY40_test.c:522: if(TF0==1) // Did the 16-bit timer overflow?
;	EFM8_JDY40_test.c:524: TF0=0;
	jbc	_TF0,L023050?
	sjmp	L023005?
L023050?:
;	EFM8_JDY40_test.c:525: overflow_count++;
	inc	r4
	cjne	r4,#0x00,L023051?
	inc	r5
L023051?:
;	EFM8_JDY40_test.c:526: if(overflow_count==10) // If it overflows too many times assume no signal is present
	cjne	r4,#0x0A,L023005?
	cjne	r5,#0x00,L023005?
;	EFM8_JDY40_test.c:528: TR0=0;
	clr	_TR0
;	EFM8_JDY40_test.c:529: return 0; // No signal
	mov	dptr,#(0x00&0x00ff)
	clr	a
	mov	b,a
	ret
L023007?:
;	EFM8_JDY40_test.c:535: TR0=0;
	clr	_TR0
;	EFM8_JDY40_test.c:536: TL0=0; TH0=0; TF0=0; overflow_count=0;
	mov	_TL0,#0x00
	mov	_TH0,#0x00
	clr	_TF0
;	EFM8_JDY40_test.c:537: TR0=1;
	setb	_TR0
;	EFM8_JDY40_test.c:538: while(PERIOD_PIN!=1) // Wait for the signal to be one
	mov	r4,#0x00
	mov	r5,#0x00
L023012?:
	jb	_P0_6,L023014?
;	EFM8_JDY40_test.c:540: if(TF0==1) // Did the 16-bit timer overflow?
;	EFM8_JDY40_test.c:542: TF0=0;
	jbc	_TF0,L023055?
	sjmp	L023012?
L023055?:
;	EFM8_JDY40_test.c:543: overflow_count++;
	inc	r4
	cjne	r4,#0x00,L023056?
	inc	r5
L023056?:
;	EFM8_JDY40_test.c:544: if(overflow_count==10) // If it overflows too many times assume no signal is present
	cjne	r4,#0x0A,L023012?
	cjne	r5,#0x00,L023012?
;	EFM8_JDY40_test.c:546: TR0=0;
	clr	_TR0
;	EFM8_JDY40_test.c:547: return 0; // No signal
	mov	dptr,#(0x00&0x00ff)
	clr	a
	mov	b,a
	ret
L023014?:
;	EFM8_JDY40_test.c:553: TR0=0;
	clr	_TR0
;	EFM8_JDY40_test.c:554: TL0=0; TH0=0; TF0=0; overflow_count=0;
	mov	_TL0,#0x00
	mov	_TH0,#0x00
	clr	_TF0
	mov	r4,#0x00
	mov	r5,#0x00
;	EFM8_JDY40_test.c:555: TR0=1; // Start the timer
	setb	_TR0
;	EFM8_JDY40_test.c:556: for(i=0; i<n; i++) // Measure the time of 'n' periods
	mov	r6,#0x00
L023025?:
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
	jnc	L023028?
;	EFM8_JDY40_test.c:558: while(PERIOD_PIN!=0) // Wait for the signal to be zero
	mov	ar7,r4
	mov	ar0,r5
L023017?:
	jnb	_P0_6,L023043?
;	EFM8_JDY40_test.c:560: if(TF0==1) // Did the 16-bit timer overflow?
;	EFM8_JDY40_test.c:562: TF0=0;
	jbc	_TF0,L023061?
	sjmp	L023017?
L023061?:
;	EFM8_JDY40_test.c:563: overflow_count++;
	inc	r7
	cjne	r7,#0x00,L023017?
	inc	r0
;	EFM8_JDY40_test.c:566: while(PERIOD_PIN!=1) // Wait for the signal to be one
	sjmp	L023017?
L023043?:
L023022?:
	jb	_P0_6,L023047?
;	EFM8_JDY40_test.c:568: if(TF0==1) // Did the 16-bit timer overflow?
;	EFM8_JDY40_test.c:570: TF0=0;
	jbc	_TF0,L023063?
	sjmp	L023022?
L023063?:
;	EFM8_JDY40_test.c:571: overflow_count++;
	inc	r7
	cjne	r7,#0x00,L023022?
	inc	r0
	sjmp	L023022?
L023047?:
	mov	ar4,r7
	mov	ar5,r0
;	EFM8_JDY40_test.c:556: for(i=0; i<n; i++) // Measure the time of 'n' periods
	inc	r6
	sjmp	L023025?
L023028?:
;	EFM8_JDY40_test.c:575: TR0=0; // Stop timer 0, the 24-bit number [overflow_count-TH0-TL0] has the period in clock cycles!
	clr	_TR0
;	EFM8_JDY40_test.c:577: return (overflow_count*65536+TH0*256+TL0);
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
;	EFM8_JDY40_test.c:580: void eputs(char *String)
;	-----------------------------------------
;	 function eputs
;	-----------------------------------------
_eputs:
	mov	r2,dpl
	mov	r3,dph
	mov	r4,b
;	EFM8_JDY40_test.c:582: while(*String)
L024001?:
	mov	dpl,r2
	mov	dph,r3
	mov	b,r4
	lcall	__gptrget
	mov	r5,a
	jz	L024004?
;	EFM8_JDY40_test.c:584: putchar(*String);
	mov	dpl,r5
	push	ar2
	push	ar3
	push	ar4
	lcall	_putchar
	pop	ar4
	pop	ar3
	pop	ar2
;	EFM8_JDY40_test.c:585: String++;
	inc	r2
	cjne	r2,#0x00,L024001?
	inc	r3
	sjmp	L024001?
L024004?:
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'PrintNumber'
;------------------------------------------------------------
;Base                      Allocated with name '_PrintNumber_PARM_2'
;digits                    Allocated with name '_PrintNumber_PARM_3'
;val                       Allocated with name '_PrintNumber_val_1_163'
;j                         Allocated with name '_PrintNumber_j_1_164'
;sloc0                     Allocated with name '_PrintNumber_sloc0_1_0'
;sloc1                     Allocated with name '_PrintNumber_sloc1_1_0'
;buff                      Allocated with name '_PrintNumber_buff_1_164'
;------------------------------------------------------------
;	EFM8_JDY40_test.c:589: void PrintNumber(long int val, int Base, int digits)
;	-----------------------------------------
;	 function PrintNumber
;	-----------------------------------------
_PrintNumber:
	mov	_PrintNumber_val_1_163,dpl
	mov	(_PrintNumber_val_1_163 + 1),dph
	mov	(_PrintNumber_val_1_163 + 2),b
	mov	(_PrintNumber_val_1_163 + 3),a
;	EFM8_JDY40_test.c:595: buff[NBITS]=0;
	mov	dptr,#(_PrintNumber_buff_1_164 + 0x0020)
	clr	a
	movx	@dptr,a
;	EFM8_JDY40_test.c:597: if(val<0)
	mov	a,(_PrintNumber_val_1_163 + 3)
	jnb	acc.7,L025012?
;	EFM8_JDY40_test.c:599: putchar('-');
	mov	dpl,#0x2D
	lcall	_putchar
;	EFM8_JDY40_test.c:600: val*=-1;
	clr	c
	clr	a
	subb	a,_PrintNumber_val_1_163
	mov	_PrintNumber_val_1_163,a
	clr	a
	subb	a,(_PrintNumber_val_1_163 + 1)
	mov	(_PrintNumber_val_1_163 + 1),a
	clr	a
	subb	a,(_PrintNumber_val_1_163 + 2)
	mov	(_PrintNumber_val_1_163 + 2),a
	clr	a
	subb	a,(_PrintNumber_val_1_163 + 3)
	mov	(_PrintNumber_val_1_163 + 3),a
;	EFM8_JDY40_test.c:604: while ( (val>0) | (digits>0) )
L025012?:
	mov	_PrintNumber_j_1_164,#0x1F
	clr	a
	mov	(_PrintNumber_j_1_164 + 1),a
	mov	r0,_PrintNumber_PARM_3
	mov	r1,(_PrintNumber_PARM_3 + 1)
L025005?:
	clr	c
	clr	a
	subb	a,_PrintNumber_val_1_163
	clr	a
	subb	a,(_PrintNumber_val_1_163 + 1)
	clr	a
	subb	a,(_PrintNumber_val_1_163 + 2)
	clr	a
	xrl	a,#0x80
	mov	b,(_PrintNumber_val_1_163 + 3)
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
	jnz	L025016?
	ljmp	L025007?
L025016?:
;	EFM8_JDY40_test.c:606: buff[j--]=HexDigit[val%Base];
	mov	r6,_PrintNumber_j_1_164
	mov	r7,(_PrintNumber_j_1_164 + 1)
	dec	_PrintNumber_j_1_164
	mov	a,#0xff
	cjne	a,_PrintNumber_j_1_164,L025017?
	dec	(_PrintNumber_j_1_164 + 1)
L025017?:
	mov	a,r6
	add	a,#_PrintNumber_buff_1_164
	mov	_PrintNumber_sloc0_1_0,a
	mov	a,r7
	addc	a,#(_PrintNumber_buff_1_164 >> 8)
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
	mov	dpl,_PrintNumber_val_1_163
	mov	dph,(_PrintNumber_val_1_163 + 1)
	mov	b,(_PrintNumber_val_1_163 + 2)
	mov	a,(_PrintNumber_val_1_163 + 3)
	push	ar0
	push	ar1
	lcall	__modslong
	mov	r6,dpl
	mov	r7,dph
	mov	a,r6
	add	a,#_PrintNumber_HexDigit_1_164
	mov	dpl,a
	mov	a,r7
	addc	a,#(_PrintNumber_HexDigit_1_164 >> 8)
	mov	dph,a
	clr	a
	movc	a,@a+dptr
	mov	dpl,_PrintNumber_sloc0_1_0
	mov	dph,(_PrintNumber_sloc0_1_0 + 1)
	movx	@dptr,a
;	EFM8_JDY40_test.c:607: val/=Base;
	mov	__divslong_PARM_2,_PrintNumber_sloc1_1_0
	mov	(__divslong_PARM_2 + 1),(_PrintNumber_sloc1_1_0 + 1)
	mov	(__divslong_PARM_2 + 2),(_PrintNumber_sloc1_1_0 + 2)
	mov	(__divslong_PARM_2 + 3),(_PrintNumber_sloc1_1_0 + 3)
	mov	dpl,_PrintNumber_val_1_163
	mov	dph,(_PrintNumber_val_1_163 + 1)
	mov	b,(_PrintNumber_val_1_163 + 2)
	mov	a,(_PrintNumber_val_1_163 + 3)
	lcall	__divslong
	mov	_PrintNumber_val_1_163,dpl
	mov	(_PrintNumber_val_1_163 + 1),dph
	mov	(_PrintNumber_val_1_163 + 2),b
	mov	(_PrintNumber_val_1_163 + 3),a
	pop	ar1
	pop	ar0
;	EFM8_JDY40_test.c:608: if(digits!=0) digits--;
	mov	a,r0
	orl	a,r1
	jnz	L025018?
	ljmp	L025005?
L025018?:
	dec	r0
	cjne	r0,#0xff,L025019?
	dec	r1
L025019?:
	ljmp	L025005?
L025007?:
;	EFM8_JDY40_test.c:610: eputs(&buff[j+1]);
	mov	a,_PrintNumber_j_1_164
	inc	a
	add	a,#_PrintNumber_buff_1_164
	mov	r2,a
	clr	a
	addc	a,#(_PrintNumber_buff_1_164 >> 8)
	mov	r3,a
	mov	r4,#0x00
	mov	dpl,r2
	mov	dph,r3
	mov	b,r4
	ljmp	_eputs
;------------------------------------------------------------
;Allocation info for local variables in function 'GetFrequency'
;------------------------------------------------------------
;c                         Allocated to registers r2 r3 r4 r5 
;f                         Allocated to registers r6 r7 r0 r1 
;------------------------------------------------------------
;	EFM8_JDY40_test.c:613: unsigned long GetFrequency (long int c)
;	-----------------------------------------
;	 function GetFrequency
;	-----------------------------------------
_GetFrequency:
	mov	r2,dpl
	mov	r3,dph
	mov	r4,b
	mov	r5,a
;	EFM8_JDY40_test.c:615: long int f = 0;
	mov	r6,#0x00
	mov	r7,#0x00
	mov	r0,#0x00
	mov	r1,#0x00
;	EFM8_JDY40_test.c:617: if(c>0)
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
	jnc	L026002?
;	EFM8_JDY40_test.c:619: f=(SYSCLK*200.0)/(c*12);
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
	sjmp	L026003?
L026002?:
;	EFM8_JDY40_test.c:629: eputs(" NO SIGNAL                     \r");
	mov	dptr,#__str_4
	mov	b,#0x80
	push	ar6
	push	ar7
	push	ar0
	push	ar1
	lcall	_eputs
	pop	ar1
	pop	ar0
	pop	ar7
	pop	ar6
L026003?:
;	EFM8_JDY40_test.c:632: return f;
	mov	dpl,r6
	mov	dph,r7
	mov	b,r0
	mov	a,r1
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'LCD_pulse'
;------------------------------------------------------------
;------------------------------------------------------------
;	EFM8_JDY40_test.c:636: void LCD_pulse (void)
;	-----------------------------------------
;	 function LCD_pulse
;	-----------------------------------------
_LCD_pulse:
;	EFM8_JDY40_test.c:638: LCD_E=1;
	setb	_P2_6
;	EFM8_JDY40_test.c:639: Timer3us(40);
	mov	dpl,#0x28
	lcall	_Timer3us
;	EFM8_JDY40_test.c:640: LCD_E=0;
	clr	_P2_6
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'LCD_byte'
;------------------------------------------------------------
;x                         Allocated to registers r2 
;------------------------------------------------------------
;	EFM8_JDY40_test.c:643: void LCD_byte (unsigned char x)
;	-----------------------------------------
;	 function LCD_byte
;	-----------------------------------------
_LCD_byte:
	mov	r2,dpl
;	EFM8_JDY40_test.c:646: ACC=x; //Send high nible
	mov	_ACC,r2
;	EFM8_JDY40_test.c:647: LCD_D7=ACC_7;
	mov	c,_ACC_7
	mov	_P0_7,c
;	EFM8_JDY40_test.c:648: LCD_D6=ACC_6;
	mov	c,_ACC_6
	mov	_P1_0,c
;	EFM8_JDY40_test.c:649: LCD_D5=ACC_5;
	mov	c,_ACC_5
	mov	_P1_1,c
;	EFM8_JDY40_test.c:650: LCD_D4=ACC_4;
	mov	c,_ACC_4
	mov	_P1_2,c
;	EFM8_JDY40_test.c:651: LCD_pulse();
	push	ar2
	lcall	_LCD_pulse
;	EFM8_JDY40_test.c:652: Timer3us(40);
	mov	dpl,#0x28
	lcall	_Timer3us
	pop	ar2
;	EFM8_JDY40_test.c:653: ACC=x; //Send low nible
	mov	_ACC,r2
;	EFM8_JDY40_test.c:654: LCD_D7=ACC_3;
	mov	c,_ACC_3
	mov	_P0_7,c
;	EFM8_JDY40_test.c:655: LCD_D6=ACC_2;
	mov	c,_ACC_2
	mov	_P1_0,c
;	EFM8_JDY40_test.c:656: LCD_D5=ACC_1;
	mov	c,_ACC_1
	mov	_P1_1,c
;	EFM8_JDY40_test.c:657: LCD_D4=ACC_0;
	mov	c,_ACC_0
	mov	_P1_2,c
;	EFM8_JDY40_test.c:658: LCD_pulse();
	ljmp	_LCD_pulse
;------------------------------------------------------------
;Allocation info for local variables in function 'WriteData'
;------------------------------------------------------------
;x                         Allocated to registers r2 
;------------------------------------------------------------
;	EFM8_JDY40_test.c:661: void WriteData (unsigned char x)
;	-----------------------------------------
;	 function WriteData
;	-----------------------------------------
_WriteData:
	mov	r2,dpl
;	EFM8_JDY40_test.c:663: LCD_RS=1;
	setb	_P2_2
;	EFM8_JDY40_test.c:664: LCD_byte(x);
	mov	dpl,r2
	lcall	_LCD_byte
;	EFM8_JDY40_test.c:665: waitms(2);
	mov	dptr,#0x0002
	ljmp	_waitms
;------------------------------------------------------------
;Allocation info for local variables in function 'WriteCommand'
;------------------------------------------------------------
;x                         Allocated to registers r2 
;------------------------------------------------------------
;	EFM8_JDY40_test.c:668: void WriteCommand (unsigned char x)
;	-----------------------------------------
;	 function WriteCommand
;	-----------------------------------------
_WriteCommand:
	mov	r2,dpl
;	EFM8_JDY40_test.c:670: LCD_RS=0;
	clr	_P2_2
;	EFM8_JDY40_test.c:671: LCD_byte(x);
	mov	dpl,r2
	lcall	_LCD_byte
;	EFM8_JDY40_test.c:672: waitms(5);
	mov	dptr,#0x0005
	ljmp	_waitms
;------------------------------------------------------------
;Allocation info for local variables in function 'LCD_4BIT'
;------------------------------------------------------------
;------------------------------------------------------------
;	EFM8_JDY40_test.c:675: void LCD_4BIT (void)
;	-----------------------------------------
;	 function LCD_4BIT
;	-----------------------------------------
_LCD_4BIT:
;	EFM8_JDY40_test.c:677: LCD_E=0; // Resting state of LCD's enable is zero
	clr	_P2_6
;	EFM8_JDY40_test.c:679: waitms(20);
	mov	dptr,#0x0014
	lcall	_waitms
;	EFM8_JDY40_test.c:681: WriteCommand(0x33);
	mov	dpl,#0x33
	lcall	_WriteCommand
;	EFM8_JDY40_test.c:682: WriteCommand(0x33);
	mov	dpl,#0x33
	lcall	_WriteCommand
;	EFM8_JDY40_test.c:683: WriteCommand(0x32); // Change to 4-bit mode
	mov	dpl,#0x32
	lcall	_WriteCommand
;	EFM8_JDY40_test.c:686: WriteCommand(0x28);
	mov	dpl,#0x28
	lcall	_WriteCommand
;	EFM8_JDY40_test.c:687: WriteCommand(0x0c);
	mov	dpl,#0x0C
	lcall	_WriteCommand
;	EFM8_JDY40_test.c:688: WriteCommand(0x01); // Clear screen command (takes some time)
	mov	dpl,#0x01
	lcall	_WriteCommand
;	EFM8_JDY40_test.c:689: waitms(20); // Wait for clear screen command to finsih.
	mov	dptr,#0x0014
	ljmp	_waitms
;------------------------------------------------------------
;Allocation info for local variables in function 'servomotion'
;------------------------------------------------------------
;j                         Allocated to registers r2 
;i                         Allocated to registers r2 r3 
;------------------------------------------------------------
;	EFM8_JDY40_test.c:693: void servomotion(void)
;	-----------------------------------------
;	 function servomotion
;	-----------------------------------------
_servomotion:
;	EFM8_JDY40_test.c:698: WriteCommand(0x40);  // Set CGRAM address
	mov	dpl,#0x40
	lcall	_WriteCommand
;	EFM8_JDY40_test.c:699: for(i=0; i<8; i++) {
	mov	r2,#0x00
	mov	r3,#0x00
L032003?:
	clr	c
	mov	a,r2
	subb	a,#0x08
	mov	a,r3
	xrl	a,#0x80
	subb	a,#0x80
	jnc	L032006?
;	EFM8_JDY40_test.c:701: WriteData(customMouth[i]);
	mov	a,r2
	add	a,#_customMouth
	mov	dpl,a
	mov	a,r3
	addc	a,#(_customMouth >> 8)
	mov	dph,a
	clr	a
	movc	a,@a+dptr
	mov	dpl,a
	push	ar2
	push	ar3
	lcall	_WriteData
	pop	ar3
	pop	ar2
;	EFM8_JDY40_test.c:699: for(i=0; i<8; i++) {
	inc	r2
	cjne	r2,#0x00,L032003?
	inc	r3
	sjmp	L032003?
L032006?:
;	EFM8_JDY40_test.c:704: WriteCommand(0x48);
	mov	dpl,#0x48
	lcall	_WriteCommand
;	EFM8_JDY40_test.c:705: for(i=0; i<8; i++) {
	mov	r2,#0x00
	mov	r3,#0x00
L032007?:
	clr	c
	mov	a,r2
	subb	a,#0x08
	mov	a,r3
	xrl	a,#0x80
	subb	a,#0x80
	jnc	L032010?
;	EFM8_JDY40_test.c:707: WriteData(customEye[i]);
	mov	a,r2
	add	a,#_customEye
	mov	dpl,a
	mov	a,r3
	addc	a,#(_customEye >> 8)
	mov	dph,a
	clr	a
	movc	a,@a+dptr
	mov	dpl,a
	push	ar2
	push	ar3
	lcall	_WriteData
	pop	ar3
	pop	ar2
;	EFM8_JDY40_test.c:705: for(i=0; i<8; i++) {
	inc	r2
	cjne	r2,#0x00,L032007?
	inc	r3
	sjmp	L032007?
L032010?:
;	EFM8_JDY40_test.c:710: WriteCommand(0x50);  // Set CGRAM address
	mov	dpl,#0x50
	lcall	_WriteCommand
;	EFM8_JDY40_test.c:711: for(i=0; i<8; i++) {
	mov	r2,#0x00
	mov	r3,#0x00
L032011?:
	clr	c
	mov	a,r2
	subb	a,#0x08
	mov	a,r3
	xrl	a,#0x80
	subb	a,#0x80
	jnc	L032014?
;	EFM8_JDY40_test.c:713: WriteData(customOpenMouth[i]);
	mov	a,r2
	add	a,#_customOpenMouth
	mov	dpl,a
	mov	a,r3
	addc	a,#(_customOpenMouth >> 8)
	mov	dph,a
	clr	a
	movc	a,@a+dptr
	mov	dpl,a
	push	ar2
	push	ar3
	lcall	_WriteData
	pop	ar3
	pop	ar2
;	EFM8_JDY40_test.c:711: for(i=0; i<8; i++) {
	inc	r2
	cjne	r2,#0x00,L032011?
	inc	r3
	sjmp	L032011?
L032014?:
;	EFM8_JDY40_test.c:716: WriteCommand(0x58);  // Set CGRAM address
	mov	dpl,#0x58
	lcall	_WriteCommand
;	EFM8_JDY40_test.c:717: for(i=0; i<8; i++) {
	mov	r2,#0x00
	mov	r3,#0x00
L032015?:
	clr	c
	mov	a,r2
	subb	a,#0x08
	mov	a,r3
	xrl	a,#0x80
	subb	a,#0x80
	jnc	L032018?
;	EFM8_JDY40_test.c:719: WriteData(customSparkle[i]);
	mov	a,r2
	add	a,#_customSparkle
	mov	dpl,a
	mov	a,r3
	addc	a,#(_customSparkle >> 8)
	mov	dph,a
	clr	a
	movc	a,@a+dptr
	mov	dpl,a
	push	ar2
	push	ar3
	lcall	_WriteData
	pop	ar3
	pop	ar2
;	EFM8_JDY40_test.c:717: for(i=0; i<8; i++) {
	inc	r2
	cjne	r2,#0x00,L032015?
	inc	r3
	sjmp	L032015?
L032018?:
;	EFM8_JDY40_test.c:722: WriteCommand(0x60);  // Set CGRAM address
	mov	dpl,#0x60
	lcall	_WriteCommand
;	EFM8_JDY40_test.c:723: for(i=0; i<8; i++) {
	mov	r2,#0x00
	mov	r3,#0x00
L032019?:
	clr	c
	mov	a,r2
	subb	a,#0x08
	mov	a,r3
	xrl	a,#0x80
	subb	a,#0x80
	jnc	L032022?
;	EFM8_JDY40_test.c:725: WriteData(customMoney[i]);
	mov	a,r2
	add	a,#_customMoney
	mov	dpl,a
	mov	a,r3
	addc	a,#(_customMoney >> 8)
	mov	dph,a
	clr	a
	movc	a,@a+dptr
	mov	dpl,a
	push	ar2
	push	ar3
	lcall	_WriteData
	pop	ar3
	pop	ar2
;	EFM8_JDY40_test.c:723: for(i=0; i<8; i++) {
	inc	r2
	cjne	r2,#0x00,L032019?
	inc	r3
	sjmp	L032019?
L032022?:
;	EFM8_JDY40_test.c:728: WriteCommand(0x68);  // Set CGRAM address
	mov	dpl,#0x68
	lcall	_WriteCommand
;	EFM8_JDY40_test.c:729: for(i=0; i<8; i++) {
	mov	r2,#0x00
	mov	r3,#0x00
L032023?:
	clr	c
	mov	a,r2
	subb	a,#0x08
	mov	a,r3
	xrl	a,#0x80
	subb	a,#0x80
	jnc	L032026?
;	EFM8_JDY40_test.c:731: WriteData(customHappyMouth[i]);
	mov	a,r2
	add	a,#_customHappyMouth
	mov	dpl,a
	mov	a,r3
	addc	a,#(_customHappyMouth >> 8)
	mov	dph,a
	clr	a
	movc	a,@a+dptr
	mov	dpl,a
	push	ar2
	push	ar3
	lcall	_WriteData
	pop	ar3
	pop	ar2
;	EFM8_JDY40_test.c:729: for(i=0; i<8; i++) {
	inc	r2
	cjne	r2,#0x00,L032023?
	inc	r3
	sjmp	L032023?
L032026?:
;	EFM8_JDY40_test.c:734: WriteCommand(0x87);
	mov	dpl,#0x87
	lcall	_WriteCommand
;	EFM8_JDY40_test.c:735: WriteData(3);
	mov	dpl,#0x03
	lcall	_WriteData
;	EFM8_JDY40_test.c:737: WriteCommand(0x89);
	mov	dpl,#0x89
	lcall	_WriteCommand
;	EFM8_JDY40_test.c:738: WriteData(3);
	mov	dpl,#0x03
	lcall	_WriteData
;	EFM8_JDY40_test.c:740: WriteCommand(0xC8);
	mov	dpl,#0xC8
	lcall	_WriteCommand
;	EFM8_JDY40_test.c:741: WriteData(2);
	mov	dpl,#0x02
	lcall	_WriteData
;	EFM8_JDY40_test.c:743: waitms(500);
	mov	dptr,#0x01F4
	lcall	_waitms
;	EFM8_JDY40_test.c:744: servo1 = 150;
	mov	_servo1,#0x96
;	EFM8_JDY40_test.c:745: waitms(100);
	mov	dptr,#0x0064
	lcall	_waitms
;	EFM8_JDY40_test.c:747: P1_5 = 1;	
	setb	_P1_5
;	EFM8_JDY40_test.c:750: for(j=250; j>180; j-=5) 
	mov	r2,#0xFA
L032027?:
	mov	a,r2
	add	a,#0xff - 0xB4
	jnc	L032030?
;	EFM8_JDY40_test.c:752: servo2 = j;
	mov	_servo2,r2
;	EFM8_JDY40_test.c:753: waitms(20);
	mov	dptr,#0x0014
	push	ar2
	lcall	_waitms
	pop	ar2
;	EFM8_JDY40_test.c:750: for(j=250; j>180; j-=5) 
	mov	a,r2
	add	a,#0xfb
	mov	r2,a
	sjmp	L032027?
L032030?:
;	EFM8_JDY40_test.c:756: waitms(1000);
	mov	dptr,#0x03E8
	lcall	_waitms
;	EFM8_JDY40_test.c:759: for(j=150; j<250; j+=5) 
	mov	r2,#0x96
L032031?:
	cjne	r2,#0xFA,L032082?
L032082?:
	jnc	L032034?
;	EFM8_JDY40_test.c:761: servo1 = j;
	mov	_servo1,r2
;	EFM8_JDY40_test.c:762: waitms(20);
	mov	dptr,#0x0014
	push	ar2
	lcall	_waitms
	pop	ar2
;	EFM8_JDY40_test.c:759: for(j=150; j<250; j+=5) 
	mov	a,#0x05
	add	a,r2
	mov	r2,a
	sjmp	L032031?
L032034?:
;	EFM8_JDY40_test.c:765: waitms(1000);
	mov	dptr,#0x03E8
	lcall	_waitms
;	EFM8_JDY40_test.c:768: for(j=180; j > 90; j-=5){
	mov	r2,#0xB4
L032035?:
	mov	a,r2
	add	a,#0xff - 0x5A
	jnc	L032038?
;	EFM8_JDY40_test.c:769: servo2 = j;
	mov	_servo2,r2
;	EFM8_JDY40_test.c:770: waitms(20);
	mov	dptr,#0x0014
	push	ar2
	lcall	_waitms
	pop	ar2
;	EFM8_JDY40_test.c:768: for(j=180; j > 90; j-=5){
	mov	a,r2
	add	a,#0xfb
	mov	r2,a
	sjmp	L032035?
L032038?:
;	EFM8_JDY40_test.c:775: P1_5 = 0;
	clr	_P1_5
;	EFM8_JDY40_test.c:777: waitms(150);
	mov	dptr,#0x0096
	lcall	_waitms
;	EFM8_JDY40_test.c:778: servo1 = 250;
	mov	_servo1,#0xFA
;	EFM8_JDY40_test.c:779: servo2 = 250; 
	mov	_servo2,#0xFA
;	EFM8_JDY40_test.c:780: EMAGNET=0;
	clr	_P1_5
;	EFM8_JDY40_test.c:782: if ( cointcount >= 10 ) {
	clr	c
	mov	a,_cointcount
	subb	a,#0x0A
	mov	a,(_cointcount + 1)
	subb	a,#0x00
	jc	L032002?
;	EFM8_JDY40_test.c:783: direction = 1;
	mov	_direction,#0x01
	clr	a
	mov	(_direction + 1),a
;	EFM8_JDY40_test.c:784: cointcount=0;
	clr	a
	mov	_cointcount,a
	mov	(_cointcount + 1),a
L032002?:
;	EFM8_JDY40_test.c:789: WriteCommand(0x87);
	mov	dpl,#0x87
	lcall	_WriteCommand
;	EFM8_JDY40_test.c:790: WriteData(1);
	mov	dpl,#0x01
	lcall	_WriteData
;	EFM8_JDY40_test.c:792: WriteCommand(0x89);
	mov	dpl,#0x89
	lcall	_WriteCommand
;	EFM8_JDY40_test.c:793: WriteData(1);
	mov	dpl,#0x01
	lcall	_WriteData
;	EFM8_JDY40_test.c:795: WriteCommand(0xC8);
	mov	dpl,#0xC8
	lcall	_WriteCommand
;	EFM8_JDY40_test.c:796: WriteData(0);
	mov	dpl,#0x00
	ljmp	_WriteData
;------------------------------------------------------------
;Allocation info for local variables in function 'automaticmode'
;------------------------------------------------------------
;sideper                   Allocated with name '_automaticmode_PARM_2'
;freq                      Allocated with name '_automaticmode_PARM_3'
;fowardper                 Allocated to registers r2 r3 r4 r5 
;control                   Allocated to registers 
;------------------------------------------------------------
;	EFM8_JDY40_test.c:804: void automaticmode(float fowardper, float sideper, long int freq)
;	-----------------------------------------
;	 function automaticmode
;	-----------------------------------------
_automaticmode:
	mov	r2,dpl
	mov	r3,dph
	mov	r4,b
	mov	r5,a
;	EFM8_JDY40_test.c:807: direction = 3;
	mov	_direction,#0x03
	clr	a
	mov	(_direction + 1),a
;	EFM8_JDY40_test.c:808: sprintf(msg, "%05ld\n\r", freq); // subtracted so that it sends a smaller value
	push	ar2
	push	ar3
	push	ar4
	push	ar5
	push	_automaticmode_PARM_3
	push	(_automaticmode_PARM_3 + 1)
	push	(_automaticmode_PARM_3 + 2)
	push	(_automaticmode_PARM_3 + 3)
	mov	a,#__str_5
	push	acc
	mov	a,#(__str_5 >> 8)
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
;	EFM8_JDY40_test.c:809: sendstr1(msg);
	mov	dptr,#_msg
	mov	b,#0x40
	lcall	_sendstr1
;	EFM8_JDY40_test.c:810: waitms(50);
	mov	dptr,#0x0032
	lcall	_waitms
	pop	ar5
	pop	ar4
	pop	ar3
	pop	ar2
;	EFM8_JDY40_test.c:812: P3_7=1;  //wheel 1
	setb	_P3_7
;	EFM8_JDY40_test.c:813: P3_2=0;	// wheel 1 
	clr	_P3_2
;	EFM8_JDY40_test.c:814: P3_0=0; // wheel 2
	clr	_P3_0
;	EFM8_JDY40_test.c:815: P2_5=1; // wheel 2
	setb	_P2_5
;	EFM8_JDY40_test.c:816: if ( freq >= 63500)  //100000    63750   65000
	clr	c
	mov	a,_automaticmode_PARM_3
	subb	a,#0x0C
	mov	a,(_automaticmode_PARM_3 + 1)
	subb	a,#0xF8
	mov	a,(_automaticmode_PARM_3 + 2)
	subb	a,#0x00
	mov	a,(_automaticmode_PARM_3 + 3)
	xrl	a,#0x80
	subb	a,#0x80
	jc	L033002?
;	EFM8_JDY40_test.c:818: P3_7=0;  //wheel 1
	clr	_P3_7
;	EFM8_JDY40_test.c:819: P3_2=1;	// wheel 1 
	setb	_P3_2
;	EFM8_JDY40_test.c:820: P3_0=1; // wheel 2
	setb	_P3_0
;	EFM8_JDY40_test.c:821: P2_5=0; // wheel 2
	clr	_P2_5
;	EFM8_JDY40_test.c:822: waitms(250);
	mov	dptr,#0x00FA
	push	ar2
	push	ar3
	push	ar4
	push	ar5
	lcall	_waitms
;	EFM8_JDY40_test.c:823: P3_7=0;  //wheel 1
	clr	_P3_7
;	EFM8_JDY40_test.c:824: P3_2=0;	// wheel 1 
	clr	_P3_2
;	EFM8_JDY40_test.c:825: P3_0=0; // wheel 2
	clr	_P3_0
;	EFM8_JDY40_test.c:826: P2_5=0; // wheel 2
	clr	_P2_5
;	EFM8_JDY40_test.c:827: servomotion();
	lcall	_servomotion
	pop	ar5
	pop	ar4
	pop	ar3
	pop	ar2
;	EFM8_JDY40_test.c:828: cointcount++;
	mov	a,#0x01
	add	a,_cointcount
	mov	_cointcount,a
	clr	a
	addc	a,(_cointcount + 1)
	mov	(_cointcount + 1),a
L033002?:
;	EFM8_JDY40_test.c:832: if ( fowardper >= p_thresh)
	mov	a,#0xCD
	push	acc
	mov	a,#0xCC
	push	acc
	push	acc
	mov	a,#0x3D
	push	acc
	mov	dpl,r2
	mov	dph,r3
	mov	b,r4
	mov	a,r5
	lcall	___fslt
	mov	r2,dpl
	mov	a,sp
	add	a,#0xfc
	mov	sp,a
	mov	a,r2
	jnz	L033004?
;	EFM8_JDY40_test.c:834: P3_7=0;  //wheel 1
	clr	_P3_7
;	EFM8_JDY40_test.c:835: P3_2=1;	// wheel 1 
	setb	_P3_2
;	EFM8_JDY40_test.c:836: P3_0=1; // wheel 2
	setb	_P3_0
;	EFM8_JDY40_test.c:837: P2_5=0; // wheel 2
	clr	_P2_5
;	EFM8_JDY40_test.c:838: waitms(300);
	mov	dptr,#0x012C
	lcall	_waitms
;	EFM8_JDY40_test.c:839: P3_7=0;  //wheel 1
	clr	_P3_7
;	EFM8_JDY40_test.c:840: P3_2=1;	// wheel 1 
	setb	_P3_2
;	EFM8_JDY40_test.c:841: P3_0=0; // wheel 2
	clr	_P3_0
;	EFM8_JDY40_test.c:842: P2_5=0; // wheel 2
	clr	_P2_5
;	EFM8_JDY40_test.c:843: waitms(750);
	mov	dptr,#0x02EE
;	EFM8_JDY40_test.c:844: return;
	ljmp	_waitms
L033004?:
;	EFM8_JDY40_test.c:849: if ( sideper >= p_thresh)
	mov	a,#0xCD
	push	acc
	mov	a,#0xCC
	push	acc
	push	acc
	mov	a,#0x3D
	push	acc
	mov	dpl,_automaticmode_PARM_2
	mov	dph,(_automaticmode_PARM_2 + 1)
	mov	b,(_automaticmode_PARM_2 + 2)
	mov	a,(_automaticmode_PARM_2 + 3)
	lcall	___fslt
	mov	r2,dpl
	mov	a,sp
	add	a,#0xfc
	mov	sp,a
	mov	a,r2
	jnz	L033007?
;	EFM8_JDY40_test.c:851: P3_7=0;  //wheel 1
	clr	_P3_7
;	EFM8_JDY40_test.c:852: P3_2=1;	// wheel 1 
	setb	_P3_2
;	EFM8_JDY40_test.c:853: P3_0=0; // wheel 2
	clr	_P3_0
;	EFM8_JDY40_test.c:854: P2_5=0; // wheel 2
	clr	_P2_5
;	EFM8_JDY40_test.c:855: waitms(750);
	mov	dptr,#0x02EE
;	EFM8_JDY40_test.c:856: control = 1;
	ljmp	_waitms
L033007?:
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'main'
;------------------------------------------------------------
;timeout                   Allocated to registers 
;pulse_width               Allocated to registers 
;pulse_width1              Allocated to registers 
;adcwheel1                 Allocated with name '_main_adcwheel1_1_199'
;adcwheel2                 Allocated with name '_main_adcwheel2_1_199'
;which                     Allocated with name '_main_which_1_199'
;i                         Allocated to registers r2 r3 
;count                     Allocated to registers r2 r3 r4 r5 
;f                         Allocated with name '_main_f_1_199'
;v                         Allocated with name '_main_v_1_199'
;sloc0                     Allocated with name '_main_sloc0_1_0'
;------------------------------------------------------------
;	EFM8_JDY40_test.c:863: void main (void)
;	-----------------------------------------
;	 function main
;	-----------------------------------------
_main:
;	EFM8_JDY40_test.c:882: waitms(500);
	mov	dptr,#0x01F4
	lcall	_waitms
;	EFM8_JDY40_test.c:883: printf("\r\nEFM8LB12 JDY-40 Slave Test.\r\n");
	mov	a,#__str_6
	push	acc
	mov	a,#(__str_6 >> 8)
	push	acc
	mov	a,#0x80
	push	acc
	lcall	_printf
	dec	sp
	dec	sp
	dec	sp
;	EFM8_JDY40_test.c:884: UART1_Init(9600);
	mov	dptr,#0x2580
	clr	a
	mov	b,a
	lcall	_UART1_Init
;	EFM8_JDY40_test.c:886: ReceptionOff();
	lcall	_ReceptionOff
;	EFM8_JDY40_test.c:888: LCD_4BIT();
	lcall	_LCD_4BIT
;	EFM8_JDY40_test.c:890: TIMER0_Init(); 
	lcall	_TIMER0_Init
;	EFM8_JDY40_test.c:892: InitPinADC(2, 1); // Configure P2.1 as analog input
	mov	_InitPinADC_PARM_2,#0x01
	mov	dpl,#0x02
	lcall	_InitPinADC
;	EFM8_JDY40_test.c:893: InitPinADC(2, 3); // Configure P2.1 as analog input
	mov	_InitPinADC_PARM_2,#0x03
	mov	dpl,#0x02
	lcall	_InitPinADC
;	EFM8_JDY40_test.c:894: InitADC();
	lcall	_InitADC
;	EFM8_JDY40_test.c:898: SendATCommand("AT+VER\r\n");
	mov	dptr,#__str_7
	mov	b,#0x80
	lcall	_SendATCommand
;	EFM8_JDY40_test.c:899: SendATCommand("AT+BAUD\r\n");
	mov	dptr,#__str_8
	mov	b,#0x80
	lcall	_SendATCommand
;	EFM8_JDY40_test.c:900: SendATCommand("AT+RFID\r\n");
	mov	dptr,#__str_9
	mov	b,#0x80
	lcall	_SendATCommand
;	EFM8_JDY40_test.c:901: SendATCommand("AT+DVID\r\n");
	mov	dptr,#__str_10
	mov	b,#0x80
	lcall	_SendATCommand
;	EFM8_JDY40_test.c:902: SendATCommand("AT+RFC120\r\n");
	mov	dptr,#__str_11
	mov	b,#0x80
	lcall	_SendATCommand
;	EFM8_JDY40_test.c:903: SendATCommand("AT+POWE\r\n");
	mov	dptr,#__str_12
	mov	b,#0x80
	lcall	_SendATCommand
;	EFM8_JDY40_test.c:904: SendATCommand("AT+CLSS\r\n");
	mov	dptr,#__str_13
	mov	b,#0x80
	lcall	_SendATCommand
;	EFM8_JDY40_test.c:908: SendATCommand("AT+DVIDFFFF\r\n");  
	mov	dptr,#__str_14
	mov	b,#0x80
	lcall	_SendATCommand
;	EFM8_JDY40_test.c:910: WriteCommand(0x40);  // Set CGRAM address
	mov	dpl,#0x40
	lcall	_WriteCommand
;	EFM8_JDY40_test.c:911: for(i=0; i<8; i++) {
	mov	r2,#0x00
	mov	r3,#0x00
L034030?:
	clr	c
	mov	a,r2
	subb	a,#0x08
	mov	a,r3
	xrl	a,#0x80
	subb	a,#0x80
	jnc	L034033?
;	EFM8_JDY40_test.c:913: WriteData(customMouth[i]);
	mov	a,r2
	add	a,#_customMouth
	mov	dpl,a
	mov	a,r3
	addc	a,#(_customMouth >> 8)
	mov	dph,a
	clr	a
	movc	a,@a+dptr
	mov	dpl,a
	push	ar2
	push	ar3
	lcall	_WriteData
	pop	ar3
	pop	ar2
;	EFM8_JDY40_test.c:911: for(i=0; i<8; i++) {
	inc	r2
	cjne	r2,#0x00,L034030?
	inc	r3
	sjmp	L034030?
L034033?:
;	EFM8_JDY40_test.c:916: WriteCommand(0x48);
	mov	dpl,#0x48
	lcall	_WriteCommand
;	EFM8_JDY40_test.c:917: for(i=0; i<8; i++) {
	mov	r2,#0x00
	mov	r3,#0x00
L034034?:
	clr	c
	mov	a,r2
	subb	a,#0x08
	mov	a,r3
	xrl	a,#0x80
	subb	a,#0x80
	jnc	L034037?
;	EFM8_JDY40_test.c:919: WriteData(customEye[i]);
	mov	a,r2
	add	a,#_customEye
	mov	dpl,a
	mov	a,r3
	addc	a,#(_customEye >> 8)
	mov	dph,a
	clr	a
	movc	a,@a+dptr
	mov	dpl,a
	push	ar2
	push	ar3
	lcall	_WriteData
	pop	ar3
	pop	ar2
;	EFM8_JDY40_test.c:917: for(i=0; i<8; i++) {
	inc	r2
	cjne	r2,#0x00,L034034?
	inc	r3
	sjmp	L034034?
L034037?:
;	EFM8_JDY40_test.c:922: WriteCommand(0x50);  // Set CGRAM address
	mov	dpl,#0x50
	lcall	_WriteCommand
;	EFM8_JDY40_test.c:923: for(i=0; i<8; i++) {
	mov	r2,#0x00
	mov	r3,#0x00
L034038?:
	clr	c
	mov	a,r2
	subb	a,#0x08
	mov	a,r3
	xrl	a,#0x80
	subb	a,#0x80
	jnc	L034041?
;	EFM8_JDY40_test.c:925: WriteData(customOpenMouth[i]);
	mov	a,r2
	add	a,#_customOpenMouth
	mov	dpl,a
	mov	a,r3
	addc	a,#(_customOpenMouth >> 8)
	mov	dph,a
	clr	a
	movc	a,@a+dptr
	mov	dpl,a
	push	ar2
	push	ar3
	lcall	_WriteData
	pop	ar3
	pop	ar2
;	EFM8_JDY40_test.c:923: for(i=0; i<8; i++) {
	inc	r2
	cjne	r2,#0x00,L034038?
	inc	r3
	sjmp	L034038?
L034041?:
;	EFM8_JDY40_test.c:928: WriteCommand(0x58);  // Set CGRAM address
	mov	dpl,#0x58
	lcall	_WriteCommand
;	EFM8_JDY40_test.c:929: for(i=0; i<8; i++) {
	mov	r2,#0x00
	mov	r3,#0x00
L034042?:
	clr	c
	mov	a,r2
	subb	a,#0x08
	mov	a,r3
	xrl	a,#0x80
	subb	a,#0x80
	jnc	L034045?
;	EFM8_JDY40_test.c:931: WriteData(customSparkle[i]);
	mov	a,r2
	add	a,#_customSparkle
	mov	dpl,a
	mov	a,r3
	addc	a,#(_customSparkle >> 8)
	mov	dph,a
	clr	a
	movc	a,@a+dptr
	mov	dpl,a
	push	ar2
	push	ar3
	lcall	_WriteData
	pop	ar3
	pop	ar2
;	EFM8_JDY40_test.c:929: for(i=0; i<8; i++) {
	inc	r2
	cjne	r2,#0x00,L034042?
	inc	r3
	sjmp	L034042?
L034045?:
;	EFM8_JDY40_test.c:934: WriteCommand(0x60);  // Set CGRAM address
	mov	dpl,#0x60
	lcall	_WriteCommand
;	EFM8_JDY40_test.c:935: for(i=0; i<8; i++) {
	mov	r2,#0x00
	mov	r3,#0x00
L034046?:
	clr	c
	mov	a,r2
	subb	a,#0x08
	mov	a,r3
	xrl	a,#0x80
	subb	a,#0x80
	jnc	L034049?
;	EFM8_JDY40_test.c:937: WriteData(customMoney[i]);
	mov	a,r2
	add	a,#_customMoney
	mov	dpl,a
	mov	a,r3
	addc	a,#(_customMoney >> 8)
	mov	dph,a
	clr	a
	movc	a,@a+dptr
	mov	dpl,a
	push	ar2
	push	ar3
	lcall	_WriteData
	pop	ar3
	pop	ar2
;	EFM8_JDY40_test.c:935: for(i=0; i<8; i++) {
	inc	r2
	cjne	r2,#0x00,L034046?
	inc	r3
	sjmp	L034046?
L034049?:
;	EFM8_JDY40_test.c:940: WriteCommand(0x68);  // Set CGRAM address
	mov	dpl,#0x68
	lcall	_WriteCommand
;	EFM8_JDY40_test.c:941: for(i=0; i<8; i++) {
	mov	r2,#0x00
	mov	r3,#0x00
L034050?:
	clr	c
	mov	a,r2
	subb	a,#0x08
	mov	a,r3
	xrl	a,#0x80
	subb	a,#0x80
	jnc	L034053?
;	EFM8_JDY40_test.c:943: WriteData(customHappyMouth[i]);
	mov	a,r2
	add	a,#_customHappyMouth
	mov	dpl,a
	mov	a,r3
	addc	a,#(_customHappyMouth >> 8)
	mov	dph,a
	clr	a
	movc	a,@a+dptr
	mov	dpl,a
	push	ar2
	push	ar3
	lcall	_WriteData
	pop	ar3
	pop	ar2
;	EFM8_JDY40_test.c:941: for(i=0; i<8; i++) {
	inc	r2
	cjne	r2,#0x00,L034050?
	inc	r3
	sjmp	L034050?
L034053?:
;	EFM8_JDY40_test.c:946: WriteCommand(0x87);
	mov	dpl,#0x87
	lcall	_WriteCommand
;	EFM8_JDY40_test.c:947: WriteData(1);
	mov	dpl,#0x01
	lcall	_WriteData
;	EFM8_JDY40_test.c:949: WriteCommand(0x89);
	mov	dpl,#0x89
	lcall	_WriteCommand
;	EFM8_JDY40_test.c:950: WriteData(1);
	mov	dpl,#0x01
	lcall	_WriteData
;	EFM8_JDY40_test.c:952: WriteCommand(0xC8);
	mov	dpl,#0xC8
	lcall	_WriteCommand
;	EFM8_JDY40_test.c:953: WriteData(0);
	mov	dpl,#0x00
	lcall	_WriteData
;	EFM8_JDY40_test.c:955: P1_5 = 0;
	clr	_P1_5
;	EFM8_JDY40_test.c:956: while(1)
L034028?:
;	EFM8_JDY40_test.c:958: EMAGNET = 0;
	clr	_P1_5
;	EFM8_JDY40_test.c:960: count = GetPeriod(200);
	mov	dptr,#0x00C8
	lcall	_GetPeriod
;	EFM8_JDY40_test.c:961: f = GetFrequency(count);
	lcall	_GetFrequency
	mov	_main_f_1_199,dpl
	mov	(_main_f_1_199 + 1),dph
	mov	(_main_f_1_199 + 2),b
	mov	(_main_f_1_199 + 3),a
;	EFM8_JDY40_test.c:963: sprintf(msg, "%05ld\n\r", f); // subtracted so that it sends a smaller value
	push	_main_f_1_199
	push	(_main_f_1_199 + 1)
	push	(_main_f_1_199 + 2)
	push	(_main_f_1_199 + 3)
	mov	a,#__str_5
	push	acc
	mov	a,#(__str_5 >> 8)
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
;	EFM8_JDY40_test.c:964: sendstr1(msg);
	mov	dptr,#_msg
	mov	b,#0x40
	lcall	_sendstr1
;	EFM8_JDY40_test.c:965: waitms(50);
	mov	dptr,#0x0032
	lcall	_waitms
;	EFM8_JDY40_test.c:968: v[0] = Volts_at_Pin(QFP32_MUX_P2_1);
	mov	dpl,#0x0E
	lcall	_Volts_at_Pin
	mov	_main_sloc0_1_0,dpl
	mov	(_main_sloc0_1_0 + 1),dph
	mov	(_main_sloc0_1_0 + 2),b
	mov	(_main_sloc0_1_0 + 3),a
	mov	_main_v_1_199,_main_sloc0_1_0
	mov	(_main_v_1_199 + 1),(_main_sloc0_1_0 + 1)
	mov	(_main_v_1_199 + 2),(_main_sloc0_1_0 + 2)
	mov	(_main_v_1_199 + 3),(_main_sloc0_1_0 + 3)
;	EFM8_JDY40_test.c:969: v[1] = Volts_at_Pin(QFP32_MUX_P2_3);
	mov	dpl,#0x10
	lcall	_Volts_at_Pin
	mov	_main_sloc0_1_0,dpl
	mov	(_main_sloc0_1_0 + 1),dph
	mov	(_main_sloc0_1_0 + 2),b
	mov	(_main_sloc0_1_0 + 3),a
	mov	(_main_v_1_199 + 0x0004),_main_sloc0_1_0
	mov	((_main_v_1_199 + 0x0004) + 1),(_main_sloc0_1_0 + 1)
	mov	((_main_v_1_199 + 0x0004) + 2),(_main_sloc0_1_0 + 2)
	mov	((_main_v_1_199 + 0x0004) + 3),(_main_sloc0_1_0 + 3)
;	EFM8_JDY40_test.c:972: if(RXU1()) // Something has arrived
	lcall	_RXU1
	jc	L034098?
	ljmp	L034028?
L034098?:
;	EFM8_JDY40_test.c:975: getstr1(buff, sizeof(buff));
	mov	_getstr1_PARM_2,#0x14
	mov	dptr,#_buff
	mov	b,#0x40
	lcall	_getstr1
;	EFM8_JDY40_test.c:976: if ( strcmp(buff, "A") == 0 )
	mov	_strcmp_PARM_2,#__str_15
	mov	(_strcmp_PARM_2 + 1),#(__str_15 >> 8)
	mov	(_strcmp_PARM_2 + 2),#0x80
	mov	dptr,#_buff
	mov	b,#0x40
	lcall	_strcmp
	mov	a,dpl
	mov	b,dph
	orl	a,b
	jz	L034099?
	ljmp	L034011?
L034099?:
;	EFM8_JDY40_test.c:978: waitms(500);
	mov	dptr,#0x01F4
	lcall	_waitms
;	EFM8_JDY40_test.c:979: while(1)
L034008?:
;	EFM8_JDY40_test.c:981: waitms(5);
	mov	dptr,#0x0005
	lcall	_waitms
;	EFM8_JDY40_test.c:982: direction=3; 
	mov	_direction,#0x03
	clr	a
	mov	(_direction + 1),a
;	EFM8_JDY40_test.c:985: v[0] = Volts_at_Pin(QFP32_MUX_P2_1);
	mov	dpl,#0x0E
	lcall	_Volts_at_Pin
	mov	_main_sloc0_1_0,dpl
	mov	(_main_sloc0_1_0 + 1),dph
	mov	(_main_sloc0_1_0 + 2),b
	mov	(_main_sloc0_1_0 + 3),a
	mov	_main_v_1_199,_main_sloc0_1_0
	mov	(_main_v_1_199 + 1),(_main_sloc0_1_0 + 1)
	mov	(_main_v_1_199 + 2),(_main_sloc0_1_0 + 2)
	mov	(_main_v_1_199 + 3),(_main_sloc0_1_0 + 3)
;	EFM8_JDY40_test.c:986: v[1] = Volts_at_Pin(QFP32_MUX_P2_3);
	mov	dpl,#0x10
	lcall	_Volts_at_Pin
	mov	_main_sloc0_1_0,dpl
	mov	(_main_sloc0_1_0 + 1),dph
	mov	(_main_sloc0_1_0 + 2),b
	mov	(_main_sloc0_1_0 + 3),a
	mov	(_main_v_1_199 + 0x0004),_main_sloc0_1_0
	mov	((_main_v_1_199 + 0x0004) + 1),(_main_sloc0_1_0 + 1)
	mov	((_main_v_1_199 + 0x0004) + 2),(_main_sloc0_1_0 + 2)
	mov	((_main_v_1_199 + 0x0004) + 3),(_main_sloc0_1_0 + 3)
;	EFM8_JDY40_test.c:987: count = GetPeriod(200);
	mov	dptr,#0x00C8
	lcall	_GetPeriod
;	EFM8_JDY40_test.c:988: f = GetFrequency(count);
	lcall	_GetFrequency
	mov	_automaticmode_PARM_3,dpl
	mov	(_automaticmode_PARM_3 + 1),dph
	mov	(_automaticmode_PARM_3 + 2),b
	mov	(_automaticmode_PARM_3 + 3),a
;	EFM8_JDY40_test.c:989: automaticmode(v[0], v[1], f);
	mov	_automaticmode_PARM_2,(_main_v_1_199 + 0x0004)
	mov	(_automaticmode_PARM_2 + 1),((_main_v_1_199 + 0x0004) + 1)
	mov	(_automaticmode_PARM_2 + 2),((_main_v_1_199 + 0x0004) + 2)
	mov	(_automaticmode_PARM_2 + 3),((_main_v_1_199 + 0x0004) + 3)
	mov	dpl,_main_v_1_199
	mov	dph,(_main_v_1_199 + 1)
	mov	b,(_main_v_1_199 + 2)
	mov	a,(_main_v_1_199 + 3)
	lcall	_automaticmode
;	EFM8_JDY40_test.c:990: if ( direction == 1 ) break;
	mov	a,#0x01
	cjne	a,_direction,L034100?
	clr	a
	cjne	a,(_direction + 1),L034100?
	sjmp	L034011?
L034100?:
;	EFM8_JDY40_test.c:992: if(RXU1())
	lcall	_RXU1
	jc	L034101?
	ljmp	L034008?
L034101?:
;	EFM8_JDY40_test.c:994: printf("hello");
	mov	a,#__str_16
	push	acc
	mov	a,#(__str_16 >> 8)
	push	acc
	mov	a,#0x80
	push	acc
	lcall	_printf
	dec	sp
	dec	sp
	dec	sp
;	EFM8_JDY40_test.c:995: getstr1(buff, sizeof(buff));
	mov	_getstr1_PARM_2,#0x14
	mov	dptr,#_buff
	mov	b,#0x40
	lcall	_getstr1
;	EFM8_JDY40_test.c:996: if (strcmp(buff, "A") == 0 ) break;
	mov	_strcmp_PARM_2,#__str_15
	mov	(_strcmp_PARM_2 + 1),#(__str_15 >> 8)
	mov	(_strcmp_PARM_2 + 2),#0x80
	mov	dptr,#_buff
	mov	b,#0x40
	lcall	_strcmp
	mov	a,dpl
	mov	b,dph
	orl	a,b
	jz	L034102?
	ljmp	L034008?
L034102?:
L034011?:
;	EFM8_JDY40_test.c:1004: if ( strcmp(buff, "S") == 0 )	
	mov	_strcmp_PARM_2,#__str_17
	mov	(_strcmp_PARM_2 + 1),#(__str_17 >> 8)
	mov	(_strcmp_PARM_2 + 2),#0x80
	mov	dptr,#_buff
	mov	b,#0x40
	lcall	_strcmp
	mov	a,dpl
	mov	b,dph
	orl	a,b
	jnz	L034013?
;	EFM8_JDY40_test.c:1006: servomotion();
	lcall	_servomotion
;	EFM8_JDY40_test.c:1007: printf("this should be the motor function");
	mov	a,#__str_18
	push	acc
	mov	a,#(__str_18 >> 8)
	push	acc
	mov	a,#0x80
	push	acc
	lcall	_printf
	dec	sp
	dec	sp
	dec	sp
;	EFM8_JDY40_test.c:1008: waitms(500);
	mov	dptr,#0x01F4
	lcall	_waitms
L034013?:
;	EFM8_JDY40_test.c:1012: sscanf(buff, "K%uW%uG%d\n", &adcwheel1, &adcwheel2, &which);
	mov	a,#_main_which_1_199
	push	acc
	mov	a,#(_main_which_1_199 >> 8)
	push	acc
	mov	a,#0x40
	push	acc
	mov	a,#_main_adcwheel2_1_199
	push	acc
	mov	a,#(_main_adcwheel2_1_199 >> 8)
	push	acc
	mov	a,#0x40
	push	acc
	mov	a,#_main_adcwheel1_1_199
	push	acc
	mov	a,#(_main_adcwheel1_1_199 >> 8)
	push	acc
	mov	a,#0x40
	push	acc
	mov	a,#__str_19
	push	acc
	mov	a,#(__str_19 >> 8)
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
	add	a,#0xf1
	mov	sp,a
;	EFM8_JDY40_test.c:1013: if (which == 0 )
	mov	a,_main_which_1_199
	orl	a,(_main_which_1_199 + 1)
	jnz	L034015?
;	EFM8_JDY40_test.c:1015: P2_5 = 0;
	clr	_P2_5
;	EFM8_JDY40_test.c:1016: P3_7=0;
	clr	_P3_7
;	EFM8_JDY40_test.c:1017: direction = 1;
	mov	_direction,#0x01
	clr	a
	mov	(_direction + 1),a
	sjmp	L034016?
L034015?:
;	EFM8_JDY40_test.c:1021: P3_2=0;
	clr	_P3_2
;	EFM8_JDY40_test.c:1022: P3_0=0;
	clr	_P3_0
;	EFM8_JDY40_test.c:1023: direction = 0;
	clr	a
	mov	_direction,a
	mov	(_direction + 1),a
L034016?:
;	EFM8_JDY40_test.c:1027: pwm_duty4 = adcwheel2;
	mov	_pwm_duty4,_main_adcwheel2_1_199
	mov	(_pwm_duty4 + 1),(_main_adcwheel2_1_199 + 1)
;	EFM8_JDY40_test.c:1028: if ( adcwheel1 == 5535 ) adcwheel1 = 65535;
	mov	a,#0x9F
	cjne	a,_main_adcwheel1_1_199,L034018?
	mov	a,#0x15
	cjne	a,(_main_adcwheel1_1_199 + 1),L034018?
	mov	_main_adcwheel1_1_199,#0xFF
	mov	(_main_adcwheel1_1_199 + 1),#0xFF
L034018?:
;	EFM8_JDY40_test.c:1029: if ( adcwheel1 == 5086 ) adcwheel1 = 65535;
	mov	a,#0xDE
	cjne	a,_main_adcwheel1_1_199,L034020?
	mov	a,#0x13
	cjne	a,(_main_adcwheel1_1_199 + 1),L034020?
	mov	_main_adcwheel1_1_199,#0xFF
	mov	(_main_adcwheel1_1_199 + 1),#0xFF
L034020?:
;	EFM8_JDY40_test.c:1030: if ( adcwheel1 == 535 ) adcwheel1 = 65535;
	mov	a,#0x17
	cjne	a,_main_adcwheel1_1_199,L034022?
	mov	a,#0x02
	cjne	a,(_main_adcwheel1_1_199 + 1),L034022?
	mov	_main_adcwheel1_1_199,#0xFF
	mov	(_main_adcwheel1_1_199 + 1),#0xFF
L034022?:
;	EFM8_JDY40_test.c:1031: if ( adcwheel1 == 86 ) adcwheel1 = 65535;
	mov	a,#0x56
	cjne	a,_main_adcwheel1_1_199,L034111?
	clr	a
	cjne	a,(_main_adcwheel1_1_199 + 1),L034111?
	sjmp	L034112?
L034111?:
	sjmp	L034024?
L034112?:
	mov	_main_adcwheel1_1_199,#0xFF
	mov	(_main_adcwheel1_1_199 + 1),#0xFF
L034024?:
;	EFM8_JDY40_test.c:1032: pwm_duty2 = adcwheel1;
	mov	_pwm_duty2,_main_adcwheel1_1_199
	mov	(_pwm_duty2 + 1),(_main_adcwheel1_1_199 + 1)
;	EFM8_JDY40_test.c:1035: waitms(5); // The radio seems to need this delay...
	mov	dptr,#0x0005
	lcall	_waitms
	ljmp	L034028?
	rseg R_CSEG

	rseg R_XINIT

	rseg R_CONST
_customMouth:
	db 0x0e	; 14 
	db 0x04	; 4 
	db 0x00	; 0 
	db 0x00	; 0 
	db 0x11	; 17 
	db 0x15	; 21 
	db 0x0a	; 10 
	db 0x00	; 0 
_customEye:
	db 0x00	; 0 
	db 0x0e	; 14 
	db 0x19	; 25 
	db 0x19	; 25 
	db 0x1f	; 31 
	db 0x17	; 23 
	db 0x0e	; 14 
	db 0x00	; 0 
_customOpenMouth:
	db 0x0e	; 14 
	db 0x04	; 4 
	db 0x00	; 0 
	db 0x0e	; 14 
	db 0x11	; 17 
	db 0x11	; 17 
	db 0x0e	; 14 
	db 0x00	; 0 
_customSparkle:
	db 0x04	; 4 
	db 0x04	; 4 
	db 0x0a	; 10 
	db 0x11	; 17 
	db 0x0a	; 10 
	db 0x04	; 4 
	db 0x04	; 4 
	db 0x00	; 0 
_customMoney:
	db 0x04	; 4 
	db 0x0e	; 14 
	db 0x15	; 21 
	db 0x14	; 20 
	db 0x0e	; 14 
	db 0x05	; 5 
	db 0x15	; 21 
	db 0x0e	; 14 
_customHappyMouth:
	db 0x0e	; 14 
	db 0x04	; 4 
	db 0x00	; 0 
	db 0x15	; 21 
	db 0x0a	; 10 
	db 0x0a	; 10 
	db 0x0e	; 14 
	db 0x00	; 0 
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
_PrintNumber_HexDigit_1_164:
	db '0123456789ABCDEF'
	db 0x00
__str_4:
	db ' NO SIGNAL                     '
	db 0x0D
	db 0x00
__str_5:
	db '%05ld'
	db 0x0A
	db 0x0D
	db 0x00
__str_6:
	db 0x0D
	db 0x0A
	db 'EFM8LB12 JDY-40 Slave Test.'
	db 0x0D
	db 0x0A
	db 0x00
__str_7:
	db 'AT+VER'
	db 0x0D
	db 0x0A
	db 0x00
__str_8:
	db 'AT+BAUD'
	db 0x0D
	db 0x0A
	db 0x00
__str_9:
	db 'AT+RFID'
	db 0x0D
	db 0x0A
	db 0x00
__str_10:
	db 'AT+DVID'
	db 0x0D
	db 0x0A
	db 0x00
__str_11:
	db 'AT+RFC120'
	db 0x0D
	db 0x0A
	db 0x00
__str_12:
	db 'AT+POWE'
	db 0x0D
	db 0x0A
	db 0x00
__str_13:
	db 'AT+CLSS'
	db 0x0D
	db 0x0A
	db 0x00
__str_14:
	db 'AT+DVIDFFFF'
	db 0x0D
	db 0x0A
	db 0x00
__str_15:
	db 'A'
	db 0x00
__str_16:
	db 'hello'
	db 0x00
__str_17:
	db 'S'
	db 0x00
__str_18:
	db 'this should be the motor function'
	db 0x00
__str_19:
	db 'K%uW%uG%d'
	db 0x0A
	db 0x00

	CSEG

end
