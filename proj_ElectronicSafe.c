//                                                                                                         */
// proj_ElectronicSafe
//
// Blue   LED flash & Buzz once  when keycode input
// Green  LED flash & Buzz twice when keycode match passcode
// Red    LED flash & Buzz three when keycode mismatch passcode
//
// GPA12 : to control Blue LED
// GPA13 : to control Green LED
// GPA14 : to control Red LED
// GPA15 : to control external Buzzer
// GPB11 : to control on-board Buzzer
// GPB15 : on-board button input (Nu-LB-NUC140 SW INT) to change passcode

// HC05 Bluetooth module
// pin1 : KEY   N.C
// pin2 : VCC   to Vcc +5V
// pin3 : GND   to GND
// pin4 : TXD   to NUC140 UART0-RX (GPB0)
// pin5 : RXD   to NUC140 UART0-TX (GPB1)
// pin6 : STATE N.C.

// SG5010 DC servo
// pin1 : signal to PWM0/GPA12 (NUC140-pin65/NUC120-pin28)
// pin2 : Vcc
// pin3 : Gnd

// SR04 Ultrasound Sensor 
// pin1 Vcc : to Vcc
// pin2 Trig: to GPB4      (NUC140VE3xN pin19)
// pin3 ECHO: to GPB2/T2EX (NUC140VE3xN pin34)
// pin4 Gnd : to GND

// RC522 RFID Reader
// SDA (SS) : connected to GPC8 (SPI1_SS)
// SCK (SCK) : connected to GPC9 (SPI1_CLK)
// MOSI : connected to GPC11(SPI1_MOSI)
// MISO : connected to GPC10(SPI1_MISO)
// IRQ : no connection
// GND : connected to Gnd
// RST : connected to 3.3V / no connection
// VCC : connected to 3.3V

#include <stdio.h>																											 
#include <string.h>
#include <stdlib.h> 
#include "NUC1xx.h"
#include "DrvSYS.h"
#include "DrvGPIO.h"
#include "scankey.h"
#include "NUC1xx-LB_002\LCD_Driver.h"
#include "Driver\DrvUART.h"
#include "Driver_PWM_Servo.h"
#include "Driver\DrvSPI.h"
#include "SPI_RC522.h"


#define  BLUE  1
#define  GREEN 2
#define  RED   3



#define  InitLock  DrvGPIO_Open(E_GPA,11,E_IO_OUTPUT)
#define  OpenLock  DrvGPIO_SetBit(E_GPA,11)
#define  CloseLock DrvGPIO_ClrBit(E_GPA,11)

//Uart defines
#define DATASIZE 8
#define TDATASIZE 23

char TEXT0[16]="Electronic Safe";
char TEXT1[16]="KeyCode :";
char TEXT2[16]="PassCode:      ";
char TEXT3[16]="vrf code:      ";
char TEXT33[16]="vrf code:      ";
char TEXT7[16]="NewCode :";
char TEXT8[16]="CurCode :";

char passcode[5] = "1234";
char keycode[5]  = "0000";
char newkeycode[5]  = "0000";
char vrf[5]  = "0000";
char vrf1[5]  = "0000";
uint8_t Tvrf[5]=  "3667";
uint8_t Tvrf1[5]=  "5656";
uint8_t Tvrf2[5]=  "6565";
uint8_t Toneuse[5]=  "1313";

uint8_t Tcode[5]=  "5668";
uint8_t onetimeuse[25]=          "One Time Use Code:";
uint8_t newline[5]=  "\n";
uint8_t  Write_buf[TDATASIZE] =  "Verification Code: ";
uint8_t  Write_buf2[TDATASIZE] = "\n";
//------- timer and sensor
#define	_SR04A_ECHO		   (GPB_2)			//NUC140VE3xN, Pin19
#define	_SR04A_TRIG		   (GPB_4)			//NUC140VE3xN, Pin34
#define	_SR04A_TRIG_Low	 (GPB_4=0)
#define	_SR04A_TRIG_High (GPB_4=1)
#define  ONESHOT  0   // counting and interrupt when reach TCMPR number, then stop
#define  PERIODIC 1   // counting and interrupt when reach TCMPR number, then counting from 0 again
#define  TOGGLE   2   // keep counting and interrupt when reach TCMPR number, tout toggled (between 0 and 1)
#define  CONTINUOUS 3 // keep counting and interrupt when reach TCMPR number
//------------------

char TEXT4[16];
volatile uint8_t comRbuf[9];
volatile uint8_t comRbytes = 0;
volatile uint8_t intflag = 0;
//---timer and sensor:
volatile uint32_t SR04A_Echo_Width = 0;
volatile uint32_t SR04A_Echo_Flag  = FALSE;
static uint16_t Timer1Counter=0; // seconds
int forgotflag=0;
static uint16_t hours=0; 
uint32_t distance_mm;
static uint16_t minutes=0; 
//------
int number,vrfcode,hitime=0,hitime1=0, Tries=0,onetimeuseflag=0;

//------- RFID--------
unsigned char UID[4],Temp[4]                                       ;
unsigned char RF_Buffer[18]                                        ;
unsigned char Password_Buffer[6]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}   ; // Mifare One ????
int counter;
int out_flag, ok_flag, read_flag;
char Text[16];

char card[4] = {0xa7, 0xc3, 0x6e, 0x4d}; 

uint8_t  Timer_flag =0;
uint32_t Timer_count =0;


//--------------------
void Init_SPI()
{
	
	DrvSPI_Open(eDRVSPI_PORT1, eDRVSPI_MASTER, eDRVSPI_TYPE1, 8);
	DrvSPI_SetEndian(eDRVSPI_PORT1, eDRVSPI_MSB_FIRST);
	DrvSPI_DisableAutoSS(eDRVSPI_PORT1);
	DrvSPI_SetClockFreq(eDRVSPI_PORT1, 50000, 0); // set SPI clock = 50KHz
}
void Reader(void)
{
	if(PcdRequest(0x52,Temp)==MI_OK)
    {
      if(PcdAnticoll(UID)==MI_OK)
				read_flag =1;
    }  
}



//-------SAFE FORGGOTEN OPEN PROTECTION------
void InitTIMER1(void)
{
	/* Step 1. Enable and Select Timer clock source */          
	SYSCLK->CLKSEL1.TMR1_S = 0;	//Select 12Mhz for Timer1 clock source 
    SYSCLK->APBCLK.TMR1_EN =1;	//Enable Timer1 clock source

	/* Step 2. Select Operation mode */	
	TIMER1->TCSR.MODE=PERIODIC;		//Select periodic mode for operation mode

	/* Step 3. Select Time out period = (Period of timer clock input) * (8-bit Prescale + 1) * (24-bit TCMP)*/
	TIMER1->TCSR.PRESCALE=255;	// Set Prescale [0~255]
	TIMER1->TCMPR = 46875;		// Set TCMPR [0~16777215]								
								// (1/12000000)*(255+1)*46875 = 1 sec / 1 Hz
/*
	//----------------
	TIMER1->TCSR.PRESCALE=60;	// Set Prescale [0~255]
	TIMER1->TCMPR = 196721;		// Set TCMPR [0~16777215]								
								// (1/12000000)*(60+1)*196721 = 1 sec / 1 Hz
	*/
	
	/* Step 4. Enable interrupt */
	TIMER1->TCSR.IE = 1;
	TIMER1->TISR.TIF = 1;		//Write 1 to clear for safty		
	NVIC_EnableIRQ(TMR1_IRQn);	//Enable Timer1 Interrupt

	/* Step 5. Enable Timer module */
	TIMER1->TCSR.CRST = 1;		//Reset up counter
	TIMER1->TCSR.CEN = 1;		//Enable Timer1

//  	TIMER1->TCSR.TDR_EN=1;		// Enable TDR function
}
void Init_TMR2(void)
{	
	//Step 1: T2EX pin Enable (PB.2, Pin34)
	SYS->GPBMFP.UART0_nRTS_nWRL = 1;	
	SYS->ALTMFP.PB2_T2EX = 1;
	
  //Step 2: Timer Controller Reset and Setting Clock Source
	SYS->IPRSTC2.TMR2_RST = 1;          //Timer Controller: Reset
	SYS->IPRSTC2.TMR2_RST = 0;          //Timer Controller: Normal
	SYSCLK->CLKSEL1.TMR2_S = 0;	        //Timer Clock = 12 MHz
	SYSCLK->APBCLK.TMR2_EN = 1;         //Timer C lock Enable

	//Step 3: Timer Controller Setting
	//  TMR0_T = (12 MHz / (11+1) / 1000000)^-1 = 1.000 Second
	TIMER2->TCMPR = 0xffffff;           //Timer Compare Value:  [0~16777215]
	TIMER2->TCSR.PRESCALE = 11;         //Timer Prescale:       [0~255]
	TIMER2->TCSR.MODE = 0;              //Timer Operation Mode: One-Shot

	//Step 4: External Capture Mode Setting
	TIMER2->TEXCON.TEXEN = 1;	          //External Capture Function Enable
	TIMER2->TEXCON.RSTCAPSEL = 0;	      //Capture Mode Select: Capture Mode
	TIMER2->TEXCON.TEX_EDGE = 2;	      //Capture Edge: Rising & Falling

	//Step 5: Timer Interrupt Setting
//	TIMER2->TCSR.IE = 1;				      //Timeout Interrupt Enable
//	TIMER2->u32TISR |= 0x01;		      //Clear Timeout Flag (TIF)
	TIMER2->TEXCON.TEXIEN = 1;		      //Capture Interrupt Enable
	TIMER2->u32TEXISR |= 0x01;		      //Clear Capture Flag (TEXIF)
	NVIC_EnableIRQ(TMR2_IRQn);		      //Timer NVIC IRQ Enable

	//Step 6: Start Timer Capture (Set by Ultrasonic_Trigger() Function)
// 	TIMER2->TCSR.CRST = 1;			      //Timer Counter Reset
// 	TIMER2->TCSR.CEN = 1;				      //Timer Start
}



// TMR2 Interrupt Handler
void TMR1_IRQHandler(void) // Timer1 interrupt subroutine 
{

	Timer1Counter+=1;
	
	if(Timer1Counter==60)
	{
		minutes+=1;
		if(minutes==60)
		{
			minutes = 0;
			hours+=1;
		}
			TIMER1->TISR.TIF =1;
			Timer1Counter=0;
		
		if(hours==24)
			hours=0;
		
	}
 	TIMER1->TISR.TIF =1; 	   
}





void TMR2_IRQHandler(void)
{
	TIMER2->TEXCON.RSTCAPSEL = 0;       // set back for falling edge to capture
	TIMER2->TCSR.CEN = 1;					      //Timer Start

	if(TIMER2->TEXISR.TEXIF == 1)	      //Capture Flag (TEXIF)
	{
	 	TIMER2->u32TEXISR |= 0x01;				//Clear Capture Flag (TEXIF)
		SR04A_Echo_Width = TIMER2->TCAP;	//Load Capture Value (Unit: us)
		SR04A_Echo_Flag  = TRUE;
	}
}

// Ultrasonic Trigger
void SR04_Trigger(void)
{
	//Trigger of Ultrasonic Sensor
	_SR04A_TRIG_High;
	DrvSYS_Delay(10);							// 10us for TRIG width
	_SR04A_TRIG_Low;
	
  TIMER2->TEXCON.RSTCAPSEL = 1; // set for rising edge trigger to reset counter
}

void Init_GPIO_SR04(void)
{
	//Ultrasonic I/O Pins Initial
	GPIOB->PMD.PMD2 = 0;							//_SR04_ECHO pin, Input						
	GPIOB->PMD.PMD4 = 1;              //_SR04_TRIG pin, Output
  _SR04A_TRIG_Low;                  // set Trig output to Low
}
//-------------------------------------------
void Init_RGBLED(void)
{
	// initialize GPIO pins
	DrvGPIO_Open(E_GPA, 12, E_IO_OUTPUT); // GPA12 pin set to output mode
	DrvGPIO_Open(E_GPA, 13, E_IO_OUTPUT); // GPA13 pin set to output mode
	DrvGPIO_Open(E_GPA, 14, E_IO_OUTPUT); // GPA14 pin set to output mode
	// set GPIO pins output Hi to disable LEDs
	DrvGPIO_SetBit(E_GPA, 12); // GPA12 pin output Hi to turn off Blue  LED
	DrvGPIO_SetBit(E_GPA, 13); // GPA13 pin output Hi to turn off Green LED
	DrvGPIO_SetBit(E_GPA, 14); // GPA14 pin output Hi to turn off Red   LED
}

void RGBLED(uint8_t LEDcolor)
{
	switch(LEDcolor) {
		case BLUE:  DrvGPIO_ClrBit(E_GPA,12); // Blue LED on
		            DrvGPIO_SetBit(E_GPA,13);
		            DrvGPIO_SetBit(E_GPA,14);
		            break;
		case GREEN: DrvGPIO_SetBit(E_GPA,12);
		            DrvGPIO_ClrBit(E_GPA,13); // Green LED on
		            DrvGPIO_SetBit(E_GPA,14);
		            break;		
		case RED:   DrvGPIO_SetBit(E_GPA,12);
		            DrvGPIO_SetBit(E_GPA,13);
		            DrvGPIO_ClrBit(E_GPA,14); // Red   LED on
		            break;		
		default :    DrvGPIO_SetBit(E_GPA,12);
		            DrvGPIO_SetBit(E_GPA,13);
		            DrvGPIO_SetBit(E_GPA,14);
                break;  
  }		
	DrvSYS_Delay(100000 * LEDcolor);
  DrvGPIO_SetBit(E_GPA,12);
	DrvGPIO_SetBit(E_GPA,13);
	DrvGPIO_SetBit(E_GPA,14);
}
void ChangeCode(void)
{
	int number1=0,j=0;
	
	//last code----------------
			print_lcd(0,"Enter cur. code                 ");
			print_lcd(1,"                  ");
			print_lcd(2,"                  ");
			print_lcd(3,"                  ");	
			DrvSYS_Delay(3000000);
			
			while(1){
				
			number1 = Scankey();		
		if (number1!=0) {
			newkeycode[j] = 0x30+number1;
			sprintf(TEXT8+9+j,"%d",number1);
			print_lcd(1,TEXT8);
			DrvSYS_Delay(3000000);
			j++;

			
			if (j==4) {
				j=0;
			if (!strcmp(passcode,newkeycode)==0)
				{
					RGBLED(RED);
					print_lcd(3,"Worng Code!           ");	
					DrvSYS_Delay(3000000);
					DrvSYS_Delay(3000000);
					

				}
				else{
					RGBLED(GREEN);
					break;}
			print_lcd(1,"                  ");
			

			}
		
			print_lcd(3,"                  ");	
		}
	}
			//-----------------------------------------------
			print_lcd(0,"Enter new code.                 ");
			print_lcd(1,"                  ");
			print_lcd(2,"                  ");
			print_lcd(3,"                  ");	
			DrvSYS_Delay(3000000);

			while(1)
	{
		

		number1 = Scankey();		
		if (number1!=0) {
			newkeycode[j] = 0x30+number1;
			//sprintf(TEXT1,"NewCode :");
			sprintf(TEXT7+9+j,"%d",number1);
			print_lcd(1,TEXT7);
			j++;
			DrvSYS_Delay(3000000);
			
			if (j==4) {
				
				//RGBLED(GREEN);					
				strcpy(passcode,newkeycode);
				sprintf(TEXT2,"PassCode: %s", passcode);
				print_lcd(2,TEXT2);
				//CloseLock;
				DrvSYS_Delay(300000);
				sprintf(TEXT1,"KeyCode :     ");
				print_lcd(1,TEXT1);
				print_lcd(2,"                ");					
				j=0;
				break;
			}
		} 
	}
		
			  
}
void ResetCode(void)
{
	int k=0,j=0,number3=0;
	
	DrvUART_Write(UART_PORT0,Write_buf,TDATASIZE);		// send verification code to terminal	
	DrvUART_Write(UART_PORT0,Tvrf1,5);		// send verification code to terminal	
	DrvUART_Write(UART_PORT0,Write_buf2,TDATASIZE);		// send verification code to terminal	
			 
	print_lcd(0,"Authentication   ");
	print_lcd(1,"needed, plese       ");
	print_lcd(2,"enter verf code:  ");
	print_lcd(3,"vrf code:                ");
	
	while(1)
		{//for entering verfication code
			vrfcode = Scankey();	//get vrf code from user
			if (vrfcode!=0)
				{
					vrf1[k] = 0x30+vrfcode;
					sprintf(TEXT33+9+k,"%d",vrfcode);
					print_lcd(3,TEXT33);
					k++;
					DrvSYS_Delay(3000000);
				}
				
			if (k==4) 
				{
					k=0;
					if(!( vrf1[0]==Tvrf1[0] && vrf1[1]==Tvrf1[1] && vrf1[2]==Tvrf1[2] && vrf1[3]==Tvrf1[3] ))
						{ 
							RGBLED(RED);
							print_lcd(3,"Incorrect!                 ");
							DrvSYS_Delay(300000000);
							print_lcd(3,"vrf code:                ");
						}else
						{
							RGBLED(GREEN);
							print_lcd(3,"Correct!                 ");
							DrvSYS_Delay(300000000);
							break;
						}
				}
		} //end of while for vrf code
		
		print_lcd(0,"Enter new code.                 ");
		print_lcd(1,"                  ");
		print_lcd(2,"                  ");
		print_lcd(3,"                  ");	
		DrvSYS_Delay(3000000);
		
		while(1)
			{
				number3 = Scankey();		
				if (number3!=0)
					{
						newkeycode[j] = 0x30+number3;
						sprintf(TEXT7+9+j,"%d",number3);
						print_lcd(1,TEXT7);
						j++;
						DrvSYS_Delay(3000000);
						
						if (j==4)
							{
								RGBLED(GREEN);					
								strcpy(passcode,newkeycode);
								sprintf(TEXT2,"PassCode: %s", passcode);
								print_lcd(2,TEXT2);
								DrvSYS_Delay(300000);
								sprintf(TEXT1,"KeyCode :     ");
								print_lcd(1,TEXT1);
								print_lcd(2,"                ");					
								j=0;
								break;
							}
					} 
			}

}
void onetimecode(void)
{

			DrvUART_Write(UART_PORT0,onetimeuse,TDATASIZE);		// send onetime use code to terminal	
			DrvUART_Write(UART_PORT0,Toneuse,5);		
			DrvUART_Write(UART_PORT0,Write_buf2,TDATASIZE);		
	
			onetimeuseflag=1;
}
void pickhelp(void)
{
	int pick=0;
	while(1)
		{
			pick = Scankey();	//returns 0 while not clicked
			if(pick!=0)
				{
			if (pick==1)
				{
					while(Scankey()!=0);
					ChangeCode();
					break;
				}
			if (pick==2)
				{
					while(Scankey()!=0);
					ResetCode();
					break;
				}
			if (pick==3)
			{
				while(Scankey()!=0);
				onetimecode();
				
				print_lcd(0,"Sending Onetime        ");
				print_lcd(1,"use code to BT        ");
				print_lcd(2,"terminal...          ");
				print_lcd(3,"                   ");
				DrvSYS_Delay(300000000);
				
				break;
			}
		
				}
		}
}

void OpenSafe(void)
{
	for (hitime=50; hitime<=58; hitime++) {//open
		
						
						PWM_Servo(0, hitime);
						print_lcd(2,"Access Granted!              ");
						DrvSYS_Delay(100000);
					}					
					print_lcd(2,"                ");	



}
void CloseSafe(void)
{
	
	for (hitime1=50; hitime1<=130; hitime1+=4) {//close
		
						PWM_Servo(0, hitime1);
						print_lcd(2,"Locking...              ");
						DrvSYS_Delay(100000);
					}
					PWM_Servo(0, 0);	
					print_lcd(2,"                ");	
					
}


void Init_Buzzer(void)
{
	DrvGPIO_Open(E_GPB, 11, E_IO_OUTPUT); // on-board Buzzer control : low-active
	DrvGPIO_Open(E_GPA, 15, E_IO_OUTPUT);	// external Buzzer control : high-active
	DrvGPIO_SetBit(E_GPB, 11);
	DrvGPIO_ClrBit(E_GPA, 15);	
}
void Buzz(uint8_t beep_no) 
{
	uint8_t i=0;
	while(i<beep_no) {
		DrvGPIO_ClrBit(E_GPB, 11);
		DrvGPIO_SetBit(E_GPA, 15);
		DrvSYS_Delay(100000);
		DrvGPIO_SetBit(E_GPB, 11);
		DrvGPIO_ClrBit(E_GPA, 15);
		DrvSYS_Delay(100000);		
		i++;
	}
}


void EINT1Callback(void)
{
	while(intflag!=1){
		print_lcd(0,"Code Settings:     ");
		print_lcd(1,"1.Change code      ");
		print_lcd(2,"2.Reset code        ");
		print_lcd(3,"3.Single use code   ");
		DrvSYS_Delay(3000000);
		pickhelp();
	
		print_lcd(1,"                  ");
		print_lcd(2,"                    ");
		intflag=1;
	}
	
		return ;		
}
	

int32_t main (void)
{
	
	
	
	uint8_t  i, safedoor=0,k=0;
  STR_UART_T sParam;
	
	UNLOCKREG();                                 // Unlock the protected registers
	DrvSYS_SetOscCtrl(E_SYS_XTL12M, 1);          // Enable 12MHz oscillator
	SYSCLK->PWRCON.XTL12M_EN=1;//servo & sensor
	DrvSYS_Delay(5000);                          // delay for stable clock
	SYSCLK->CLKSEL0.HCLK_S=0;//servo
	DrvSYS_SelectHCLKSource(0);	                 // HCLK clock source. 0: external 12MHz;
	while(DrvSYS_GetChipClockSourceStatus(E_SYS_XTL12M) == 0);
	DrvSYS_Open(50000000);
	LOCKREG();                                   // Lock the protected registers

	DrvSYS_SetClockDivider(E_SYS_HCLK_DIV, 0); /* HCLK clock frequency = HCLK clock source / (HCLK_N + 1) */
	DrvGPIO_InitFunction(E_FUNC_SPI1);
	
	Init_RGBLED();                               // initial RGB LEDs
	Init_Buzzer();		                           // initial Buzzer	
	InitTIMER1();                             // timer for clock
	Init_TMR2();                      // initialize Timer2 Capture for sensor
	Init_SPI();
	
	DrvGPIO_Open(E_GPB, 15, E_IO_INPUT);         // initial SWINT button for changing passcode
	DrvGPIO_EnableEINT1(E_IO_RISING, E_MODE_EDGE, EINT1Callback); // set up Interrupt callback	
	
	PcdReset();
	PcdAntennaOn();
	
	//UART:-----------------------------------------------		
	DrvGPIO_InitFunction(E_FUNC_UART0); // Set UART pins
	/* UART Setting */
   sParam.u32BaudRate 	= 9600;
   sParam.u8cDataBits 	= DRVUART_DATABITS_8;
   sParam.u8cStopBits 	= DRVUART_STOPBITS_1;
	 sParam.u8cParity 	= DRVUART_PARITY_NONE;
   sParam.u8cRxTriggerLevel= DRVUART_FIFO_1BYTES;
		 
	/* Set UART Configuration */
	 if(DrvUART_Open(UART_PORT0,&sParam) != E_SUCCESS);
 //------------------------END------------------------
	
	 Initial_panel();                             // initial LCD panel
	 clr_all_panel();

	 OpenKeyPad();	                               // initialize 3x3 keypad
	 InitPWM(0);            // initialize PWM0 SERVO
	 i=0;	
	
			

	 while(1)
		{
			intflag=0;
			
			Reader();  // call function for reading rfid tag
			
			SR04_Trigger();                 // Trigger Ultrasound Sensor for 10us   		
			DrvSYS_Delay(40000);            // Wait 40ms for Echo to trigger interrupt
			
			distance_mm = SR04A_Echo_Width * (340/2) / 1000;
			DrvSYS_Delay(10000);           // 10ms from Echo to next Trigger

			if(safedoor==1) // left forgotten protection
				{
					if(SR04A_Echo_Flag==TRUE)
						{
							SR04A_Echo_Flag = FALSE;
							if(distance_mm<500) 
								{
									print_lcd(2, "                   ");
									Timer1Counter=0;
									forgotflag=0;
								}
							if(distance_mm>500 )//if user just left; start timer 
								{
									
									if(forgotflag==0){
										Timer1Counter=0;
										forgotflag=1; //means user is away
										}
								}
						 if (distance_mm>500 && Timer1Counter>=15 )//user forgot safe open; close lock/door
							 {
								 Buzz(3);	
							 	 print_lcd(2, "User is away!       ");//have to close lock
								 DrvSYS_Delay(100000000);	    // Delay 		
								 print_lcd(2, "              ");
								 CloseSafe();
								 safedoor=0;
								 forgotflag=0;
							 }
						}
				}
				print_lcd(0,TEXT0);                          // print title
				print_lcd(1,TEXT1);
				print_lcd(3,"press SW for hlp!              ");	
				number = Scankey();	
				if (number!=0)
					{
						keycode[i] = 0x30+number;
						sprintf(TEXT1+9+i,"%d",number);
						print_lcd(1,TEXT1);
						i++;
						DrvSYS_Delay(3000000);
						if (i==4) 
							{
								if ((strcmp(passcode,keycode)==0 ||  (onetimeuseflag==1 && keycode[0]==Toneuse[0] && keycode[1]==Toneuse[1] && keycode[2]==Toneuse[2] && keycode[3]==Toneuse[3]  ) || ((UID[0]==card[0]) && (UID[1]==card[1]) && (UID[2]==card[2]) && (UID[3]==card[3]))) )
									{
										if(onetimeuseflag==1)
											onetimeuseflag=0;
										Tries=0;
										if(safedoor==0)
											{
												Buzz(2);
												OpenSafe();
												print_lcd(2,"Unlocking...              ");
												DrvSYS_Delay(3000000);
												safedoor=1;//safe door is open
											}else{// if safe door is open; user entered correct code; closelock
												Buzz(2);
												CloseSafe();
												safedoor=0;
											}
											print_lcd(2,"                ");
									}else{//safe door closed; wrong code entered
										RGBLED(RED);	
										print_lcd(2,"Access Denied !   ");				
										DrvSYS_Delay(300000000);
										print_lcd(2,"                ");	
										Tries++;
										if (Tries==3)
											{//-------------In case of 3 failed attemets:----------------------
												Buzz(5);
												RGBLED(RED);
												
												DrvUART_Write(UART_PORT0,Write_buf,TDATASIZE);		// send verification code to terminal	
												DrvUART_Write(UART_PORT0,Tvrf,5);		// send verification code to terminal	
												DrvUART_Write(UART_PORT0,Write_buf2,TDATASIZE);		// send verification code to terminal	

												print_lcd(0,"Authentication   ");
												print_lcd(1,"needed, plese       ");
												print_lcd(2,"enter verf code:  ");
												print_lcd(3,"vrf code:             ");
												DrvSYS_Delay(3000000);

												Tries=0;
												while(1)
													{//for entering verfication code
														vrfcode = Scankey();	//get vrf code from user
														if (vrfcode!=0)
															{
																vrf[k] = 0x30+vrfcode;
																sprintf(TEXT3+9+k,"%d",vrfcode);
																print_lcd(3,TEXT3);
																k++;
																DrvSYS_Delay(3000000);
															}
														if (k==4)
															{
																k=0;
																if(!( vrf[0]==Tvrf[0] && vrf[1]==Tvrf[1] && vrf[2]==Tvrf[2] && vrf[3]==Tvrf[3] ))
																	{ 
																		RGBLED(RED);
																		print_lcd(3,"Incorrect!                 ");
																		DrvSYS_Delay(300000000);
																		print_lcd(3,"vrf code:             ");
																	}else{
																		RGBLED(GREEN);
																		print_lcd(3,"Correct!                 ");
																		DrvSYS_Delay(300000000);

																		print_lcd(1,"                     ");
																		print_lcd(2,"                      ");
																		print_lcd(3,"                    ");	
																		print_lcd(0,"Electronic Safe      ");
																		print_lcd(3,"press SW for hlp!              ");
																		break;
																	}
															} //end of while for vrf code
															
													}
											}//----------- 3 failed attempts --- END
											
									}
									sprintf(TEXT1,"KeyCode :       ");
									print_lcd(1,TEXT1);				
									i = 0;
							}
					} 
		}
}

