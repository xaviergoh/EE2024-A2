//Goh Jun Yi Xavier A155434H
//Shreya Bhatia A0162818Y

//------------------------------------------------------------------->//
//                          LIBRARIES
//------------------------------------------------------------------->//

//C Standard
#include "stdio.h"
#include "stdlib.h"
#include "inttypes.h"

//Board Related
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_timer.h"
#include "LPC17XX.h" //HAL Library - Refer for System Interrupts/Handlers
//#include "lpc17xx_rit.h" - not used yet
#include "lpc17xx_uart.h"

//Peripherals
#include "joystick.h"
#include "pca9532.h"
#include "acc.h"
#include "oled.h"
#include "rgb.h"
#include "led7seg.h"
#include "temp.h"
#include "light.h"
#include "rotary.h"


//<------------------END OF LIBRARY----------------------------//

//------------------------------>//
//		DEFINITIONS
//------------------------------>//

//EINT0/1 Definitions
#define PINSEL_EINT0			20
#define PINSEL_EINT1			22

#define SBIT_EINT0				0
#define SBIT_EINT1				1

#define SBIT_EXTMODE0		0
#define SBIT_EXTMODE1		1

#define SBIT_EXTPOLAR0	0
#define SBIT_EXTPOLAR1	1

//TIMER0/1 Definitions
#define SBIT_TIMER0		1
#define SBIT_TIMER1		2

#define SBIT_MR0I 			0
#define SBIT_MR0R 		1

#define SBIT_CNTEN		0

#define PCLK_TIMER0	2
#define PCLK_TIMER1	4

#define MilliToMicroSec(x) (x*1000)	//ms is multiplied by 1000 to get us

unsigned int getPrescalarForUs(uint8_t timerPclkBit);

//THRESHOLD Definitions
#define TEMP_THRESHOLD			25
#define TEMP_LoTHRESHOLD		0
#define LIGHT_THRESHOLD			3000
#define LIGHT_LoTHRESHOLD	0
#define ACC_THRESHOLD			25.6

//------END OF DEFINITIONS------>//

//-------------------------------------------------------------------------->//
//                          DECLARATIONS
//-------------------------------------------------------------------------->//

//ENUMERATIONS
//An enumeration is a data type with a set of named values called elements

typedef enum {
	STATIONARY, FORWARD, REVERSE
} system_mode_t;

typedef enum{
	BLINK_NONE, BLINK_RED, BLINK_GREEN, BLINK_BLUE, BLINK_BOTH  //Both (Red & Blue)
} blink_mode_t;

volatile blink_mode_t BLINK_MODE;

//ACCELEROMETER & JOYSTICK
int8_t xyz[3] = {0};
int32_t xoff = 0;
int32_t yoff = 0;
int32_t zoff = 0;

int8_t x = 0;
int8_t y = 0;
int8_t z = 0;
uint8_t dir = 1;
uint8_t wait = 0;
uint8_t state = 0;


//TIME
volatile uint32_t msTicks = 0;
static const int TIcksInOneSecond = 1000;
int TickInterval = 333; //Set time interval in ms

//get current tick (time)
uint32_t getTicks(void)
{
    return msTicks;
}
volatile uint32_t SSEG_Timer = 0;


//ETC
const uint8_t ASCII_list[] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
const uint8_t inv_ASCII[] = {																			//Inverted SSEG display
		// 0 - 9
		0x24, 0x7D, 0xE0, 0x70, 0x39, 0x32, 0x23, 0x7C, 0x20, 0x38,
		//A - F
		0x28, 0x20, 0xA6, 0x24, 0xA2, 0xAA,
		//Other characters
		0xFB, 0xDF, 0xFF};

const uint8_t RGB_list[] = { RGB_RED, RGB_GREEN, RGB_BLUE }; 		//RGB_COLOR defined in rgb.c
uint8_t display_acc[40] = {}; 																				//Each element in display_acc is 8 bits
uint8_t display_mode[40] = {};
uint8_t display_temp[40] = {};
uint8_t display_lux[40] = {};
uint8_t display_xyz[40] = {};

//Flags
int TEMP_FLAG = 0;
int LIGHT_FLAG = 0;
int ACC_FLAG = 0;


//TEMPERATURE
int32_t temp_value = 0;
int periodCounter = 0;
int tempPeriods = 170;
int t1 = 0;
int t2 = 0;
int TEMP_DIVISOR = 10;

//LIGHT
uint32_t light_value = 0;

// Interrupt Variables
uint8_t MODE_TOGGLE = 0; 				//Modes of Operation — Stationary (0) , Forward (1) , Reverse (2)
uint8_t EINT0_RESET=0; 					//start timer
uint8_t EINT3_RESET=0;
uint32_t EINT0_Timer = 0;
uint32_t EINT3_Timer = 0;
int EINT3_Flag = 0;
int EINT0_FLAG = 0;


//UART (RAT Telemetry)
char UART_msg[100] = {};
char* WARNING = NULL;
char* light_warning = NULL;
char* temp_warning = NULL;
int RAT_COUNTER = 0;						//Counter for TX to RAT from CATS
uint8_t rev_buf[255];								//Reception Buffer
uint32_t rev_cnt = 0;								//Reception Counter
uint32_t isReceived = 0;						//Received Flag

//----
int red = 0;
int blue = 0;
int n = 0;

//WARNINGS
void set_warnings()
{
	light_warning = "Obstacle Near. \r\n";
	temp_warning = "Battery Hot \r\n";
}

//-----------------------END OF DECLARATIONS--------------------------->//




//----------------------------------------------------------------->//
//                              16LED
//----------------------------------------------------------------->//
//LED Bar position

static uint8_t barPos = 2;
static void moveBar(uint8_t steps, uint8_t dir)
{
    uint16_t ledOn = 0;
    if (barPos == 0)
        ledOn = (1 << 0) | (3 << 14);
    else if (barPos == 1)
        ledOn = (3 << 0) | (1 << 15);
    else
        ledOn = 0x07 << (barPos-2);
    barPos += (dir*steps);
    barPos = (barPos % 16);
    pca9532_setLeds(ledOn, 0xffff);
}

void ledlight () {
	if (light_value <=140){

		   pca9532_setLeds(0x0000, 0xffff);
	}
	else if (light_value > 240 && light_value <= 270) {

		pca9532_setLeds(0x1ff8, 0xe007);
	}
	else if (light_value > 270 && light_value <= 310) {
		pca9532_setLeds(0x3ffC, 0xC003);
	}

	else if (light_value > 310 && light_value <= 400) {
		pca9532_setLeds(0x7ffe, 0x8001);
	}
	else  {
		pca9532_setLeds(0xffff, 0x0000);
	}
}


//<----------------------END OF LED----------------------------//

//-------------------------------------------------------------------->//
//                              JOYSTICK
//-------------------------------------------------------------------->//
//Draw on OLED using JoyStick
static void drawOled(uint8_t joyState)
{
    static int wait = 0;
    static uint8_t currX = 48;
    static uint8_t currY = 32;
    static uint8_t lastX = 0;
    static uint8_t lastY = 0;

    if ((joyState & JOYSTICK_CENTER) != 0) {
        oled_clearScreen(OLED_COLOR_BLACK);
        return;
    }

    if (wait++ < 3)
        return;
    wait = 0;
    if ((joyState & JOYSTICK_UP) != 0 && currY > 0) {
        currY--;
    }
    if ((joyState & JOYSTICK_DOWN) != 0 && currY < OLED_DISPLAY_HEIGHT-1) {
        currY++;
    }
    if ((joyState & JOYSTICK_RIGHT) != 0 && currX < OLED_DISPLAY_WIDTH-1) {
        currX++;
    }
    if ((joyState & JOYSTICK_LEFT) != 0 && currX > 0) {
        currX--;
    }
    if (lastX != currX || lastY != currY) {
        oled_putPixel(currX, currY, OLED_COLOR_WHITE);
        lastX = currX;
        lastY = currY;
    }
}
//<--------------------------END OF JOYSTICK---------------------------------//





//----------------------------------------------------------------------------------------------------------------->//
//                                                          SSP
//------------------------------------------------------------------------------------------------------------------>//
// Seven Segment PinCfg
static void init_ssp(void)
{
    SSP_CFG_Type SSP_ConfigStruct;
    PINSEL_CFG_Type PinCfg;

    /*
     * Initialize SPI pin connect
     * P0.7 - SCK;
     * P0.8 - MISO
     * P0.9 - MOSI
     * P2.2 - SSEL - used as GPIO
     */
    PinCfg.Funcnum = 2;
    PinCfg.OpenDrain = 0;
    PinCfg.Pinmode = 0;
    PinCfg.Portnum = 0;
    PinCfg.Pinnum = 7;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 8;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 9;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Funcnum = 0;
    PinCfg.Portnum = 2;
    PinCfg.Pinnum = 2;
    PINSEL_ConfigPin(&PinCfg);

    SSP_ConfigStructInit(&SSP_ConfigStruct);

    // Initialize SSP peripheral with parameter given in structure above
    SSP_Init(LPC_SSP1, &SSP_ConfigStruct);

    // Enable SSP peripheral
    SSP_Cmd(LPC_SSP1, ENABLE);

}
//<-----------------------------------------END OF SSP--------------------------------------------------//

//--------------------------------------------------------------------------------------------------------->//
//                                                          I2C
//---------------------------------------------------------------------------------------------------------->//
static void init_i2c(void)
{
    PINSEL_CFG_Type PinCfg;

    /* Initialize I2C2 pin connect */
    PinCfg.Funcnum = 2;
    PinCfg.Pinnum = 10;
    PinCfg.Portnum = 0;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 11;
    PINSEL_ConfigPin(&PinCfg);

    // Initialize I2C peripheral
    I2C_Init(LPC_I2C2, 100000);

    /* Enable I2C1 operation */
    I2C_Cmd(LPC_I2C2, ENABLE);
}
//<----------------------------END OF I2C----------------------------------------//

//--------------------------------------------------------------------------------------->//
//                                      GPIO
//--------------------------------------------------------------------------------------->//
static void init_GPIO(void)
{
	PINSEL_CFG_Type PinCfg;			//START of PinCfg

	//Initialize button SW2
	// TBA

	// Initialize button SW3 (P2.10)
	PinCfg.Portnum = 2;
	PinCfg. Pinnum = 10;
	PinCfg.Funcnum = 0;
	PinCfg.OpenDrain = 0;
	PinCfg. Pinmode = 0;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(2, 1<<10, 0);

	// Initialize button SW4 (P1.31)
	PinCfg.Funcnum = 0;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 1;
	PinCfg.Pinnum = 31;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(1, 1<<31, 0); //Parameters: portNum, bitValue, dir (0: input, 1: output)


	// Initialize button SW5
	// TBA

	//Red LED (P2.0)
	PinCfg.Funcnum = 0;
	PinCfg.Pinnum = 0;
	PinCfg.Portnum = 2;
	PINSEL_ConfigPin(&PinCfg);

	//Blue LED (P0.26)
	PinCfg.Funcnum = 0;
	PinCfg.Pinnum = 26;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);

	//Green LED (P2.1)
	PinCfg.Funcnum = 0;
	PinCfg.Pinnum = 1;
	PinCfg.Portnum = 2;
	PINSEL_ConfigPin(&PinCfg);


	//Temperature Sensor (P0.2)
	PinCfg.Funcnum = 0;
	PinCfg.Pinnum = 2;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);

	//Light Sensor (P2.5)
	PinCfg.Funcnum = 0;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 2; // Port 2
	PinCfg. Pinnum = 5; // Pin 5
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(2, (1 << 5), 0);

    /* ---- Speaker ------> */

    GPIO_SetDir(2, 1<<0, 1);
    GPIO_SetDir(2, 1<<1, 1);

    GPIO_SetDir(0, 1<<27, 1);
    GPIO_SetDir(0, 1<<28, 1);
    GPIO_SetDir(2, 1<<13, 1);
    GPIO_SetDir(0, 1<<26, 1);

    GPIO_ClearValue(0, 1<<27); //LM4811-clk
    GPIO_ClearValue(0, 1<<28); //LM4811-up/dn
    GPIO_ClearValue(2, 1<<13); //LM4811-shutdn

    /* <---- Speaker ------ */
}

//<-------------------------END OF GPIO-------------------------------------------//



//----------------------------------------------------------------------->//
//                                      UART
//----------------------------------------------------------------------->//

// UART Receive Callback Function
void UART_Received(void)
{
	if (UART_Receive(LPC_UART3, &rev_buf[rev_cnt], 1, NONE_BLOCKING) == 1)
	{
		if (rev_buf[rev_cnt] == '\r')
		{
			isReceived = 1;
		}
		rev_cnt++;
		if (rev_cnt == 255) rev_cnt = 0;
	}
}


void UART_init(void)
{
	//Debug
	//printf("UART Initialised \n");

	// PINSEL Configuration
	PINSEL_CFG_Type CPin;
	CPin.OpenDrain = 0;
	CPin.Pinmode = 0;
	CPin.Funcnum = 2;
	CPin.Pinnum = 0;							//TXD3
	CPin.Portnum = 0;
	PINSEL_ConfigPin(&CPin);
	CPin.Pinnum = 1;							//RXD3
	CPin.Portnum = 0;
	PINSEL_ConfigPin(&CPin);

	// UART Configurations
	UART_CFG_Type UCfg;
	UCfg.Baud_rate = 115200;
	UCfg.Databits = UART_DATABIT_8;
	UCfg.Parity = UART_PARITY_NONE;
	UCfg.Stopbits = UART_STOPBIT_1;

	// Supply Power & Setup Parameters
	UART_Init(LPC_UART3, &UCfg);

	// Enable UART RX Interrupt
	UART_IntConfig(LPC_UART3, UART_INTCFG_RBR, ENABLE);


	//UART FIFO Config
	UART_CFG_Type UARTFIFOConfigStruct;

	/* Init FIFOConfig to default state,
	     * using FIFO will allow LPC to have more time to handle interrupts,
	     * and also prevent data loss at high rate
	*/
	UART_FIFOConfigStructInit(&UARTFIFOConfigStruct);
	//Init FIFO for UART3
	UART_FIFOConfig(LPC_UART3, &UARTFIFOConfigStruct);

	// Setup callback for Receive Message
	UART_SetupCbs(LPC_UART3, 0, (void *)UART_Received);
	// Enable UART Transmit
	UART_TxCmd(LPC_UART3, ENABLE);
	/* Enable UART Rx interrupt */
	UART_IntConfig(LPC_UART3, UART_INTCFG_RBR, ENABLE);


}

//---------------------------END OF UART----------------------------->//

//----------------------------------------------------------------->//
//                      READ/CONFIGURE ALL
//----------------------------------------------------------------->//

void all_config(){
    //Configure SysTick to generate an interrupt every TicksPerSec (1ms)
    //SystemCoreClock is the system clock frequency supplied to the SysTick time
    SysTick_Config(SystemCoreClock/1000);


    //Accelerometer Config — Assume baseboard in zero-g position
    acc_read(&x, &y, &z);
    xoff = 0-x;
    yoff = 0-y;
    zoff = 64-z;

    // OLED
    oled_clearScreen(OLED_COLOR_BLACK);

    // Interrupts/Exceptions
        LPC_GPIOINT->IO2IntEnF |=1<<10;
        NVIC_EnableIRQ(EINT3_IRQn);

    //Configure Light
	light_setRange(LIGHT_RANGE_4000);
	//light_setHiThreshold (300);									//Debug Purpose
	light_setHiThreshold (LIGHT_THRESHOLD);
	light_setLoThreshold (40);
	light_setIrqInCycles(LIGHT_CYCLE_8);
	light_clearIrqStatus();
}

// systick_delay - creates a delay of the appropriate number of Systicks (happens every 1 ms)
__INLINE static void systick_delay (uint32_t delayTicks) {
uint32_t currentTicks;
currentTicks = msTicks; // read current tick counter
// Now loop until required number of ticks passes
while ((msTicks - currentTicks) < delayTicks);
}
//////////////////
void all_read(){
    //Light Config
    light_value = light_read();

    //Temperature Config
    temp_value = temp_read();
}


//Function to retrieve prescalar for us (micro second)
unsigned int getPrescalarForUs(uint8_t timerPclkBit)
{
    unsigned int PCLK;																					//Peripheral Clock
    unsigned int prescalarForUs;
    PCLK = (LPC_SC->PCLKSEL0 >> timerPclkBit) & 0x03;  					//Get pclk for timer

    switch ( PCLK )                                    														//Determine the pclk
    {
    case 0x00:
        PCLK = SystemCoreClock/4;
        break;

    case 0x01:
        PCLK = SystemCoreClock;
        break;

    case 0x02:
        PCLK = SystemCoreClock/2;
        break;

    case 0x03:
        PCLK = SystemCoreClock/8;
        break;

    default:
        PCLK = SystemCoreClock/4;
        break;
    }
    prescalarForUs = PCLK/1000000 - 1;                    										//Prescalar for 1us (1000000Counts/sec)
    return prescalarForUs;
}

//EINT0 and EINT1 initiliase (p25)
void EINT01_init()
{
	LPC_SC->EXTINT      = (1<<SBIT_EINT0)    | (1<<SBIT_EINT1);	    						// Clear Pending interrupts
	LPC_PINCON->PINSEL4 = (1<<PINSEL_EINT0)  | (1<<PINSEL_EINT1);   			//Config P2.10 as EINT0 (SW3) and P2.11 as EINT1
	LPC_SC->EXTMODE     = (1<<SBIT_EXTMODE0) | (1<<SBIT_EXTMODE1);  		//Configure EINTx as Edge Triggered
	LPC_SC->EXTPOLAR    = (1<<SBIT_EXTPOLAR0)| (1<<SBIT_EXTPOLAR0);  	//Configure EINTx as Falling Edge

}

// TIMER Initialise
void timer_init(void)
{
	//TIMER0
	LPC_SC->PCONP |= (1<<SBIT_TIMER0); 												// Power ON Timer0
	LPC_TIM0->MCR  = (1<<SBIT_MR0I)|(1<<SBIT_MR0R);						// Clear TC on MR0 match and Interrupts
	LPC_SC->PCLKSEL0 |= 0 << PCLK_TIMER0;										//CCLK/4
	LPC_TIM0->PR   = getPrescalarForUs(PCLK_TIMER0);      					///Prescalar for 1us
	LPC_TIM0->MR0  = MilliToMicroSec(333);                 								//Load timer value to generate 333ms delay
	LPC_TIM0->TCR  = (1 <<SBIT_CNTEN);                    								//Start timer by setting the Counter Enable

	//TIMER1
	LPC_SC->PCONP |= (1<<SBIT_TIMER1);												//Power ON Timer1
	LPC_TIM1->MCR  = (1<<SBIT_MR0I) | (1<<SBIT_MR0R);     				//Clear TC on MR0 match and Generate Interrupt
	LPC_SC->PCLKSEL0 |= 0 << PCLK_TIMER1;
	LPC_TIM1->PR   = getPrescalarForUs(PCLK_TIMER1);      					//Prescalar for 1us
	LPC_TIM1->MR0  = MilliToMicroSec(500);                 								//Load timer value to generate 500ms delay
	LPC_TIM1->TCR  = (1 <<SBIT_CNTEN);                    								//Start timer by setting the Counter Enable


}

void NVIC_init(void)
{

	//Parameters: Priority Grouping (PG), Pre-Empt Priority (PP), Sub-Priority (SP)
	uint32_t PG = 5;
	NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(PG, 0b00, 0b00));

	NVIC_SetPriority(EINT0_IRQn, NVIC_EncodePriority(PG, 0b11, 0b001));
	NVIC_SetPriority(EINT1_IRQn, NVIC_EncodePriority(PG, 0b11, 0b001));
	NVIC_SetPriority(EINT3_IRQn,NVIC_EncodePriority(PG,0b10,0b000));
	NVIC_SetPriority(UART3_IRQn,NVIC_EncodePriority(PG,0b10,0b01));

	NVIC_ClearPendingIRQ(EINT0_IRQn);
	NVIC_ClearPendingIRQ(EINT1_IRQn);
	NVIC_ClearPendingIRQ(EINT3_IRQn);
	NVIC_ClearPendingIRQ(UART3_IRQn);


//	NVIC_EnableIRQ(EINT0_IRQn);
//	NVIC_EnableIRQ(EINT1_IRQn);
	NVIC_EnableIRQ(EINT3_IRQn);
	NVIC_EnableIRQ(UART3_IRQn);


	NVIC_SetPriority(TIMER0_IRQn, NVIC_EncodePriority(PG,0b01,0b000));
	NVIC_SetPriority(TIMER1_IRQn, NVIC_EncodePriority(PG,0b01,0b000));
	NVIC_ClearPendingIRQ(TIMER0_IRQn);
	NVIC_ClearPendingIRQ(TIMER1_IRQn);

	//Problematic
//	NVIC_EnableIRQ(TIMER0_IRQn); 							//Enable Timer0 Interrupt
//	NVIC_EnableIRQ(TIMER1_IRQn);								//Enable Timer1 Interrupt

}

//<------------END OF READ/CONFIGURATIONS-----------//

//----------------------------------------------------------------------------------------->//
//                              INITIALISE ALL
//----------------------------------------------------------------------------------------->//
void all_init()
{
	//INITIALISE
    pca9532_init();					//16LED
    joystick_init();						//Joystick
    acc_init();							//Accelerometer
    oled_init();							//OLED
    led7seg_init();					//SSEG
    rgb_init();								//RGB LED
    temp_init(getTicks);			//Temp Sensor
    light_init();							//Light Sensor
    UART_init();						//UART
    EINT01_init();						//EINT0, EINT1
    timer_init();							//TIMER0, TIMER1
    NVIC_init();

	//ENABLES
    light_enable();

            // Enable GPIO Interrupt P2.5 for light sensor
         NVIC_ClearPendingIRQ(EINT3_IRQn);
            LPC_GPIOINT->IO2IntEnF |= 1 << 5;
          //  NVIC_EnableIRQ(EINT3_IRQn);

}


//<-----------------------END OF INITIALISE ALL------------------------------//


//---------------------------------------------------------------------->//
//          OTHER USER-DEFINED FUNCTIONS
//---------------------------------------------------------------------->//
int reset1 = 1;
int rgbtimer1;
int reset2 = 1;
int rgbtimer2 = 1;

void tempBLINK()
{
	switch (BLINK_MODE)
		{

		//BLINK_NONE
		case BLINK_NONE:
			GPIO_ClearValue(2, 1 << 0);

			GPIO_ClearValue(0, 1<<26);
		break;

		//BLINK_RED
		case BLINK_RED:
			if (reset1 == 1) {
			rgbtimer1=msTicks;
			reset1 = 0;
			GPIO_ClearValue( 2, 1);
			}
			else {
				if (msTicks-rgbtimer1 >= 333) {
				       GPIO_SetValue( 2, 1);
				       reset1 = 1;
				}
			}
		break;
	/*
		//BLINK_GREEN
			case BLINK_GREEN:
				if ((GPIO_ReadValue(2) >> 1)  & 0x01)
				{
					GPIO_ClearValue(2, 1<<1);
				}
				else
				{
					GPIO_SetValue( 2, (1<<1) );
				}
			break;
	*/
		//BLINK_BLUE
			case BLINK_BLUE:
				if (reset2 == 1) {
				rgbtimer2=msTicks;
				reset2 = 0;
				GPIO_ClearValue( 0, (1<<26) );
				}
				else {
					if (msTicks-rgbtimer2 >= 333) {
						 GPIO_SetValue( 0, (1<<26) );
					       reset2 = 1;
					}
				}
			break;

		//BLINK_BOTH — Combine RED & GREEN case
		case BLINK_BOTH:
			if (reset2 == 1) {
						rgbtimer2=msTicks;
						reset2 = 0;
						GPIO_ClearValue( 0, (1<<26) );
						GPIO_ClearValue( 2, 1);
						}
						else {
							if (msTicks-rgbtimer2 >= 333) {
								 GPIO_SetValue( 0, (1<<26) );
							       reset2 = 1;
							}
						}
		break;

//		default:
//
//		break;
		}
}

//Function to Increment 7 Segment
int Inc7_Counter=0;
int SSEG_RESET = 0;
void Inc_7 (){
    if (Inc7_Counter > 15){
        Inc7_Counter = 0;
        RAT_COUNTER++;
        UART_SendString(LPC_UART3,(uint8_t *) UART_msg);
    }
    //led7seg_setChar(ASCII_list[Inc7_Counter], FALSE); 				//Displays standard 7seg
    led7seg_setChar(inv_ASCII[Inc7_Counter], TRUE);			//Inverted SSEG dislay for better readability
    if (SSEG_RESET == 0)
    {
        SSEG_Timer = msTicks;
        SSEG_RESET = 1;
    }
    else
    {
        if (msTicks - SSEG_Timer >= 1000)
        {
            Inc7_Counter++;
            SSEG_RESET = 0;
        }
    }
}



void toggleMode_1sec(){
    switch (MODE_TOGGLE) {

        case 0:
        	pca9532_setLeds(0x0000, 0xffff);
        	GPIO_ClearValue( 2, 1);
        	        	GPIO_ClearValue( 0, (1<<26) );
            led7seg_setChar('S', FALSE);
            if (n>0){
            red = 1;
            blue = 1;
            }
        Mode_Stationary();

        break;

        case 1:


        	pca9532_setLeds(0x0000, 0xffff);
        Mode_Forward();
n++;

        break;

        case 2:
            led7seg_setChar('B', FALSE);
        	GPIO_ClearValue( 2, 1);
        	GPIO_ClearValue( 0, (1<<26) );
        Mode_Backward();
        break;
        }

        if (EINT3_RESET == 1) //When not interrupted, count for 1s
        {
            if (msTicks - EINT3_Timer > 1000)
            {
                EINT3_RESET = 0;
                if (MODE_TOGGLE == 0)
                {
//                    printf("Exceeded - Forward \n");
                    oled_clearScreen(OLED_COLOR_BLACK);
                    UART_SendString(LPC_UART3,(uint8_t *) "Entering Forward Mode \r\n");
                    MODE_TOGGLE = 1;
                }
                else
                {
//                    printf("Exceeded - Stationary \n");
                    oled_clearScreen(OLED_COLOR_BLACK);
                    EINT3_RESET = 0;
                    UART_SendString(LPC_UART3,(uint8_t *) "Entering Stationary Mode \r\n");
                    MODE_TOGGLE = 0;
                }
            }
        }
        else if (EINT3_RESET == 2)
        {
            if (msTicks - EINT3_Timer > 1000)
            {
                oled_clearScreen(OLED_COLOR_BLACK);
                EINT3_RESET = 0;
                UART_SendString(LPC_UART3,(uint8_t *) "Entering Reverse Mode \r\n");
                MODE_TOGGLE = 2;
            }
        }
}


void forwardTelemetry(void)
{
	//RAT_COUNTER++;
	switch (BLINK_MODE)
	{
	//TEMP_THRESHOLD Exceeded
	case BLINK_RED:
		UART_SendString(LPC_UART3,(uint8_t *)"Temp. too high.\r\n");
		break;

	//LIGHT_THRESHOLD Exceeded
	case BLINK_BLUE:
		UART_SendString(LPC_UART3,(uint8_t *)"Collision detected.\r\n");
		break;

	//TEMP_THRESHOLD & LIGHT_THRESHOLD Exceeded (RED case > BLUE case)
	case BLINK_BOTH:
		UART_SendString(LPC_UART3,(uint8_t *)"Temperature too high.\r\n");
		UART_SendString(LPC_UART3,(uint8_t *)"Collision detected.\r\n");
		break;

	default:
		break;
	}
	//Format: "XXX_Temp_yy.zz_ACC_aa.xx\r\n"
	sprintf(UART_msg, "%03d_Temp %.2f_ACC_%.2f \r\n", RAT_COUNTER, temp_value/10.0, x/64.0);
	//UART_SendString(LPC_UART3,(uint8_t *) UART_msg);
}



//#####################################################################
//MODE SEGMENT
//#####################################################################


void Mode_Stationary ()
{
//  printf("STATIONARY MODE \n");
    sprintf(display_acc, "Stationary");     //Works but not optimal
    oled_putString(5, 5, display_acc, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    BLINK_MODE = BLINK_NONE;
}

//---------------------------------STATIONARY------------------------------>//

/* Forward Mode Requirements
 * DEFAULT MODE
    1. Sensors and Actuators are enabled (DONE)
    2. Send to RAT "Entering Forward mode. \r\n" — \r\n is new line for Windows OS
    3. GRAPHICS_DISPLAY must display "Forward" (DONE)
    4. SEGMENT_DISPLAY hexadecimal value inceases by 1 for every second that passes (A - F) (DONE)
    5. TEMPERATURE_SENSOR and ACCELEROMETER are sampled every second. (???)
    6. Current readings from TEMPERATURE_SENSOR and ACCELEROMETER are updated on
        GRAPHICS_DISPLAY every time the SEGMENT_DISPLAY shows "5 / A / F" (DONE)
    7. Transmission of current readings from TEMPERATURE_SENSOR and ACCELEROMETER to RAT
        every time SEGMENT_DISPLAY shows "F"
    8. Format of transmitted data should be as follows: "XXX_Temp_yy.zz_ACC_aa.xx\r\n"

        capxxx represents a 3-digit value that starts from 000 and increments by 001 after each transmission to RAT, from CATS.
        where aa.xx is the x-axis reading from the accelerometer, in 'g's, up to 2 decimal places.
        yy.zz stands for the temperature reading from the sensor up to 2 decimal places.
        capxxx never resets itself to 000, unless CATS itself is powered on from a power off state. It is assumed that 999 will never be
        reached.

        To be continued...
*/

void Mode_Forward (){


	uint32_t prevForwardTicks = getTicks();
	//forwardTelemetry();
    Inc_7();

    prevForwardTicks = getTicks();
    acc_read(&x, &y, &z);
    x = x+xoff;
    y = y+yoff;
    z = z+zoff;

    sprintf(display_mode, "Forward");   //Works but not optimal
    oled_putString(5, 5, display_mode, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    if (Inc7_Counter==(5)||Inc7_Counter==(10)||Inc7_Counter==(15))
    {
		sprintf(display_xyz, "%.2f|%.2f|%.2f\n", x/64.0, y/64.0, z/64.0);
		sprintf(display_acc, "Accel: %2.2f g", x/64.0);
		sprintf(display_temp, "Temp (C): %2.2f", temp_value/10.0);
		//oled_putString(5, 16, "x|y|z: \n", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		oled_putString(5, 16, display_xyz, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		oled_putString(5, 26, display_acc, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		oled_putString(5, 36, display_temp, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    }
    if (Inc7_Counter == 15)
    {
    	forwardTelemetry();
    }
    if (x>=ACC_THRESHOLD ||x<= -(ACC_THRESHOLD)){
    	ACC_FLAG = 1;
    }
  else
  {
	  ACC_FLAG = 0;
    }

    if ((temp_value/10.0)>=TEMP_THRESHOLD) {
//printf("Temperature Exceeded 32");
    	TEMP_FLAG = 1;
    }
    else
    {
    	TEMP_FLAG = 0;
    }

}



//-------------------------- FORWARD--------------------------------->//


void Mode_Backward (){

    //printf("REVERSE MODE \n");
	BLINK_MODE = BLINK_NONE;
ledlight();
    sprintf(display_mode, "Reverse");  //Works but not optimal
    sprintf(display_lux, "Lux: %" PRIu32 " \n", light_value);
    oled_putString(5, 5, display_mode, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    oled_putString(5, 16, display_lux, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
}

void FLAGhandler()
{
	if (!ACC_FLAG && TEMP_FLAG ){
		BLINK_MODE = BLINK_NONE;
		BLINK_MODE = BLINK_RED;
		oled_putString(5, 46, "Temp. too high", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	}
	if (ACC_FLAG && !TEMP_FLAG){
		BLINK_MODE = BLINK_NONE;
		BLINK_MODE = BLINK_BLUE;
		oled_putString(5, 46, "Airbag released", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	}
	if (ACC_FLAG && TEMP_FLAG){
		BLINK_MODE = BLINK_NONE;
		BLINK_MODE = BLINK_BOTH;
		oled_putString(5, 46, "Temp , Air", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	}
	if (LIGHT_FLAG)
	{
		LPC_GPIOINT->IO2IntEnF &= ~(1<<5);			//Disable Light Interrupt
		oled_putString(5, 36, "Obstacle Near", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	}

//	if (!ACC_FLAG && !TEMP_FLAG){
//		BLINK_MODE = BLINK_NONE;
//	}


}
//#####################################################################
//END OF MODE SEGMENT
//#####################################################################


//<---------------END OF USER-DEFINED FUNCTIONS-----------------//


//#######################################################
//#######################################################
//                                          MAIN BLOCK
//#######################################################
//#######################################################

int main (void) {
    init_i2c();
    init_ssp();
    init_GPIO();

    all_init();
    all_config();
    light_enable();
    all_config();


    while (1)
    { all_read();

    	tempBLINK();
        toggleMode_1sec(); //Toggles Mode after 1 second
        FLAGhandler();
        /* ####### Joystick and OLED  ###### */
        /* # */
        state = joystick_read(); //Lags when temperature is read...
        if (state != 0)
        {
            drawOled(state);
        }
        //Play Song — Everything else pauses???

    } //End of MAIN WHILE

}
//<---------------------END OF MAIN----------------------------------//

//------------------------------------------------------------------------------>
//                      INTERRUPT HANDLERS
//------------------------------------------------------------------------------>
void check_failed(uint8_t *file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while(1);
}

//Exception Handler runs every 1ms
void SysTick_Handler(void)
{
    msTicks++;

}

//EINT3 INTERRUPT Handler//

void EINT0_IRQHandler(void)
{
	LPC_SC->EXTINT = (1<<SBIT_EINT0);  	//Clear Interrupt Flag
	//LPC_GPIO2->FIOPIN ^= (1<< LED1);   		//Toggle LED1 every time INTR0 is active
	/*
	 EINT0_RESET++;
	 */
}

void EINT1_IRQHandler(void)
{
    LPC_SC->EXTINT = (1<<SBIT_EINT1);  /* Clear Interrupt Flag */
}



void EINT3_IRQHandler(void)
{

    if (((LPC_GPIOINT->IO2IntStatF >> 10) & 0x01) & (EINT3_RESET == 0)) //Flag to check if passed timer
    {
//        printf("Pending Status \n");
        EINT3_Timer = msTicks; // Fixed value EINT3_Timer
        EINT3_RESET = 1;
        LPC_GPIOINT->IO2IntClr = 1 << 10; //Clear Interrupt Status
    }

    else if (((LPC_GPIOINT->IO2IntStatF >> 10) & 0x01) & (EINT3_RESET == 1))
    {
//        printf("Confirmed Status: Reverse \n");
        EINT3_RESET = 2;
        //MODE_TOGGLE = 2;
        //printf("Entering Reverse \n");
        LPC_GPIOINT->IO2IntClr = 1 << 10; //Clear Interrupt Status
    }
    if ((LPC_GPIOINT->IO2IntStatF >> 5)& 0x1){
         LPC_GPIOINT->IO2IntClr |= 1 << 5;
         light_clearIrqStatus();
         light_value = light_read();
         LIGHT_FLAG = 1;
     //    printf("Threshold Exceeded @: %u \n", light_value);
     }else{
    	 LIGHT_FLAG = 0;
     }

    // Temperature Sensor Interrupt (Calculation retrieve from temp.c) (Active HIGH)
    if ((LPC_GPIOINT->IO0IntStatR>>2)& 0x1)				//Read Digital HIGH
    {
    	//printf("TEMP INTERRUPT ACTIVE \n");
    	if (periodCounter == 0)
    {
    		t1 = getTicks(); 														//Start timer for period = 0
    		periodCounter++;
    }
    	else if (periodCounter == tempPeriods)
    {
    		t2 = getTicks();
    		if (t2 > t1)
    		{
    			t2 = t2 - t1;
    		}
    		else
    		{
    			t2 = (0xFFFFFFFF - t1 + 1) + t2;
    		}
    		LPC_GPIOINT->IO0IntEnR &= ~(1<<2); 			//Disable Temp Interrupt
    		periodCounter = 0;
    }
    else
    {
    	periodCounter++;
    }
    	LPC_GPIOINT->IO0IntClr = 1<<2;							//Clear Pending
    }
}

void UART3_IRQHandler(void)
{
	UART3_StdIntHandler();				//Based on lpc17xx_art.c
}


//<--------------------END OF INTERRUPTS-----------------------//



