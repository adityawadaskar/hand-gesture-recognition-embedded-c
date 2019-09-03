#define NDEBUG  //disable assert
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <board.h>
#include <assert.h>
#include <math.h>
#include "Fusion.h"
#include "LEDStrip.h"

#include "board.h"
#include "stdio.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/
#define GPIO_INTERRUPT_PIN     10			  // GPIO pin number mapped to interrupt
#define GPIO_INTERRUPT_PORT    GPIOINT_PORT2  // GPIO port number mapped to interrupt
#if defined(BOARD_EA_DEVKIT_1788) || defined(BOARD_EA_DEVKIT_4088)
#define UART_SELECTION 	LPC_UART0
#define IRQ_SELECTION 	UART0_IRQn
#define HANDLER_NAME 	UART0_IRQHandler
#elif defined(BOARD_NXP_LPCXPRESSO_1769)
#define UART_SELECTION 	LPC_UART3
#define IRQ_SELECTION 	UART3_IRQn
#define HANDLER_NAME 	UART3_IRQHandler
#else
#error No UART selected for undefined board
#endif

STATIC RINGBUFF_T txring, rxring;

/*Important variables*/
char temperature[3];	//holds temperature
bool fAlarmTimeMatched = false;	//true if alarm matched

/* Transmit and receive ring buffer sizes */
#define UART_SRB_SIZE 128	/* Send */
#define UART_RRB_SIZE 32	/* Receive */

/* Transmit and receive buffers */
static uint8_t rxbuff[UART_RRB_SIZE], txbuff[UART_SRB_SIZE];

const char inst1[] = "LPC17xx/40xx UART example using ring buffers\r\n";
const char inst2[] = "Press a key to echo it back or ESC to quit\r\n";

void HANDLER_NAME(void)
{
	/* Want to handle any errors? Do it here. */

	/* Use default ring buffer handler. Override this with your own
	   code if you need more capability. */
	Chip_UART_IRQRBHandler(UART_SELECTION, &rxring, &txring);
}

//
#define sq(x)   		                ((x)*(x))
/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/
#define DEFAULT_I2C          I2C0

#define I2C_EEPROM_BUS       DEFAULT_I2C
#define I2C_IOX_BUS          DEFAULT_I2C

#define SPEED_100KHZ         100000
#define SPEED_400KHZ         400000

#define GPIO_LED_PIN     10				// GPIO pin number mapped to LED toggle
#define GPIO_LED_PORT    GPIOINT_PORT2	// GPIO port number mapped to LED toggle
#define GPIO_IRQ_HANDLER  			GPIO_IRQHandler/* GPIO interrupt IRQ function name */
#define GPIO_INTERRUPT_NVIC_NAME    GPIO_IRQn	/* GPIO interrupt NVIC interrupt name */


static int mode_poll;   /* Poll/Interrupt mode flag */
static I2C_ID_T i2cDev = DEFAULT_I2C; /* Currently active I2C device */

/* EEPROM SLAVE data */
#define I2C_SLAVE_EEPROM_SIZE       64
#define I2C_SLAVE_EEPROM_ADDR       0x5A
#define I2C_SLAVE_TEMP_ADDR         0x68

/* Xfer structure for slave operations */
static I2C_XFER_T temp_xfer;
static I2C_XFER_T iox_xfer;

static uint8_t i2Cbuffer[2][256];



/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* State machine handler for I2C0 and I2C1 */
static void i2c_state_handling(I2C_ID_T id)
{
	if (Chip_I2C_IsMasterActive(id)) {
		Chip_I2C_MasterStateHandler(id);
	} else {
		Chip_I2C_SlaveStateHandler(id);
	}
}

/* Set I2C mode to polling/interrupt */
static void i2c_set_mode(I2C_ID_T id, int polling)
{
	if(!polling) {
		mode_poll &= ~(1 << id);
		Chip_I2C_SetMasterEventHandler(id, Chip_I2C_EventHandler);
		NVIC_EnableIRQ(id == I2C0 ? I2C0_IRQn : I2C1_IRQn);
	} else {
		mode_poll |= 1 << id;
		NVIC_DisableIRQ(id == I2C0 ? I2C0_IRQn : I2C1_IRQn);
		Chip_I2C_SetMasterEventHandler(id, Chip_I2C_EventHandlerPolling);
	}
}

/* Initialize the I2C bus */
static void i2c_app_init(I2C_ID_T id, int speed)
{
	Board_I2C_Init(id);

	/* Initialize I2C */
	Chip_I2C_Init(id);
	Chip_I2C_SetClockRate(id, speed);

	/* Set default mode to interrupt */
	i2c_set_mode(id, 0);
}

static void i2c_read_setup(I2C_XFER_T *xfer, uint8_t addr, int numBytes)
{
	xfer->slaveAddr = addr;
	xfer->rxBuff = 0;
	xfer->txBuff = 0;
	xfer->txSz = 0;
	xfer->rxSz = numBytes;
	xfer->rxBuff = i2Cbuffer[1];

}

bool fDebouncing;
void GPIO_IRQ_HANDLER(void)
{
	if(fDebouncing) {}  // If not debouncing
	else {
		// Toggle LED
		Board_LED_Toggle(0);  // Toggle LED0
		takeInput = !takeInput;
		// Start debounce delay
		fDebouncing = true;  // Update boolean variable

		// Start timer here
		Chip_TIMER_Enable(LPC_TIMER0);  // Start TIMER0
	}
	Chip_GPIOINT_ClearIntStatus(LPC_GPIOINT, GPIO_INTERRUPT_PORT, 1 << GPIO_INTERRUPT_PIN);  // Clear interrupt flag
}

void Configure_GPIO_Interrupt(){
	// Turn on LED0
	Board_LED_Set(0, false);

	// Initialize GPIO button pin as input
	Chip_IOCON_PinMux(LPC_GPIO, GPIO_LED_PORT, GPIO_LED_PIN, IOCON_FUNC0, IOCON_MODE_PULLUP);	// Configures pin as GPIO w/ pullup resistor
	Chip_GPIO_SetPinDIRInput(LPC_GPIO, GPIO_LED_PORT, GPIO_LED_PIN);  // Configures pin as an input

	// Configure the GPIO interrupt
	Chip_GPIOINT_SetIntFalling(LPC_GPIOINT, GPIO_LED_PORT, 1 << GPIO_LED_PIN);  // Configure GPIO to trigger an interrupt when input falls from high to low

	// Enable interrupt in the NVIC
	NVIC_ClearPendingIRQ(GPIO_INTERRUPT_NVIC_NAME);  // Clear pending interrupt flag
	NVIC_EnableIRQ(GPIO_INTERRUPT_NVIC_NAME);  // Enable interrupt handling in NVIC

}
/*****************************************************************************
 * Public functions
 ****************************************************************************/
/**
 * @brief	SysTick Interrupt Handler
 * @return	Nothing
 * @note	Systick interrupt handler updates the button status
 */
void SysTick_Handler(void)
{
}

/**
 * @brief	I2C Interrupt Handler
 * @return	None
 */
void I2C1_IRQHandler(void)
{
	i2c_state_handling(I2C1);
}

/**
 * @brief	I2C0 Interrupt handler
 * @return	None
 */
void I2C0_IRQHandler(void)
{
	i2c_state_handling(I2C0);
}

/**
 * @brief	Main program body
 * @return	int
 */
void MPU6050_Init(void)
{
	MPU6050_Write(0x6B,0b00000001);  //device
	MPU6050_Write(0x1B,0b00000000);  //Gyro
	MPU6050_Write(0x1C,0b00000000);  //Acceleration
	MPU6050_Write(0x6B,0b00000001);  //Device
	MPU6050_Write(0x1A,0b00000110);	 //DLPF
}

//Read the register value and output it
void MPU6050_Read(uint8_t Reg){
	uint8_t rx_data[1];
	int k = Chip_I2C_MasterCmdRead(I2C0,0x68,Reg,rx_data,1);
	printf("the value of %d is set to %d \r\n",Reg,rx_data[0]);
}

/*Referencing MPU6050 datasheet page 35  assuming Chip_I2C_MasterSend Does AD+W for us
 * return 1 if we succefully write tto the register
 */

void MPU6050_Write(uint8_t Reg, uint8_t value){
	uint8_t data[2];
	data[0] = Reg;
	data[1] = value;
	int k1 = Chip_I2C_MasterSend(I2C0, 0x68 , data, sizeof(data));
	assert(k1 > 0);
}
/*
 * Read Acceleration, Temperature, and Gyro data
 * discard Temperature,  and return a pointer of 12 byte containing the 6 DOF Motion Data
 */


void MPU6050_GetMotionData(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz){
	float ratio1 = 0.00119751;
	float ratio2 = 0.007629395;
	uint8_t rx_data[14];
	//printf("INSIDE MOTIONDATA\n");

	int k = Chip_I2C_MasterCmdRead(I2C0,0x68,0x3B,rx_data,14);
	//printf("INSIDE MOTIONDATA_@@@@\n");
	assert(k==14);
	//memcpy(agBuf, rx_data, 14);		//Copy rx_data into agBuf

	//uint16_t MotionData[6];
	*ax = rx_data[0]<<8 | rx_data[1];
	*ay = rx_data[2]<<8 | rx_data[3];
	*az = rx_data[4]<<8 | rx_data[5];
	*gx = rx_data[8]<<8 | rx_data[9];
	*gy = rx_data[10]<<8 | rx_data[11];
	*gz = rx_data[12]<<8 | rx_data[13];
	/*
	*ax=(*ax)*ratio1;
	*ay=(*ay)*ratio1;
	*az=(*az)*ratio1;
	*gx=(*gx)*ratio2;
	*gy=(*gy)*ratio2;
	*gz=(*gz)*ratio2;
	*gz=(*gz)*ratio2;
	*/
	/*printf("X Acceleration: %d\n", ax);
	printf("Y Acceleration: %d\n", ay);
	printf("Z Acceleration: %d\n", az);*/


}


void TIMER0_IRQHandler(void)
{
	fDebouncing = false; 				  // Update boolean variable
	Chip_TIMER_Disable(LPC_TIMER0);		  // Stop TIMER0
	Chip_TIMER_Reset(LPC_TIMER0);		  // Reset TIMER0
	Chip_TIMER_ClearMatch(LPC_TIMER0,0);  // Clear TIMER0 interrupt
}

void Timer0_setup(){
	int PrescaleValue0 = 120000;

	// Initialize TIMER0
	Chip_TIMER_Init(LPC_TIMER0);						// Initialize TIMER0
	Chip_TIMER_PrescaleSet(LPC_TIMER0,PrescaleValue0);  // Set prescale value
	Chip_TIMER_SetMatch(LPC_TIMER0,0,200);				// Set match value
	Chip_TIMER_MatchEnableInt(LPC_TIMER0, 0);			// Configure to trigger interrupt on match

	// Enable timer interrupt
	NVIC_ClearPendingIRQ(TIMER0_IRQn);
	NVIC_EnableIRQ(TIMER0_IRQn);

	// Enable timer and RTC
	Chip_TIMER_Enable(LPC_TIMER0);
}
bool SamplingFlag=0;
void TIMER1_IRQHandler(void)
{
	//printf("HELLONIGGA\n");
	if (Chip_TIMER_MatchPending(LPC_TIMER1, 0)) {
		//printf("HELLONIGGA\n");
		SamplingFlag = 1;
		Chip_TIMER_Reset(LPC_TIMER1);	// Reset TIMER0
		Chip_TIMER_ClearMatch(LPC_TIMER1,0);  // Clear TIMER0 interrupt
	}
}
void Timer1_setup(){
	int PrescaleValue1 = 120000;

	// Initialize TIMER0
	Chip_TIMER_Init(LPC_TIMER1);						// Initialize TIMER0
	Chip_TIMER_PrescaleSet(LPC_TIMER1,PrescaleValue1);  // Set prescale value
	Chip_TIMER_SetMatch(LPC_TIMER1,0,5);				// Set match value
	Chip_TIMER_MatchEnableInt(LPC_TIMER1, 0);			// Configure to trigger interrupt on match

	// Enable timer interrupt
	NVIC_ClearPendingIRQ(TIMER1_IRQn);
	NVIC_EnableIRQ(TIMER1_IRQn);

	// Enable timer and RTC
	Chip_TIMER_Enable(LPC_TIMER1);
}

bool MotionDone = false;
bool GestureBegin = false;

void TIMER2_IRQHandler(void)
{
	if (Chip_TIMER_MatchPending(LPC_TIMER2, 0)) {
		Chip_TIMER_Reset(LPC_TIMER2);	// Reset TIMER0
		GestureBegin = false;
		MotionDone = true;
		Chip_TIMER_ClearMatch(LPC_TIMER2,0);  // Clear TIMER0 interrupt
	}
}
void Timer2_setup(){
	// Initialize TIMER2
	Chip_TIMER_Init(LPC_TIMER2);						// Initialize TIMER0
	Chip_TIMER_PrescaleSet(LPC_TIMER2,120000);  // Set prescale value
	Chip_TIMER_SetMatch(LPC_TIMER2,0,1000);				// Set match value
	Chip_TIMER_MatchEnableInt(LPC_TIMER2, 0);			// Configure to trigger interrupt on match

	// Enable timer interrupt
	NVIC_ClearPendingIRQ(TIMER2_IRQn);
	NVIC_EnableIRQ(TIMER2_IRQn);
}

int16_t ax, ay, az, gx, gy, gz;
float ag[6];

/*Initial UART setup*/
void UART_setup(){
	uint8_t key;
	int bytes;

	/* Setup UART for 115.2K8N1 */
	Chip_UART_Init(UART_SELECTION);
	Chip_UART_SetBaud(UART_SELECTION, 115200);
	Chip_UART_ConfigData(UART_SELECTION, (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT));
	Chip_UART_SetupFIFOS(UART_SELECTION, (UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2));
	Chip_UART_TXEnable(UART_SELECTION);

	/* Before using the ring buffers, initialize them using the ring
	   buffer init function */
	RingBuffer_Init(&rxring, rxbuff, 1, UART_RRB_SIZE);
	RingBuffer_Init(&txring, txbuff, 1, UART_SRB_SIZE);

	/* Reset and enable FIFOs, FIFO trigger level 3 (14 chars) */
	Chip_UART_SetupFIFOS(UART_SELECTION, (UART_FCR_FIFO_EN | UART_FCR_RX_RS |
							UART_FCR_TX_RS | UART_FCR_TRG_LEV3));

	/* Enable receive data and line status interrupt */
	Chip_UART_IntEnable(UART_SELECTION, (UART_IER_RBRINT | UART_IER_RLSINT));

	/* preemption = 1, sub-priority = 1 */
	NVIC_SetPriority(IRQ_SELECTION, 1);
	NVIC_EnableIRQ(IRQ_SELECTION);

	/* Send initial messages */
	Chip_UART_SendRB(UART_SELECTION, &txring, inst1, sizeof(inst1) - 1);
	Chip_UART_SendRB(UART_SELECTION, &txring, inst2, sizeof(inst2) - 1);
}

//SSP - SPI Initialization
void Spi_setup(){
	Board_SSP_Init(LPC_SSP);
	Chip_SSP_Init(LPC_SSP);

    TxBuf_Init();
    ssp_format.frameFormat = SSP_FRAMEFORMAT_SPI;
    ssp_format.bits = SSP_DATA_BITS;
    ssp_format.clockMode = SSP_CLOCK_MODE3;
    Chip_SSP_SetFormat(LPC_SSP, ssp_format.bits, ssp_format.frameFormat, ssp_format.clockMode);
    Chip_SSP_SetBitRate(LPC_SSP, 100000);
    Chip_SSP_Enable(LPC_SSP);
    Chip_SSP_SetMaster(LPC_SSP, 1);
}

//To display live position of LED based on hand acceleration
bool RealTime = false;

int main(void)
{
	SystemCoreClockUpdate();
	Board_Init();

	Spi_setup();
	i2c_app_init(I2C0, SPEED_100KHZ);
	i2c_set_mode(I2C0, 0);
	i2cDev = I2C0;
	Configure_GPIO_Interrupt();

	MPU6050_Init();
	Board_UART_Init(UART_SELECTION);
	Timer0_setup();
	Timer1_setup();   //Update the readings every 20ms
	Timer2_setup();
	UART_setup();
	for (int i = 0; i < 10000; i++); //delay necessary for UART

	float a_x, a_y, a_z, g_x, g_y, g_z;

	TxBuf_Init();
	ShowPixels();

	FusionAhrs fusionAhrs;
    FusionAhrsInitialise(&fusionAhrs, 0.5f, 20.0f, 70.0f); // valid magnetic field defined as 20 uT to 70 uT
    FusionVector3 earthAcceleration;

    int x_crest, y_crest, x_trough, y_trough, count = 20, indicator;
    int XmaxInd, YmaxInd, XminInd, YminInd;
    float ax_max, ax_min, ay_max, ay_min, axMaxC, ayMaxC, axMinC, ayMinC;
    float thHigh = 0.4, thLow = -0.4;
    float ax_prev, ax_current, ax_next, ay_prev, ay_current, ay_next, temp, temp2, averaged_Ax, averaged_Ay;

	int AyMaxStableTime, AyMinStableTime, AxMaxStableTime, AxMinStableTime;
	int FirstAxMaxTime = 0;
	int FirstAxMinTime = 0;
	int FirstAyMaxTime = 0;
	int FirstAyMinTime = 0;
	float MagAx,MagAy;
	int time=0;
	float GlobalAxMax,GlobalAxMin,GlobalAyMax,GlobalAyMin;

	float y_velocity = 0, y_velocity_count = 0;

    while (1){
    	FirstAxMaxTime = 0;
    	FirstAxMinTime = 0;
    	FirstAyMaxTime = 0;
    	FirstAyMinTime = 0;
    	indicator = 0;
    	x_crest = 0; y_crest = 0; x_trough = 0; y_trough = 0;
    	XmaxInd = 1, YmaxInd = 1; XminInd = 1; YminInd = 1;
    	ax_max = 0; ax_min = 0;
    	axMaxC = 0; axMinC = 0;
    	ay_max = 0; ay_min = 0;
    	ayMaxC = 0; ayMinC = 0;
        ax_prev = 0; ax_current = 0; ay_prev = 0; ay_current = 0;
        ax_next = 0; ay_next = 0; temp = 0; temp2 = 0; averaged_Ax = 0; averaged_Ay = 0;
        GlobalAxMax = ax_max, GlobalAxMin = ax_min, GlobalAyMax = ay_max, GlobalAyMin = ay_min;
        time = 0;

        while (1) {

        	if(SamplingFlag == 1){

        		MPU6050_GetMotionData(&ax, &ay, &az, &gx, &gy, &gz);

        		a_x = ax/16384.f;
        		a_y = ay/16384.f;
        		a_z = az/16384.f;
        		g_x = gx/131.072f;
        		g_y = gy/131.072f;
        		g_z = gz/131.072f;
        		FusionVector3 gyroscope = {
        			.axis.x = g_x,
					.axis.y = g_y,
					.axis.z = g_z,
				}; // literal values should be replaced with sensor measurements

				FusionVector3 accelerometer = {
					 .axis.x = a_x,
					 .axis.y = a_y,
					 .axis.z = a_z,
				}; // literal values should be replaced with sensor measurements

				FusionAhrsUpdate(&fusionAhrs, gyroscope, accelerometer, FUSION_VECTOR3_ZERO, 0.01f); //update with new readings

				earthAcceleration = FusionAhrsCalculateEarthAcceleration(&fusionAhrs);

				//Smoothen X data
				temp = ax_next;
				temp2 = ax_current;
				ax_next = earthAcceleration.axis.x;
				ax_current = temp;
				ax_prev = temp2;
				averaged_Ax = (ax_next+ax_current+ax_prev)/3;

				//Smoothen Y data
				temp = ay_next;
				temp2 = ay_current;
				ay_next = earthAcceleration.axis.y;
				ay_current = temp;
				ay_prev = temp2;
				averaged_Ay = (ay_next+ay_current+ay_prev)/3;

				/*temp = ay_next;
				temp2 = ay_current;
				ay_next = earthAcceleration.axis.y;
				ay_current = temp;
				ay_prev = temp2;
				averaged_Ay = (ay_next+ay_current+ay_prev)/3;
*/
				char AX[8], AY[8], AZ[8];
				snprintf(AX, sizeof(AX), "%f", averaged_Ax);
				snprintf(AY, sizeof(AY), "%f", averaged_Ay);
				snprintf(AZ, sizeof(AZ), "%f", earthAcceleration.axis.z);

				//Chip_UART_SendRB(UART_SELECTION, &txring, AX, sizeof(AX));
				//Chip_UART_SendRB(UART_SELECTION, &txring, "   ", sizeof("   ")-1);	//add space
				Chip_UART_SendRB(UART_SELECTION, &txring, AY, sizeof(AY));
				Chip_UART_SendRB(UART_SELECTION, &txring, "   ", sizeof("   ")-1);	//add space
				Chip_UART_SendRB(UART_SELECTION, &txring, AZ, sizeof(AZ));
				Chip_UART_SendRB(UART_SELECTION, &txring, "\n", sizeof("\n")-1);	//newline

				FusionAhrsZeroYaw(&fusionAhrs);

				//Update an xmax
				if(earthAcceleration.axis.z > ax_max) {
						ax_max = earthAcceleration.axis.z;
						XmaxInd = 1;
						AxMaxStableTime=0;
				}
				if(earthAcceleration.axis.z < ax_max && XmaxInd == 1){
					AxMaxStableTime++;
					if( AxMaxStableTime > count ){
						if(ax_max > thHigh){
							x_crest++;
							if(ax_max > GlobalAxMax)
								GlobalAxMax = ax_max;
							if(x_crest==1) {
								FirstAxMaxTime = time;
								if (!GestureBegin && RealTime) {
									Chip_TIMER_Enable(LPC_TIMER2);
									GestureBegin = true;
								}
							}
						}
						ax_max = 0;
						XmaxInd = 0;
					}
				}

				//Update a xmin
				if(earthAcceleration.axis.z < ax_min) {
						ax_min = earthAcceleration.axis.z;
						XminInd = 1;
						AxMinStableTime=0;
				}
				if(earthAcceleration.axis.z > ax_min && XminInd == 1){
					AxMinStableTime++;
					if( AxMinStableTime > count ){
						if(ax_min < thLow){
							x_trough++;
							if(ax_min < GlobalAxMin)
								GlobalAxMin = ax_min;
							if(x_trough==1) {
								FirstAxMinTime = time;
								if (!GestureBegin && RealTime) {
									Chip_TIMER_Enable(LPC_TIMER2);
									GestureBegin = true;
								}
							}
						}
						ax_min = 0;
						XminInd = 0;
					}
				}

				//Update an ymax/     Find local maximum, eliminating noise;
				if(averaged_Ay > ay_max) {
						ay_max = averaged_Ay;
						YmaxInd = 1;
						AyMaxStableTime=0;
				}
				if(averaged_Ay < ay_max && YmaxInd == 1){    //It start falling after we have a potential local maximum
					AyMaxStableTime++;					     //Stabletime++;
					if( AyMaxStableTime > count ){			 //We get 10 consecutive point less than potential maximum
						if(ay_max > thHigh){				 //Check if the local max is greater than threshold
							y_crest++;					     // crest++
							if(ay_max > GlobalAyMax)
								GlobalAyMax = ay_max;
							if(y_crest==1){						 //recording the first time when we get y_crest;
								FirstAyMaxTime = time;
								if (!GestureBegin && RealTime) {
									Chip_TIMER_Enable(LPC_TIMER2);
									GestureBegin = true;
								}
							}
						}
						ay_max = 0;							 //local max found reset local max
						YmaxInd = 0;						 //reset locla
					}
				}

				//Update a ymin
				if(averaged_Ay < ay_min) {
						ay_min = averaged_Ay;
						YminInd = 1;
						AyMinStableTime=0;
				}
				if(averaged_Ay > ay_min && YminInd == 1){
					AyMinStableTime++;
					if( AyMinStableTime > count ){
						if(ay_min < thLow){
							y_trough++;
							if(ay_min < GlobalAyMin)
								GlobalAyMin = ay_min;
							if(y_trough==1) {
								FirstAyMinTime = time;
								if (!GestureBegin && RealTime) {
									Chip_TIMER_Enable(LPC_TIMER2);
									GestureBegin = true;
								}
							}
						}
						ay_min = 0;
						YminInd = 0;
					}
				}
				time++;
				SamplingFlag = 0;
				indicator = 1;
				//FusionAhrsReinitialise(&fusionAhrs);
			}
            else if (takeInput == 0 && indicator == 1 && !RealTime) break;
        	else if (MotionDone && RealTime) break;
        }

        //Done with one gesture, determine gesture!
        if(x_crest < 1 && y_crest <1){
        	//printf("no motion \n");
        }
        else if( x_crest>1 || y_crest >1 ){
        	//printf("circular \n");
        	if(FirstAyMaxTime > FirstAyMinTime) {
        		printf("ClockWise\n");
        		if (!RealTime) CW();
        	}
        	else {
        		printf("CounterClockWise\n");
        		if (!RealTime) CCW();
        	}
        }
        else{
        	//printf("linear \n");
        	MagAx = fabs( GlobalAxMax - GlobalAxMin );
        	MagAy = fabs( GlobalAyMax - GlobalAyMin );
        	//printf("MagAx is %f   MagAy is %f\n", MagAx,MagAy );
        	if(MagAx > MagAy){
        		//printf("First Max Time is %d \n",FirstAxMaxTime);
        		//printf("First Min Time is %d \n",FirstAxMinTime);
        		if(FirstAxMaxTime > FirstAxMinTime) {
        			printf("Down\n");
        			if (!RealTime) LEDFountainOut();
        		}
        		else {
        			printf("Up\n");
        			if (!RealTime) LEDFountainIn();
        		}
        	}
        	else{
        		//printf("First Max Time is %d \n",FirstAyMaxTime);
        		//printf("First Min Time is %d \n",FirstAyMinTime);
        		if(FirstAyMaxTime > FirstAyMinTime) {
        		    printf("right\n");
        		    if (!RealTime) LEDRight();
        		}
        		else {
        		    printf("left\n");
        		    if (!RealTime) LEDLeft();
        		}
        	}
        }
        MotionDone = false;
        /*printf("The X crest count is %d\n", x_crest);
        printf("The X trough count is %d\n", x_trough);
        printf("The Y crest count is %d\n", y_crest);
        printf("The Y trough count is %d\n", y_trough);*/
    }

	// DeInitialize UART0 peripheral
	NVIC_DisableIRQ(IRQ_SELECTION);
	Chip_UART_DeInit(UART_SELECTION);

	Chip_I2C_DeInit(I2C0);
	Chip_SSP_DeInit(LPC_SSP);

	printf("END PROGRAM\n");
    Chip_SSP_DeInit(LPC_SSP);
	return 0;
}

