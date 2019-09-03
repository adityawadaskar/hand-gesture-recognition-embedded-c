/*
 * LEDStrip.h
 *
 *  Created on: Mar 21, 2018
 *      Author: dali
 */

#ifndef LEDSTRIP_H_
#define LEDSTRIP_H_

#include "board.h"
#include "stdio.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

// FIXME: SSP0 not working on LPCXpresso LPC1769.  There seems to be some sort
//        of contention on the MISO signal.  The contention originates on the
//          LPCXpresso side of the signal.
//
#define LPC_SSP           LPC_SSP2
#define SSP_IRQ           SSP1_IRQn
#define LPC_GPDMA_SSP_TX  GPDMA_CONN_SSP1_Tx
#define LPC_GPDMA_SSP_RX  GPDMA_CONN_SSP1_Rx
#define SSPIRQHANDLER     SSP1_IRQHandler

#define BUFFER_SIZE                         128//(0x100)
#define SSP_DATA_BITS                       (SSP_BITS_8)
#define SSP_DATA_BIT_NUM(databits)          (databits + 1)
#define SSP_DATA_BYTES(databits)            (((databits) > SSP_BITS_8) ? 2 : 1)
#define SSP_LO_BYTE_MSK(databits)           ((SSP_DATA_BYTES(databits) > 1) ? 0xFF : (0xFF >> \
                                                                                      (8 - SSP_DATA_BIT_NUM(databits))))
#define SSP_HI_BYTE_MSK(databits)           ((SSP_DATA_BYTES(databits) > 1) ? (0xFF >> \
                                                                               (16 - SSP_DATA_BIT_NUM(databits))) : 0)
bool takeInput = false;
char color = 'r';
int r = 255, g = 255, b = 255;

#define SSP_MODE_SEL                        (0x31)
#define SSP_TRANSFER_MODE_SEL               (0x32)
#define SSP_MASTER_MODE_SEL                 (0x31)
#define SSP_SLAVE_MODE_SEL                  (0x32)
#define SSP_POLLING_SEL                     (0x31)
#define SSP_INTERRUPT_SEL                   (0x32)

/* Tx buffer */
static uint8_t Tx_Buf[BUFFER_SIZE];

/* Rx buffer */
static uint8_t Rx_Buf[BUFFER_SIZE];

static SSP_ConfigFormat ssp_format;
static Chip_SSP_DATA_SETUP_T xf_setup;

 /* defined(DEBUG_ENABLE) */

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

//Set Color of Specific LED
void SetPixelColor(int pixel, int r, int g, int b) {
	int index = 4+(pixel*4);
	Tx_Buf[index+1] = b;
	Tx_Buf[index+2] = g;
	Tx_Buf[index+3] = r;
}

//Set Brightness of Specific LED
void SetPixelBrightness(int pixel, int brightness) {
	Tx_Buf[4+(pixel*4)] = 0xE0 + brightness;
}

//Output color pattern
void ShowPixels() {
	xf_setup.length = BUFFER_SIZE;
    xf_setup.tx_data = Tx_Buf;
    xf_setup.rx_cnt = xf_setup.tx_cnt = 0;
    Chip_SSP_RWFrames_Blocking(LPC_SSP, &xf_setup);
}

//Delay - used for LED patterns
void delay_ms (uint16_t ms) {
	uint16_t delay;
	volatile uint32_t i;
	for (delay = ms; delay >0 ; delay--)
		for (i = 20; i > 0; i--);
}

//Make LED bounce across strip
void LEDBounce() {
	TxBuf_Init();
	LEDRight();
	LEDLeft();
}

//LED traverse right
void LEDRight() {
	while (!takeInput) {
		for (int i = 0; i < 30; i++) {
			SetPixelColor(i, r, g, b);
			SetPixelBrightness(i, 255);
			ShowPixels();
			delay_ms(1);
			SetPixelColor(i, 0, 0, 0);
			ShowPixels();
			delay_ms(1);
		}
	}
}

//LED traverse left
void LEDLeft() {
	while (!takeInput) {
		for (int i = 29; i > 0; i--) {
			SetPixelColor(i, r, g, b);
			SetPixelBrightness(i, 255);
			ShowPixels();
			delay_ms(1);
			SetPixelColor(i, 0, 0, 0);
			ShowPixels();
			delay_ms(1);
		}
	}
}

void LEDFountainIn() {
	while (!takeInput) {
		for (int i = 0; i < 15; i++) {
			SetPixelColor(i, r, g, b);
			SetPixelBrightness(i, 255);
			SetPixelColor(29-i, r, g, b);
			SetPixelBrightness(29-i, 255);
			ShowPixels();
			delay_ms(3);
			SetPixelColor(i, 0, 0, 0);
			SetPixelColor(29-i, 0, 0, 0);
			ShowPixels();
			delay_ms(3);
		}
	}
}

void LEDFountainOut() {
	int count = 0;
	while (!takeInput) {
		for (int i = 14; i >= 0; i--) {
			SetPixelColor(i, r, g, b);
			SetPixelBrightness(i, 255);
			SetPixelColor(30-i, r, g, b);
			SetPixelBrightness(30-i, 255);
			ShowPixels();
			delay_ms(3);
			SetPixelColor(i, 0, 0, 0);
			SetPixelColor(30-i, 0, 0, 0);
			ShowPixels();
			delay_ms(3);
		}
	}
}

void CCW() {
	if (color == 'r') {
		r = 0; g = 255; b = 0;
		color = 'g';
	}
	else if (color == 'g') {
		r = 0; g = 0; b = 255;
		color = 'b';
	}
	else {
		r = 255; g = 0; b = 0;
		color = 'r';
	}
}

void CW() {
	if (color == 'r') {
		r = 0; g = 0; b = 255;
		color = 'b';
	}
	else if (color == 'g') {
		r = 255; g = 0; b = 0;
		color = 'r';
	}
	else {
		r = 0; g = 255; b = 0;
		color = 'g';
	}
}

void TxBuf_Init() {
	for (int i = 0; i < 128; i++)	//Initialize to 0
		Tx_Buf[i] = 0;
	for (int i = 0; i < 30; i++)	//LED frame ID bits
		Tx_Buf[(4*i)+4] = 0xE0;
	for (int i = 124; i < 128; i++)	//end frame
		Tx_Buf[i] = 255;
}


#endif /* LEDSTRIP_H_ */
