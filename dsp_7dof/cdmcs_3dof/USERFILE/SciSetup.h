#ifndef SCI_SETUP_H
#define SCI_SETUP_H

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File

#define SCIC_XMT_BUF_SIZE 256
#define SCIC_RCV_BUF_SIZE 256
/*
typedef struct RING_BUF
{
	uint16 buf[SCIC_XMT_BUF_SIZE];
	int16 index;
	int16 size;
}scic_buf;

scic_buf scic_xmt_buf;
scic_buf scic_rcv_buf;
*/
void ScicInit(void);
void ScicFifoInit(void);
void scic_xmit(Uint16 a);
void ScicXmt(Uint16 *buf, int length);
void scic_msg(char *msg);
float RcvMsg2DutyCycle(Uint16 *msg);
void ScicXmtStart(void);

interrupt void ISRScicRx(void);
interrupt void ISRScicTx(void);

extern Uint16 COM_REG[1024];

#endif

//===========================================================================
// No more.
//===========================================================================




