#include "SciSetup.h"

void FindFrameHeader(void);

int ScicXmtBufIndex = 0;
int ScicXmtBufNum = 0;
Uint16 ScicXmtBuf[SCIC_XMT_BUF_SIZE];

int ScicRcvBufIndex = 0;
int ScicRcvBufNum = 0;
Uint16 ScicRcvBuf[SCIC_RCV_BUF_SIZE];

#pragma DATA_SECTION(COM_REG,"com_reg");
Uint16 COM_REG[1024] = {0};

// Test 1,SCIA  DLB, 8-bit word, baud rate 0x000F, default, 1 STOP bit, no parity
void ScicInit()
{
	// Note: Clocks were turned on to the SCIA peripheral
	// in the InitSysCtrl() function
	
	ScicRegs.SCICCR.all = 0x0007;   // 1 stop bit,  No loopback
	       // No parity,8 char bits,
	       // async mode, idle-line protocol
	ScicRegs.SCICTL1.all = 0x0003;  // enable TX, RX, internal SCICLK,
	       // Disable RX ERR, SLEEP, TXWAKE
	ScicRegs.SCICTL2.all =  0x0003;
	ScicRegs.SCICTL2.bit.TXINTENA = 1;			// ���÷����жϣ��ȴ���Ҫ��������ʱ���������ж�
	ScicRegs.SCICTL2.bit.RXBKINTENA = 1;
	#if (CPU_FRQ_150MHZ)
		//ScicRegs.SCIHBAUD = 0x0000;  
		//ScicRegs.SCILBAUD = 0x0079;		// 38400 baud @LSPCLK = 37.5MHz
		ScicRegs.SCIHBAUD = 0x0000;
		ScicRegs.SCILBAUD = 0x0027; 	// 115200 baud @LSPCLK = 37.5MHz.
	#endif
	#if (CPU_FRQ_100MHZ)
		ScicRegs.SCIHBAUD = 0x0000;  // 115200 baud @LSPCLK = 20MHz.
		ScicRegs.SCILBAUD = 0x0014;
	#endif
		ScicRegs.SCICTL1.all = 0x0023;  // Relinquish SCI from Reset
}

// Transmit a character from the SCI
void scic_xmit(Uint16 a)
{
	ScicXmtBuf[ScicXmtBufIndex] = a;
	ScicXmtBufNum++;
	ScicXmtStart();
}

// �����ֽ�����
void ScicXmt(Uint16 *buf, int length)
{
	int i = 0;
	Uint16 index = ScicXmtBufIndex;
	length = length > SCIC_XMT_BUF_SIZE ? SCIC_XMT_BUF_SIZE : length;
	for(; i < length; i++)
	{
		if(index == SCIC_XMT_BUF_SIZE)	index = 0;
		ScicXmtBuf[index++] = buf[i];
		ScicXmtBufNum++;
	}
	ScicXmtStart();	
}

// �����������256�ֽ�
void scic_msg(char * msg)
{
	int i = 0;
	Uint16 index = ScicXmtBufIndex;
	while(msg[i] != '\0')
	{
		if(index == SCIC_XMT_BUF_SIZE)	index = 0;
		ScicXmtBuf[index++] = msg[i++];
		ScicXmtBufNum++;
		if(i > SCIC_XMT_BUF_SIZE)	break;
	}
	ScicXmtStart();
}

float RcvMsg2DutyCycle(Uint16 *msg)
{
	int i = 0;
	int isBeginConvert = 0;
	int count = 0;
	float res = 0.0;
	float base = 0.1;
	for(; i < 10; i++)
	{
		if(msg[i] == 'e' || count >= 4)
			break;
		if(isBeginConvert == 1)
		{
			res += base * (msg[i] - 0x30);
			base *= 0.1;
			count++;
		}
		else if(msg[i] == '.')
		{
			isBeginConvert = 1;
		}
	}
	return res;
}
// Initalize the SCI FIFO
void ScicFifoInit()
{
	int i;
	for(i = 0; i < SCIC_RCV_BUF_SIZE; i++)
	{
		ScicRcvBuf[i] = 0;
		ScicXmtBuf[i] = 0;
	}

	// ����FIFO�Ĵ���������FIFO����16�ֽڣ�С�ڵ���0���ֽڴ��������ж�
	ScicRegs.SCIFFTX.all = 0x5060;
	// ����FIFO�Ĵ���������FIFO����16�ֽڣ����ڵ���1���ֽڴ��������ж�
    ScicRegs.SCIFFRX.all = 0x50E1;
    ScicRegs.SCIFFCT.all = 0x0;
	
	ScicRegs.SCIFFTX.bit.TXFFIENA = 0;
	ScicRegs.SCIFFTX.bit.SCIRST = 1;
	ScicRegs.SCIFFTX.bit.TXFIFOXRESET = 1;
	ScicRegs.SCIFFRX.bit.RXFIFORESET = 1;

	EALLOW;  // This is needed to write to EALLOW protected registers
	PieVectTable.SCIRXINTC = &ISRScicRx;
	PieVectTable.SCITXINTC = &ISRScicTx;
	EDIS;    // This is needed to disable write to EALLOW protected registers

    // SCIRXINTC ��Ӧ INT8.5
	// SCITXINTC ��Ӧ INT8.6
    IER |= M_INT8;
	PieCtrlRegs.PIEIER8.bit.INTx5 = 1;
	PieCtrlRegs.PIEIER8.bit.INTx6 = 1;

}

interrupt void ISRScicRx(void)
{
	if(ScicRegs.SCIFFRX.bit.RXFFOVF == 1)
	{
		// ����FIFO���

		ScicRegs.SCIFFRX.bit.RXFFOVRCLR = 1;
	}
	else if(ScicRegs.SCIFFRX.bit.RXFFINT == 1)
	{
		// ����FIFO���ڵ��ڸ���ֵ
		int16 i;
		Uint16 num = ScicRegs.SCIFFRX.bit.RXFFST;
		for(i = 0; i < num; i++)
		{
			if(ScicRcvBufIndex == SCIC_RCV_BUF_SIZE)
			{
				ScicRcvBufIndex = 0;
			}
			ScicRcvBuf[ScicRcvBufIndex++] = ScicRegs.SCIRXBUF.all;
		}
		ScicRcvBufNum += num;
		FindFrameHeader();
		ScicRegs.SCIFFRX.bit.RXFFINTCLR = 1;
	}
   // Acknowledge this interrupt to receive more interrupts from group 8
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;
}

interrupt void ISRScicTx(void)
{
	if(ScicRegs.SCIFFTX.bit.TXFFINT == 1)
	{
		// ����FIFOС�ڵ��ڸ���ֵ
		int16 i;
		Uint16 num = 16 - ScicRegs.SCIFFTX.bit.TXFFST;
		if(num > ScicXmtBufNum)	// ���͵�����ΪFIFOʣ��������Ҫ�������ݵ���Сֵ
		{
			num = ScicXmtBufNum;
		}
		for(i = 0; i < num; i++)
		{
			if(ScicXmtBufIndex == SCIC_XMT_BUF_SIZE)
			{
				ScicXmtBufIndex = 0;
			}
			ScicRegs.SCITXBUF = ScicXmtBuf[ScicXmtBufIndex++];
		}
		ScicXmtBufNum -= num;
		if(ScicXmtBufNum == 0)
		{
			ScicRegs.SCIFFTX.bit.TXFFIENA = 0;	
		}
		ScicRegs.SCIFFTX.bit.TXFFINTCLR = 1;
	}
   // Acknowledge this interrupt to receive more interrupts from group 8
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;	
}

void ScicXmtStart(void)
{
	ScicRegs.SCIFFTX.bit.TXFFIENA = 1;
}

// ��Ѱ֡ͷ
void FindFrameHeader(void)
{
	int i, j, k;
	if(ScicRcvBufNum < 8)
	{
		return ;
	}
	else
	{
		k = (ScicRcvBufIndex - ScicRcvBufNum) & 0x00FF;
		for(i = 0; i < ScicRcvBufNum; i++)
		{
			// ���֡ͷ
			if((ScicRcvBuf[(k + i) & 0x00FF] == 0xFF) &&
				(ScicRcvBuf[(k + i + 1) & 0x00FF] == 0xFA))
			{
				int16 len = ScicRcvBuf[(k + i + 2) & 0x00FF];

				if(len > ScicRcvBufNum - 3)	// ���������㣬�ȴ��´δ���
					return;	
				else						// �������㹻����ʼ��������
				{	
					Uint16 checksum = 0;
					k += 3;
					// ���У��λ
					for(j = 0; j < len - 1; j++)
					{
						checksum += ScicRcvBuf[(k + i + j) & 0x00FF];
					}
					if((checksum & 0x00FF) != ScicRcvBuf[(k + i + j) & 0x00FF])
					{
						//У�����ɾ����֡
						ScicRcvBufNum -= (len + 3);
						return;
					}
					else
					{
						for(j = 0; j < len - 1; j = j + 4)
						{
							Uint16 addr = (ScicRcvBuf[(k + i + j) & 0x00FF] << 8) |
							 				ScicRcvBuf[(k + i + j + 1) & 0x00FF];
							Uint16 data = (ScicRcvBuf[(k + i + j + 2) & 0x00FF] << 8) |
							 				ScicRcvBuf[(k + i + j + 3) & 0x00FF];
							if(addr > 0x400)
								return;
							else
								COM_REG[addr] = data;
						}
						ScicRcvBufNum -= (len + 3);
						return;
					}
				}
			}
		}	
	} 

}

//===========================================================================
// No more.
//===========================================================================



