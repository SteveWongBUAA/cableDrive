#include "Ecan.h"
#include "stdio.h"

#define missCountTolerated 200000
void InitMyEcan()
{


	// eCAN control registers require read/write access using 32-bits.  Thus we
	// will create a set of shadow registers for this example.  These shadow
	// registers will be used to make sure the access is 32-bits and not 16.
	   struct ECAN_REGS ECanbShadow;

	// Step 1. Initialize System Control:
	// PLL, WatchDog, enable Peripheral Clocks
	// This example function is found in the DSP2833x_SysCtrl.c file.
	//   InitSysCtrl();

	// Step 2. Initalize GPIO:
	// This example function is found in the DSP2833x_Gpio.c file and
	// illustrates how to set the GPIO to it's default state.
	// InitGpio();  // Skipped for this example

	// For this example, configure CAN pins using GPIO regs here
	// This function is found in DSP2833x_ECan.c
	   InitECanGpio();

	// Step 3. Clear all interrupts and initialize PIE vector table:
	// Disable CPU interrupts
	//   DINT;

	// Initialize PIE control registers to their default state.
	// The default state is all PIE interrupts disabled and flags
	// are cleared.
	// This function is found in the DSP2833x_PieCtrl.c file.
	//   InitPieCtrl();

	// Disable CPU interrupts and clear all CPU interrupt flags:
	//   IER = 0x0000;
	//   IFR = 0x0000;

	// Initialize the PIE vector table with pointers to the shell Interrupt
	// Service Routines (ISR).
	// This will populate the entire table, even if the interrupt
	// is not used in this example.  This is useful for debug purposes.
	// The shell ISR routines are found in DSP2833x_DefaultIsr.c.
	// This function is found in DSP2833x_PieVect.c.
	//   InitPieVectTable();

	// Step 4. Initialize all the Device Peripherals:
	// This function is found in DSP2833x_InitPeripherals.c
	// InitPeripherals(); // Not required for this example

	// Step 5. User specific code, enable interrupts:

	    //MessageReceivedCount = 0;
	    //ErrorCount = 0;
	    //PassCount = 0;

	    // eCAN control registers require 32-bit access.
	    // If you want to write to a single bit, the compiler may break this
	    // access into a 16-bit access.  One solution, that is presented here,
	    // is to use a shadow register to force the 32-bit access.

	    // Read the entire register into a shadow register.  This access
	    // will be 32-bits.  Change the desired bit and copy the value back
	    // to the eCAN register with a 32-bit write.

	    // Configure the eCAN RX and TX pins for eCAN transmissions
	    EALLOW;
	    ECanbShadow.CANTIOC.all = ECanbRegs.CANTIOC.all;
	    ECanbShadow.CANTIOC.bit.TXFUNC = 1;
	    ECanbRegs.CANTIOC.all = ECanbShadow.CANTIOC.all;

	    ECanbShadow.CANRIOC.all = ECanbRegs.CANRIOC.all;
	    ECanbShadow.CANRIOC.bit.RXFUNC = 1;
	    ECanbRegs.CANRIOC.all = ECanbShadow.CANRIOC.all;
	    EDIS;

	    // Disable all Mailboxes
	    // Since this write is to the entire register (instead of a bit
	    // field) a shadow register is not required.
	    ECanbRegs.CANME.all = 0;

	    // Mailboxs can be written to 16-bits or 32-bits at a time
	    // Write to the MSGID field of TRANSMIT mailboxes MBOX0 - 15
	    //新电路板与IMU的通讯需要实现为双向的，不仅要从DSP读到IMU的数据，还需要实现用DSP给IMU写命令，使欧拉角数据清零。
	    //在这种情况下不能继续使用原来的CANOpen协议，需要使用LpCAN协议。
	    //IMU通电时默认以CANOpen协议一直在向外发送数据，同时也在等待Goto_Command_Mode的命令。
	    //写命令的帧ID是514h+ID号（IMU说明书中没有写清楚），根据CAN邮箱的ID设置，ID为1的IMU的帧ID为515h，对应的邮箱ID应该设置为1454h。

	    ECanbMboxes.MBOX0.MSGID.all = 0x14540000;//514h+1，LpCAN协议,写命令的帧ID;
	    ECanbMboxes.MBOX1.MSGID.all = 0x14580000;//514h+2
	    ECanbMboxes.MBOX2.MSGID.all = 0x145C0000;//514h+3
	    ECanbMboxes.MBOX3.MSGID.all = 0x9555AAA3;
	    ECanbMboxes.MBOX4.MSGID.all = 0x9555AAA4;
	    ECanbMboxes.MBOX5.MSGID.all = 0x9555AAA5;
	    ECanbMboxes.MBOX6.MSGID.all = 0x9555AAA6;
	    ECanbMboxes.MBOX7.MSGID.all = 0x9555AAA7;
	    ECanbMboxes.MBOX8.MSGID.all = 0x9555AAA8;
	    ECanbMboxes.MBOX9.MSGID.all = 0x9555AAA9;
	    ECanbMboxes.MBOX10.MSGID.all = 0x9555AAAA;
	    ECanbMboxes.MBOX11.MSGID.all = 0x9555AAAB;
	    ECanbMboxes.MBOX12.MSGID.all = 0x9555AAAC;
	    ECanbMboxes.MBOX13.MSGID.all = 0x9555AAAD;
	    ECanbMboxes.MBOX14.MSGID.all = 0x9555AAAE;
	    ECanbMboxes.MBOX15.MSGID.all = 0x9555AAAF;

	    // Write to the MSGID field of RECEIVE mailboxes MBOX16 - 31

	    //接收邮箱设置为一样的ID，用作接收回复的数据包
	    ECanbMboxes.MBOX16.MSGID.all = 0x14540000;//514h+1，LpCAN协议,写命令的帧ID;
	    ECanbMboxes.MBOX17.MSGID.all = 0x14580000;//514h+2
	    ECanbMboxes.MBOX18.MSGID.all = 0x145C0000;//514h+3
	    ECanbMboxes.MBOX19.MSGID.all = 0x14530000;
	    ECanbMboxes.MBOX20.MSGID.all = 0x9555AAAA;
	    ECanbMboxes.MBOX21.MSGID.all = 0x9555AAAA;
	    ECanbMboxes.MBOX22.MSGID.all = 0x9555AAAA;
	    ECanbMboxes.MBOX23.MSGID.all = 0x9555AAAA;
	    ECanbMboxes.MBOX24.MSGID.all = 0x9555AAAA;
	    ECanbMboxes.MBOX25.MSGID.all = 0x9555AAA9;
	    ECanbMboxes.MBOX26.MSGID.all = 0x9555AAAA;
	    ECanbMboxes.MBOX27.MSGID.all = 0x9555AAAB;
	    ECanbMboxes.MBOX28.MSGID.all = 0x9555AAAC;
	    ECanbMboxes.MBOX29.MSGID.all = 0x9555AAAD;
	    ECanbMboxes.MBOX30.MSGID.all = 0x9555AAAE;
	    ECanbMboxes.MBOX31.MSGID.all = 0x9555AAAF;

	    // Configure Mailboxes 0-15 as Tx, 16-31 as Rx
	    // Since this write is to the entire register (instead of a bit
	    // field) a shadow register is not required.
	    ECanbRegs.CANMD.all = 0xFFFF0000;

	    // Enable all Mailboxes */
	    // Since this write is to the entire register (instead of a bit
	    // field) a shadow register is not required.
	    ECanbRegs.CANME.all = 0xFFFFFFFF;

	    // Specify that 8 bits will be sent/received
	    ECanbMboxes.MBOX0.MSGCTRL.bit.DLC = 8;
	    ECanbMboxes.MBOX1.MSGCTRL.bit.DLC = 8;
	    ECanbMboxes.MBOX2.MSGCTRL.bit.DLC = 8;
	    ECanbMboxes.MBOX3.MSGCTRL.bit.DLC = 8;
	    ECanbMboxes.MBOX4.MSGCTRL.bit.DLC = 8;
	    ECanbMboxes.MBOX5.MSGCTRL.bit.DLC = 8;
	    ECanbMboxes.MBOX6.MSGCTRL.bit.DLC = 8;
	    ECanbMboxes.MBOX7.MSGCTRL.bit.DLC = 8;
	    ECanbMboxes.MBOX8.MSGCTRL.bit.DLC = 8;
	    ECanbMboxes.MBOX9.MSGCTRL.bit.DLC = 8;
	    ECanbMboxes.MBOX10.MSGCTRL.bit.DLC = 8;
	    ECanbMboxes.MBOX11.MSGCTRL.bit.DLC = 8;
	    ECanbMboxes.MBOX12.MSGCTRL.bit.DLC = 8;
	    ECanbMboxes.MBOX13.MSGCTRL.bit.DLC = 8;
	    ECanbMboxes.MBOX14.MSGCTRL.bit.DLC = 8;
	    ECanbMboxes.MBOX15.MSGCTRL.bit.DLC = 8;

	    // No remote frame is requested
	    // Since RTR bit is undefined upon reset,
	    // it must be initialized to the proper value
	    ECanbMboxes.MBOX0.MSGCTRL.bit.RTR = 0;
	    ECanbMboxes.MBOX1.MSGCTRL.bit.RTR = 0;
	    ECanbMboxes.MBOX2.MSGCTRL.bit.RTR = 0;
	    ECanbMboxes.MBOX3.MSGCTRL.bit.RTR = 0;
	    ECanbMboxes.MBOX4.MSGCTRL.bit.RTR = 0;
	    ECanbMboxes.MBOX5.MSGCTRL.bit.RTR = 0;
	    ECanbMboxes.MBOX6.MSGCTRL.bit.RTR = 0;
	    ECanbMboxes.MBOX7.MSGCTRL.bit.RTR = 0;
	    ECanbMboxes.MBOX8.MSGCTRL.bit.RTR = 0;
	    ECanbMboxes.MBOX9.MSGCTRL.bit.RTR = 0;
	    ECanbMboxes.MBOX10.MSGCTRL.bit.RTR = 0;
	    ECanbMboxes.MBOX11.MSGCTRL.bit.RTR = 0;
	    ECanbMboxes.MBOX12.MSGCTRL.bit.RTR = 0;
	    ECanbMboxes.MBOX13.MSGCTRL.bit.RTR = 0;
	    ECanbMboxes.MBOX14.MSGCTRL.bit.RTR = 0;
	    ECanbMboxes.MBOX15.MSGCTRL.bit.RTR = 0;

	    // Write to the mailbox RAM field of MBOX0 - 15
	    ECanbMboxes.MBOX0.MDL.all = 0x9555AAA0;
	    ECanbMboxes.MBOX0.MDH.all = 0x89ABCDEF;

	    ECanbMboxes.MBOX1.MDL.all = 0x9555AAA1;
	    ECanbMboxes.MBOX1.MDH.all = 0x89ABCDEF;

	    ECanbMboxes.MBOX2.MDL.all = 0x9555AAA2;
	    ECanbMboxes.MBOX2.MDH.all = 0x89ABCDEF;

	    ECanbMboxes.MBOX3.MDL.all = 0x9555AAA3;
	    ECanbMboxes.MBOX3.MDH.all = 0x89ABCDEF;

	    ECanbMboxes.MBOX4.MDL.all = 0x9555AAA4;
	    ECanbMboxes.MBOX4.MDH.all = 0x89ABCDEF;

	    ECanbMboxes.MBOX5.MDL.all = 0x9555AAA5;
	    ECanbMboxes.MBOX5.MDH.all = 0x89ABCDEF;

	    ECanbMboxes.MBOX6.MDL.all = 0x9555AAA6;
	    ECanbMboxes.MBOX6.MDH.all = 0x89ABCDEF;

	    ECanbMboxes.MBOX7.MDL.all = 0x9555AAA7;
	    ECanbMboxes.MBOX7.MDH.all = 0x89ABCDEF;

	    ECanbMboxes.MBOX8.MDL.all = 0x9555AAA8;
	    ECanbMboxes.MBOX8.MDH.all = 0x89ABCDEF;

	    ECanbMboxes.MBOX9.MDL.all = 0x9555AAA9;
	    ECanbMboxes.MBOX9.MDH.all = 0x89ABCDEF;

	    ECanbMboxes.MBOX10.MDL.all = 0x9555AAAA;
	    ECanbMboxes.MBOX10.MDH.all = 0x89ABCDEF;

	    ECanbMboxes.MBOX11.MDL.all = 0x9555AAAB;
	    ECanbMboxes.MBOX11.MDH.all = 0x89ABCDEF;

	    ECanbMboxes.MBOX12.MDL.all = 0x9555AAAC;
	    ECanbMboxes.MBOX12.MDH.all = 0x89ABCDEF;

	    ECanbMboxes.MBOX13.MDL.all = 0x9555AAAD;
	    ECanbMboxes.MBOX13.MDH.all = 0x89ABCDEF;

	    ECanbMboxes.MBOX14.MDL.all = 0x9555AAAE;
	    ECanbMboxes.MBOX14.MDH.all = 0x89ABCDEF;

	    ECanbMboxes.MBOX15.MDL.all = 0x9555AAAF;
	    ECanbMboxes.MBOX15.MDH.all = 0x89ABCDEF;

	    //Received Box
	    ECanbMboxes.MBOX16.MDL.all = 0x00000000;
	    ECanbMboxes.MBOX16.MDH.all = 0x00000000;

	    ECanbMboxes.MBOX17.MDL.all = 0x00000000;
	    ECanbMboxes.MBOX17.MDH.all = 0x00000000;

	    ECanbMboxes.MBOX18.MDL.all = 0x00000000;
	    ECanbMboxes.MBOX18.MDH.all = 0x00000000;

	    ECanbMboxes.MBOX19.MDL.all = 0x00000000;
	    ECanbMboxes.MBOX19.MDH.all = 0x00000000;

	    ECanbMboxes.MBOX20.MDL.all = 0x00000000;
	    ECanbMboxes.MBOX20.MDH.all = 0x00000000;

	    ECanbMboxes.MBOX21.MDL.all = 0x00000000;
	    ECanbMboxes.MBOX21.MDH.all = 0x00000000;

	    ECanbMboxes.MBOX22.MDL.all = 0x00000000;
	    ECanbMboxes.MBOX22.MDH.all = 0x00000000;

	    ECanbMboxes.MBOX23.MDL.all = 0x00000000;
	    ECanbMboxes.MBOX23.MDH.all = 0x00000000;

	    ECanbMboxes.MBOX24.MDL.all = 0x00000000;
	    ECanbMboxes.MBOX24.MDH.all = 0x00000000;

	    ECanbMboxes.MBOX25.MDL.all = 0x00000000;
	    ECanbMboxes.MBOX25.MDH.all = 0x00000000;



	    // Since this write is to the entire register (instead of a bit
	    // field) a shadow register is not required.
	    EALLOW;
	    ECanbRegs.CANMIM.all = 0xFFFFFFFF;

	    // Request permission to change the configuration registers
	    ECanbShadow.CANMC.all = ECanbRegs.CANMC.all;
	    ECanbShadow.CANMC.bit.CCR = 1;
	    ECanbRegs.CANMC.all = ECanbShadow.CANMC.all;
	    EDIS;

	    // Wait until the CPU has been granted permission to change the
	    // configuration registers
	    // Wait for CCE bit to be set..
	    do
	    {
	      ECanbShadow.CANES.all = ECanbRegs.CANES.all;
	    } while(ECanbShadow.CANES.bit.CCE != 1 );

	    // Configure the eCAN timing
	    EALLOW;
	    ECanbShadow.CANBTC.all = ECanbRegs.CANBTC.all;

	    ECanbShadow.CANBTC.bit.BRPREG = 9;    // (BRPREG + 1) = 10 feeds a 15 MHz CAN clock
	    ECanbShadow.CANBTC.bit.TSEG2REG = 5 ; // to the CAN module. (150 / 10 = 15)
	    ECanbShadow.CANBTC.bit.TSEG1REG = 7;  // Bit time = 15
	    ECanbRegs.CANBTC.all = ECanbShadow.CANBTC.all;

	    ECanbShadow.CANMC.all = ECanbRegs.CANMC.all;
	    ECanbShadow.CANMC.bit.CCR = 0;
	    ECanbRegs.CANMC.all = ECanbShadow.CANMC.all;
	    EDIS;

	    // Wait until the CPU no longer has permission to change the
	    // configuration registers
	    do
	    {
	      ECanbShadow.CANES.all = ECanbRegs.CANES.all;
	    } while(ECanbShadow.CANES.bit.CCE != 0 );

	    // Configure the eCAN for self test mode
	    // Enable the enhanced features of the eCAN.
	    EALLOW;
	    ECanbShadow.CANMC.all = ECanbRegs.CANMC.all;
	    ECanbShadow.CANMC.bit.STM = 0;    // Configure CAN for self-test mode
	    ECanbShadow.CANMC.bit.SCB = 1;    // eCAN mode (reqd to access 32 mailboxes)
	    ECanbRegs.CANMC.all = ECanbShadow.CANMC.all;
	    EDIS;
}

//void mailbox_read(int16 MBXnbr)
//{
//   volatile struct MBOX *Mailbox;
//   Mailbox = &ECanbMboxes.MBOX0 + MBXnbr;
//   //TestMbox1 = Mailbox->MDL.all; // = 0x9555AAAn (n is the MBX number)
//   //TestMbox2 = Mailbox->MDH.all; // = 0x89ABCDEF (a constant)
//   //TestMbox3 = Mailbox->MSGID.all;// = 0x9555AAAn (n is the MBX number)
//   //MessageReceived[MBXnbr-16] = Mailbox->MDL.all;
//   Uint32 temp = InverseOrder32(Mailbox->MDL.all);
//   IMUdata[MBXnbr-16] = *(float32*)&temp;
//
//} // MSGID of a rcv MBX is transmitted as the MDL data.


Uint32 InverseOrder32(Uint32 a)
{
	Uint32 temp = 0X00000000;
	temp = temp | ((a & 0x000000FF)<<24);
	temp = temp | ((a & 0x0000FF00)<<8);
	temp = temp | ((a & 0x00FF0000)>>8);
	temp = temp | ((a & 0xFF000000)>>24);
	return temp;
}

void IMU_read_CANopen(float32 *IMUdata)
{
	volatile struct MBOX *Mailbox;
	Uint16  j;
    for(j=16; j<16+9; j++)         // Read & check 16 mailboxes
    {
    	Mailbox = &ECanbMboxes.MBOX0 + j;
    	Uint32 temp = InverseOrder32(Mailbox->MDL.all);
    	IMUdata[j-16] = *(float32*)&temp;

    }
}

Uint32 IMU1_GotoCmd()
{
	//GOTO COMMAND MODE
	//切换为LpCAN协议要先把IMU切换为Command Mode。
	//切换到Command Mode的Command No.是6
	//将ID为1的IMU设置为Command_Mode的数据包应该是3A01 0006 0000 0007 000D 0A00
	ECanbMboxes.MBOX0.MDL.all = 0x3A010006;
	ECanbMboxes.MBOX0.MDH.all = 0x00000007;

	ECanbRegs.CANTRS.all = 0x00000001;  // Set TRS
	while(ECanbRegs.CANTA.all != 0x00000001 ) {}  // Wait for all TAn bits to be set..
	ECanbRegs.CANTA.all = 0x00000001;   // Clear TAn

	ECanbMboxes.MBOX0.MDL.all = 0x000D0A00;
	ECanbMboxes.MBOX0.MDH.all = 0x00000000;

	ECanbRegs.CANTRS.all = 0x00000001;  // Set TRS
	while(ECanbRegs.CANTA.all != 0x00000001 ) {}  // Wait for all TAn bits to be set..
	ECanbRegs.CANTA.all = 0x00000001;   // Clear TAn

	volatile struct MBOX *Mailbox;

	Uint16 count1 = 0;
	Uint32 missCount = 0;
	Uint32 IMU1MSGL[2];
	//Uint32 IMU1MSGH[2];
	//读取IMU返回的数据的时候需要处理一整个数据包，这个数据包被拆分为若干个CAN message，必须写程序来控制消息的接收。
	//其中，如果某个邮箱n收到消息，则寄存器RMP的第n位被置为1。这些位只能被CPU重置为0，只能被内部逻辑（收到消息）设置为1。
	//要把寄存器RMP中的位设为0，需要通过把相应的位写为1（不是写为0）。
	//通过程序把相应的位写为0后，当又有新消息到达的时候，内部逻辑又会把相应的位写成1，如此反复接收连续的消息。

	//给IMU发送Goto_Command_Mode，IMU会返回两帧CAN message。
	//count1用于记录接收的message数量。万一IMU没返回CAN message,为了防止死循环，设置missCount，missCount超过容忍值则跳出循环。
	while(count1<2 && missCount<missCountTolerated)
	{
		if(ECanbRegs.CANRMP.bit.RMP16 == 1)
		{
			Mailbox = &ECanbMboxes.MBOX0 + 16;
			IMU1MSGL[count1] = Mailbox->MDL.all;
			//IMU1MSGH[count1] = Mailbox->MDH.all;
			ECanbRegs.CANRMP.bit.RMP16 = 1;//Reset RMP[n]
			count1 ++;
		}
		else
			missCount ++;
	}
	if(missCount >= missCountTolerated)
		return 0;
	//Analyze the data chunk
	if(IMU1MSGL[0] == 0X3A010000)//REPLY_ACK
	{
		return 1;
	}
	else//REPLY_NACK
		return 0;
}


Uint32 IMU2_GotoCmd()
{
	//GOTO COMMAND MODE
	ECanbMboxes.MBOX1.MDL.all = 0x3A020006;
	ECanbMboxes.MBOX1.MDH.all = 0x00000008;

	ECanbRegs.CANTRS.all = 0x00000002;  // Set TRS
	while(ECanbRegs.CANTA.all != 0x00000002 ) {}  // Wait for all TAn bits to be set..
	ECanbRegs.CANTA.all = 0x00000002;   // Clear TAn

	ECanbMboxes.MBOX1.MDL.all = 0x000D0A00;
	ECanbMboxes.MBOX1.MDH.all = 0x00000000;

	ECanbRegs.CANTRS.all = 0x00000002;  // Set TRS
	while(ECanbRegs.CANTA.all != 0x00000002 ) {}  // Wait for all TAn bits to be set..
	ECanbRegs.CANTA.all = 0x00000002;   // Clear TAn

	volatile struct MBOX *Mailbox;

	Uint16 count2 = 0;
	Uint32 missCount = 0;
	Uint32 IMU2MSGL[2];
	//Uint32 IMU2MSGH[2];

	while(count2<2 && missCount<missCountTolerated)
	{
		if(ECanbRegs.CANRMP.bit.RMP17 == 1)
		{
			Mailbox = &ECanbMboxes.MBOX0 + 17;
			IMU2MSGL[count2] = Mailbox->MDL.all;
			//IMU2MSGH[count2] = Mailbox->MDH.all;
			ECanbRegs.CANRMP.bit.RMP17 = 1;//Reset RMP[n]
			count2 ++;
		}
		else
			missCount++;
	}
	if(missCount >= missCountTolerated)
		return 0;
	//Analyze the data chunk
	if(IMU2MSGL[0] == 0X3A020000)//REPLY_ACK
	{
		return 1;
	}
	else//REPLY_NACK
		return 0;
}

Uint32 IMU3_GotoCmd()
{
	//GOTO COMMAND MODE
	ECanbMboxes.MBOX2.MDL.all = 0x3A030006;
	ECanbMboxes.MBOX2.MDH.all = 0x00000009;

	ECanbRegs.CANTRS.all = 0x00000004;  // Set TRS
	while(ECanbRegs.CANTA.all != 0x00000004 ) {}  // Wait for all TAn bits to be set..
	ECanbRegs.CANTA.all = 0x00000004;   // Clear TAn

	ECanbMboxes.MBOX2.MDL.all = 0x000D0A00;
	ECanbMboxes.MBOX2.MDH.all = 0x00000000;

	ECanbRegs.CANTRS.all = 0x00000004;  // Set TRS
	while(ECanbRegs.CANTA.all != 0x00000004 ) {}  // Wait for all TAn bits to be set..
	ECanbRegs.CANTA.all = 0x00000004;   // Clear TAn

	volatile struct MBOX *Mailbox;

	Uint16 count3 = 0;
	Uint32 missCount = 0;
	Uint32 IMU3MSGL[2];
	//Uint32 IMU3MSGH[2];

	while(count3<2 && missCount<missCountTolerated)
	{
		if(ECanbRegs.CANRMP.bit.RMP18 == 1)
		{
			Mailbox = &ECanbMboxes.MBOX0 + 18;
			IMU3MSGL[count3] = Mailbox->MDL.all;
			//IMU3MSGH[count3] = Mailbox->MDH.all;
			ECanbRegs.CANRMP.bit.RMP18 = 1;//Reset RMP[n]
			count3 ++;
		}
		else
			missCount++;
	}
	if(missCount >= missCountTolerated)
		return 0;
	//Analyze the data chunk
	if(IMU3MSGL[0] == 0X3A030000)//REPLY_ACK
	{
		return 1;
	}
	else//REPLY_NACK
		return 0;
}

Uint32 IMU1_SetOffset()
{
	//GOTO COMMAND MODE
	ECanbMboxes.MBOX0.MDL.all = 0x3A010012;
	ECanbMboxes.MBOX0.MDH.all = 0x00000013;

	ECanbRegs.CANTRS.all = 0x00000001;  // Set TRS
	while(ECanbRegs.CANTA.all != 0x00000001 ) {}  // Wait for all TAn bits to be set..
	ECanbRegs.CANTA.all = 0x00000001;   // Clear TAn

	ECanbMboxes.MBOX0.MDL.all = 0x000D0A00;
	ECanbMboxes.MBOX0.MDH.all = 0x00000000;

	ECanbRegs.CANTRS.all = 0x00000001;  // Set TRS
	while(ECanbRegs.CANTA.all != 0x00000001 ) {}  // Wait for all TAn bits to be set..
	ECanbRegs.CANTA.all = 0x00000001;   // Clear TAn

	volatile struct MBOX *Mailbox;

	Uint16 count1 = 0;
	Uint32 missCount = 0;
	Uint32 IMU1MSGL[2];
	//Uint32 IMU1MSGH[2];

	while(count1<2 && missCount<missCountTolerated)
	{
		if(ECanbRegs.CANRMP.bit.RMP16 == 1)
		{
			Mailbox = &ECanbMboxes.MBOX0 + 16;
			IMU1MSGL[count1] = Mailbox->MDL.all;
			//IMU1MSGH[count1] = Mailbox->MDH.all;
			ECanbRegs.CANRMP.bit.RMP16 = 1;//Reset RMP[n]
			count1 ++;
		}
		else
			missCount ++;
	}
	if(missCount >= missCountTolerated)
		return 0;
	//Analyze the data chunk
	if(IMU1MSGL[0] == 0X3A010000)//REPLY_ACK
	{
		return 1;
	}
	else//REPLY_NACK
		return 0;
}


Uint32 IMU2_SetOffset()
{
	//GOTO COMMAND MODE
	ECanbMboxes.MBOX1.MDL.all = 0x3A020012;
	ECanbMboxes.MBOX1.MDH.all = 0x00000014;

	ECanbRegs.CANTRS.all = 0x00000002;  // Set TRS
	while(ECanbRegs.CANTA.all != 0x00000002 ) {}  // Wait for all TAn bits to be set..
	ECanbRegs.CANTA.all = 0x00000002;   // Clear TAn

	ECanbMboxes.MBOX1.MDL.all = 0x000D0A00;
	ECanbMboxes.MBOX1.MDH.all = 0x00000000;

	ECanbRegs.CANTRS.all = 0x00000002;  // Set TRS
	while(ECanbRegs.CANTA.all != 0x00000002 ) {}  // Wait for all TAn bits to be set..
	ECanbRegs.CANTA.all = 0x00000002;   // Clear TAn

	volatile struct MBOX *Mailbox;

	Uint16 count2 = 0;
	Uint32 missCount = 0;
	Uint32 IMU2MSGL[2];
	//Uint32 IMU2MSGH[2];

	while(count2<2 && missCount<missCountTolerated)
	{
		if(ECanbRegs.CANRMP.bit.RMP17 == 1)
		{
			Mailbox = &ECanbMboxes.MBOX0 + 17;
			IMU2MSGL[count2] = Mailbox->MDL.all;
			//IMU2MSGH[count2] = Mailbox->MDH.all;
			ECanbRegs.CANRMP.bit.RMP17 = 1;//Reset RMP[n]
			count2 ++;
		}
		else
			missCount++;
	}
	if(missCount >= missCountTolerated)
		return 0;
	//Analyze the data chunk
	if(IMU2MSGL[0] == 0X3A020000)//REPLY_ACK
	{
		return 1;
	}
	else//REPLY_NACK
		return 0;
}

Uint32 IMU3_SetOffset()
{
	//Send Command
	ECanbMboxes.MBOX2.MDL.all = 0x3A030012;
	ECanbMboxes.MBOX2.MDH.all = 0x00000015;

	ECanbRegs.CANTRS.all = 0x00000004;  // Set TRS
	while(ECanbRegs.CANTA.all != 0x00000004 ) {}  // Wait for all TAn bits to be set..
	ECanbRegs.CANTA.all = 0x00000004;   // Clear TAn

	ECanbMboxes.MBOX2.MDL.all = 0x000D0A00;
	ECanbMboxes.MBOX2.MDH.all = 0x00000000;

	ECanbRegs.CANTRS.all = 0x00000004;  // Set TRS
	while(ECanbRegs.CANTA.all != 0x00000004 ) {}  // Wait for all TAn bits to be set..
	ECanbRegs.CANTA.all = 0x00000004;   // Clear TAn

	volatile struct MBOX *Mailbox;

	Uint16 count3 = 0;
	Uint32 missCount = 0;
	Uint32 IMU3MSGL[2];
	//Uint32 IMU3MSGH[2];

	while(count3<2 && missCount<missCountTolerated)
	{
		if(ECanbRegs.CANRMP.bit.RMP18 == 1)
		{
			Mailbox = &ECanbMboxes.MBOX0 + 18;
			IMU3MSGL[count3] = Mailbox->MDL.all;
			//IMU3MSGH[count3] = Mailbox->MDH.all;
			ECanbRegs.CANRMP.bit.RMP18 = 1;//Reset RMP[n]
			count3 ++;
		}
		else
			missCount++;
	}
	if(missCount >= missCountTolerated)
		return 0;
	//Analyze the data chunk
	if(IMU3MSGL[0] == 0X3A030000)//REPLY_ACK
	{
		return 1;
	}
	else//REPLY_NACK
		return 0;
}


Uint32 IMU1_read_LpCan(Uint32 *IMUhex,float32 *IMUdata)
{
	//GET EULER ANGLE
	ECanbMboxes.MBOX0.MDL.all = 0x3A010009;
	ECanbMboxes.MBOX0.MDH.all = 0x0000000A;

	ECanbRegs.CANTRS.all = 0x00000001;  // Set TRS
	while(ECanbRegs.CANTA.all != 0x00000001 ) {}  // Wait for all TAn bits to be set..
	ECanbRegs.CANTA.all = 0x00000001;   // Clear TAn

	ECanbMboxes.MBOX0.MDL.all = 0x000D0A00;
	ECanbMboxes.MBOX0.MDH.all = 0x00000000;

	ECanbRegs.CANTRS.all = 0x00000001;  // Set TRS
	while(ECanbRegs.CANTA.all != 0x00000001 ) {}  // Wait for all TAn bits to be set..
	ECanbRegs.CANTA.all = 0x00000001;   // Clear TAn

	volatile struct MBOX *Mailbox;

	Uint16 count1 = 0;
	Uint32 missCount = 0;
	Uint32 IMU1MSGL[4];
	Uint32 IMU1MSGH[4];

	while(count1<4 && missCount<missCountTolerated)//需要接收三帧数据，但也不能死循环在这。
	{
		if(ECanbRegs.CANRMP.bit.RMP16 == 1)//来了新的数据
		{
			Mailbox = &ECanbMboxes.MBOX0 + 16;
			IMU1MSGL[count1] = Mailbox->MDL.all;
			IMU1MSGH[count1] = Mailbox->MDH.all;
			ECanbRegs.CANRMP.bit.RMP16 = 1;//Reset RMP[n]，读走数据，清标志位，等待下一次的数据
			count1 ++;
		}
		else
		{
			missCount++;
		}
	}

	//Analyze the data chunk
	if(IMU1MSGL[0] == 0X3A010009)
	{
		Uint32 temp;
		temp = InverseOrder32(((IMU1MSGL[1] & 0X000000FF) << 24) | ((IMU1MSGH[1] & 0XFFFFFF00) >> 8));//Euler Angle X
		IMUhex[0] = ((IMU1MSGL[1] & 0X000000FF) << 24) | ((IMU1MSGH[1] & 0XFFFFFF00) >> 8);
		IMUdata[0] = *(float32*)&temp;
		temp = InverseOrder32(((IMU1MSGH[1] & 0X000000FF) << 24) | ((IMU1MSGL[2] & 0XFFFFFF00) >> 8));//Euler Angle Y
		IMUhex[1] = ((IMU1MSGH[1] & 0X000000FF) << 24) | ((IMU1MSGL[2] & 0XFFFFFF00) >> 8);
		IMUdata[1] = *(float32*)&temp;
		temp = InverseOrder32(((IMU1MSGL[2] & 0X000000FF) << 24) | ((IMU1MSGH[2] & 0XFFFFFF00) >> 8));//Euler Angle Z
		IMUhex[2] = ((IMU1MSGL[2] & 0X000000FF) << 24) | ((IMU1MSGH[2] & 0XFFFFFF00) >> 8);
		IMUdata[2] = *(float32*)&temp;
	}
	else
		IMUdata[0] = IMUdata[1] = IMUdata[2] = 4;//返回无效数据

	return missCount;
}

Uint32 IMU2_read_LpCan(Uint32 *IMUhex,float32 *IMUdata)
{
	//GET EULER ANGLE
	ECanbMboxes.MBOX1.MDL.all = 0x3A020009;
	ECanbMboxes.MBOX1.MDH.all = 0x0000000B;

	ECanbRegs.CANTRS.all = 0x00000002;  // Set TRS for  mailbox0-2
	while(ECanbRegs.CANTA.all != 0x00000002 ) {}  // Wait for all TAn bits to be set..
	ECanbRegs.CANTA.all = 0x00000002;   // Clear mailbox0 TAn

	ECanbMboxes.MBOX1.MDL.all = 0x000D0A00;
	ECanbMboxes.MBOX1.MDH.all = 0x00000000;

	ECanbRegs.CANTRS.all = 0x00000002;  // Set TRS for mailbox0-2
	while(ECanbRegs.CANTA.all != 0x00000002 ) {}  // Wait for all TAn bits to be set..
	ECanbRegs.CANTA.all = 0x00000002;   // Clear mailbox0 TAn

	volatile struct MBOX *Mailbox;

	Uint16 count2 = 0;
	Uint32 missCount = 0;

	Uint32 IMU2MSGL[4];
	Uint32 IMU2MSGH[4];


	while(count2<4 && missCount<missCountTolerated)
	{
		if(ECanbRegs.CANRMP.bit.RMP17 == 1)
		{
			Mailbox = &ECanbMboxes.MBOX0 + 17;
			IMU2MSGL[count2] = Mailbox->MDL.all;
			IMU2MSGH[count2] = Mailbox->MDH.all;
			ECanbRegs.CANRMP.bit.RMP17 = 1;//Reset RMP[n]
			count2 ++;
		}
		else
		{
			missCount++;
		}
	}


	//Analyze the data chunk
	if(IMU2MSGL[0] == 0X3A020009)
	{
		Uint32 temp;
		temp = InverseOrder32(((IMU2MSGL[1] & 0X000000FF) << 24) | ((IMU2MSGH[1] & 0XFFFFFF00) >> 8));//Euler Angle X
		IMUhex[3] = ((IMU2MSGL[1] & 0X000000FF) << 24) | ((IMU2MSGH[1] & 0XFFFFFF00) >> 8);
		IMUdata[3] = *(float32*)&temp;
		temp = InverseOrder32(((IMU2MSGH[1] & 0X000000FF) << 24) | ((IMU2MSGL[2] & 0XFFFFFF00) >> 8));//Euler Angle Y
		IMUhex[4] = ((IMU2MSGH[1] & 0X000000FF) << 24) | ((IMU2MSGL[2] & 0XFFFFFF00) >> 8);
		IMUdata[4] = *(float32*)&temp;
		temp = InverseOrder32(((IMU2MSGL[2] & 0X000000FF) << 24) | ((IMU2MSGH[2] & 0XFFFFFF00) >> 8));//Euler Angle Z
		IMUhex[5] = ((IMU2MSGL[2] & 0X000000FF) << 24) | ((IMU2MSGH[2] & 0XFFFFFF00) >> 8);
		IMUdata[5] = *(float32*)&temp;
	}
	else
		IMUdata[3] = IMUdata[4] = IMUdata[5] = 4;
	return missCount;
}


Uint32 IMU3_read_LpCan(Uint32 *IMUhex,float32 *IMUdata)
{
	//GET EULER ANGLE
	ECanbMboxes.MBOX2.MDL.all = 0x3A030009;
	ECanbMboxes.MBOX2.MDH.all = 0x0000000C;

	ECanbRegs.CANTRS.all = 0x00000004;  // Set TRS for  mailbox0-2
	while(ECanbRegs.CANTA.all != 0x00000004 ) {}  // Wait for all TAn bits to be set..
	ECanbRegs.CANTA.all = 0x00000004;   // Clear mailbox0 TAn

	ECanbMboxes.MBOX2.MDL.all = 0x000D0A00;
	ECanbMboxes.MBOX2.MDH.all = 0x00000000;

	ECanbRegs.CANTRS.all = 0x00000004;  // Set TRS for mailbox0-2
	while(ECanbRegs.CANTA.all != 0x00000004 ) {}  // Wait for all TAn bits to be set..
	ECanbRegs.CANTA.all = 0x00000004;   // Clear mailbox0 TAn

	volatile struct MBOX *Mailbox;

	Uint16 count3 = 0;
	Uint32 missCount = 0;
	Uint32 IMU3MSGL[4];
	Uint32 IMU3MSGH[4];

	while(count3<4 && missCount<missCountTolerated)
	{
		if(ECanbRegs.CANRMP.bit.RMP18 == 1)
		{
			Mailbox = &ECanbMboxes.MBOX0 + 18;
			IMU3MSGL[count3] = Mailbox->MDL.all;
			IMU3MSGH[count3] = Mailbox->MDH.all;
			ECanbRegs.CANRMP.bit.RMP18 = 1;//Reset RMP[n]
			count3 ++;
		}
		else
		{
			missCount ++;
		}

	}
	//Analyze the data chunk
	if(IMU3MSGL[0] == 0X3A030009)
	{
		Uint32 temp;
		temp = InverseOrder32(((IMU3MSGL[1] & 0X000000FF) << 24) | ((IMU3MSGH[1] & 0XFFFFFF00) >> 8));//Euler Angle X
		IMUhex[6] = ((IMU3MSGL[1] & 0X000000FF) << 24) | ((IMU3MSGH[1] & 0XFFFFFF00) >> 8);
		IMUdata[6] = *(float32*)&temp;
		temp = InverseOrder32(((IMU3MSGH[1] & 0X000000FF) << 24) | ((IMU3MSGL[2] & 0XFFFFFF00) >> 8));//Euler Angle Y
		IMUhex[7] = ((IMU3MSGH[1] & 0X000000FF) << 24) | ((IMU3MSGL[2] & 0XFFFFFF00) >> 8);
		IMUdata[7] = *(float32*)&temp;
		temp = InverseOrder32(((IMU3MSGL[2] & 0X000000FF) << 24) | ((IMU3MSGH[2] & 0XFFFFFF00) >> 8));//Euler Angle Z
		IMUhex[8] = ((IMU3MSGL[2] & 0X000000FF) << 24) | ((IMU3MSGH[2] & 0XFFFFFF00) >> 8);
		IMUdata[8] = *(float32*)&temp;
	}
	else
		IMUdata[6] = IMUdata[7] = IMUdata[8] = 4;
	return missCount;
}


void IMU_GotoStream()
{
	ECanbMboxes.MBOX0.MDL.all = 0x3A010007;
	ECanbMboxes.MBOX0.MDH.all = 0x00000008;
	ECanbMboxes.MBOX1.MDL.all = 0x3A020007;
	ECanbMboxes.MBOX1.MDH.all = 0x00000009;
	ECanbMboxes.MBOX2.MDL.all = 0x3A030007;
	ECanbMboxes.MBOX2.MDH.all = 0x0000000A;
	ECanbRegs.CANTRS.all = 0x00000007;  // Set TRS for  mailbox0-2
	while(ECanbRegs.CANTA.all != 0x00000007 ) {}  // Wait for all TAn bits to be set..
	ECanbRegs.CANTA.all = 0x00000007;   // Clear mailbox0 TAn
	ECanbMboxes.MBOX0.MDL.all = 0x000D0A00;
	ECanbMboxes.MBOX0.MDH.all = 0x00000000;
	ECanbMboxes.MBOX1.MDL.all = 0x000D0A00;
	ECanbMboxes.MBOX1.MDH.all = 0x00000000;
	ECanbMboxes.MBOX2.MDL.all = 0x000D0A00;
	ECanbMboxes.MBOX2.MDH.all = 0x00000000;
	ECanbRegs.CANTRS.all = 0x00000007;  // Set TRS for mailbox0-2
	while(ECanbRegs.CANTA.all != 0x00000007 ) {}  // Wait for all TAn bits to be set..
	ECanbRegs.CANTA.all = 0x00000007;   // Clear mailbox0 TAn
}


