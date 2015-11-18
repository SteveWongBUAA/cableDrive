/*下面是对外设的各个地址进行了宏定义*/ 

//Read Add 
`define IncEncoder1Add		6'h10
`define IncEncoder2Add		6'h11
`define IncEncoder3Add		6'h12
`define IncEncoder4Add		6'h13
`define IncEncoder5Add		6'h14
`define IncEncoder6Add		6'h15
`define IncEncoder7Add		6'h16

`define AbsEncoder0Add		6'h20
`define AbsEncoder1Add		6'h21
`define AbsEncoder2Add		6'h22
`define AbsEncoder3Add		6'h23
`define AbsEncoder4Add		6'h24
`define AbsEncoder5Add		6'h25
`define AbsEncoder6Add		6'h26
`define AbsEncoder7Add		6'h27
`define AbsEncoder8Add		6'h28
`define AbsEncoder9Add		6'h29
`define AbsEncoderAAdd		6'h2A
`define AbsEncoderBAdd		6'h2B
`define AbsEncoderCAdd		6'h2C
`define AbsEncoderDAdd		6'h2D
`define AbsEncoderEAdd		6'h2E
`define AbsEncoderFAdd		6'h2F

//Write Add
`define ControlAdd		6'h00
`define LedAdd				6'h01
 
/*模块变量定义*/ 
module ParaBus(DSP_Add, DSP_Data, WR, RD, CS0, LED,
					IncEncoder1, IncEncoder2, IncEncoder3, IncEncoder4,
					IncEncoder5, IncEncoder6, IncEncoder7, IncEncoderRd,
					AbsEncoder0, AbsEncoder1, AbsEncoder2, AbsEncoder3,
					AbsEncoder4, AbsEncoder5, AbsEncoder6, AbsEncoder7,
					AbsEncoder8, AbsEncoder9, AbsEncoderA, AbsEncoderB,
					AbsEncoderC, AbsEncoderD, AbsEncoderE, AbsEncoderF); 
/*CPLD输入信号的定义*/
input [7:0] DSP_Add;			//DSP对CPLD的地址定义 
input WR,RD,CS0;				//读写和CPLD片选定义
input [15:0] IncEncoder1;
input [15:0] IncEncoder2;
input [15:0] IncEncoder3;
input [15:0] IncEncoder4;
input [15:0] IncEncoder5;
input [15:0] IncEncoder6;
input [15:0] IncEncoder7;

input [15:0] AbsEncoder0;
input [15:0] AbsEncoder1;
input [15:0] AbsEncoder2;
input [15:0] AbsEncoder3;
input [15:0] AbsEncoder4;
input [15:0] AbsEncoder5;
input [15:0] AbsEncoder6;
input [15:0] AbsEncoder7;
input [15:0] AbsEncoder8;
input [15:0] AbsEncoder9;
input [15:0] AbsEncoderA;
input [15:0] AbsEncoderB;
input [15:0] AbsEncoderC;
input [15:0] AbsEncoderD;
input [15:0] AbsEncoderE;
input [15:0] AbsEncoderF;

 
/*CPLD输出信号的定义*/
output [3:0] LED;				//4位LED等定义输出
output IncEncoderRd;
 
inout [15:0] DSP_Data;		//DSP与CPLD之间的数据线定义为输入输出 
 
/*变量寄存器的定义*/ 
reg [15:0] DSP_Data_reg;
reg [15:0] Control_reg;
reg [3:0] LED_reg; 
 
////*循环执行判断按键与CAN发送接收的变化*/ 
//always  
//begin 
//
//end 
 
 
/*上边沿写信号发生,主要是将DSP的数据送到CPLD定义的各自的的寄存器中,也就是通过识别不同的地址来把数据写入各自寄存器中*/ 
always @ (posedge WR) 
begin 
	if(CS0==1'b0)	//判断CPLD是否被片选 
	begin 
		case(DSP_Add)
			`ControlAdd:	// 控制寄存器地址
			begin
				Control_reg = DSP_Data;
			end
			`LedAdd: 		// LED灯地址
			begin 
				LED_reg = DSP_Data[3:0]; 
			end
		endcase 
	end 
end 
 
/*下边沿来读数据,这里主要是从CPLD中读数据到DSP中,与上面的DSP往CPLD中写数据正好相反*/ 
always @ (negedge RD)///下边沿产生 
begin 
if(CS0 == 1'b0)	//CPLD被片选 
    begin
		case(DSP_Add)
			`ControlAdd:
			begin
				DSP_Data_reg = Control_reg;
			end
        	`IncEncoder1Add:	//
			begin 
				DSP_Data_reg = IncEncoder1;	//
			end
        	`IncEncoder2Add:	//
			begin 
				DSP_Data_reg = IncEncoder2;	//
			end
        	`IncEncoder3Add:	//
			begin 
				DSP_Data_reg = IncEncoder3;	//
			end
        	`IncEncoder4Add:	//
			begin 
				DSP_Data_reg = IncEncoder4;	//
			end
        	`IncEncoder5Add:	//
			begin 
				DSP_Data_reg = IncEncoder5;	//
			end
        	`IncEncoder6Add:	//
			begin 
				DSP_Data_reg = IncEncoder6;	//
			end
        	`IncEncoder7Add:	//
			begin 
				DSP_Data_reg = IncEncoder7;	//
			end
			// 绝对编码器地址
        	`AbsEncoder0Add:	//
			begin 
				DSP_Data_reg = AbsEncoder0;	//
			end
        	`AbsEncoder1Add:	//
			begin 
				DSP_Data_reg = AbsEncoder1;	//
			end
        	`AbsEncoder2Add:	//
			begin 
				DSP_Data_reg = AbsEncoder2;	//
			end
        	`AbsEncoder3Add:	//
			begin 
				DSP_Data_reg = AbsEncoder3;	//
			end
        	`AbsEncoder4Add:	//
			begin 
				DSP_Data_reg = AbsEncoder4;	//
			end
        	`AbsEncoder5Add:	//
			begin 
				DSP_Data_reg = AbsEncoder5;	//
			end
        	`AbsEncoder6Add:	//
			begin 
				DSP_Data_reg = AbsEncoder6;	//
			end			
       	`AbsEncoder7Add:	//
			begin 
				DSP_Data_reg = AbsEncoder7;	//
			end
        	`AbsEncoder8Add:	//
			begin 
				DSP_Data_reg = AbsEncoder8;	//
			end
        	`AbsEncoder9Add:	//
			begin 
				DSP_Data_reg = AbsEncoder9;	//
			end
        	`AbsEncoderAAdd:	//
			begin 
				DSP_Data_reg = AbsEncoderA;	//
			end
        	`AbsEncoderBAdd:	//
			begin 
				DSP_Data_reg = AbsEncoderB;	//
			end
        	`AbsEncoderCAdd:	//
			begin 
				DSP_Data_reg = AbsEncoderC;	//
			end
        	`AbsEncoderDAdd:	//
			begin 
				DSP_Data_reg = AbsEncoderD;	//
			end
        	`AbsEncoderEAdd:	//
			begin 
				DSP_Data_reg = AbsEncoderE;	//
			end			
       	`AbsEncoderFAdd:	//
			begin 
				DSP_Data_reg = AbsEncoderF;	//
			end	
			endcase 
	end 
end
 
assign IncEncoderRd = Control_reg[0];
assign LED = LED_reg;
assign DSP_Data = (CS0 == 1'b0 && RD == 1'b0) ? DSP_Data_reg : 16'hz;  
 
endmodule 