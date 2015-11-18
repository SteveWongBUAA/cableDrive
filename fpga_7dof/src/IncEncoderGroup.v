module IncEncoderGroup(clk_encoder, rst_n, RD, chA, chB,
								data_out0, data_out1, data_out2, data_out3, data_out4, data_out5, data_out6);

input clk_encoder, rst_n;
input RD;
input [6:0]chA;
input [6:0]chB;

output [15:0]data_out0;
output [15:0]data_out1;
output [15:0]data_out2;
output [15:0]data_out3;
output [15:0]data_out4;
output [15:0]data_out5;
output [15:0]data_out6;

	/* Encoder */
	Encoder	encoder_0(
					.clk(clk_encoder), 
					.clk_rd(~RD), 
					.chA(chA[0]), 
					.chB(chB[0]), 
					.rst_n(rst_n), 
					.data(data_out0)
					);
					
	Encoder	encoder_1(
					.clk(clk_encoder), 
					.clk_rd(~RD), 
					.chA(chA[1]), 
					.chB(chB[1]), 
					.rst_n(rst_n), 
					.data(data_out1)
					);
					
	Encoder	encoder_2(
					.clk(clk_encoder), 
					.clk_rd(~RD), 
					.chA(chA[2]), 
					.chB(chB[2]), 
					.rst_n(rst_n), 
					.data(data_out2)
					);
					
	Encoder	encoder_3(
					.clk(clk_encoder), 
					.clk_rd(~RD), 
					.chA(chA[3]), 
					.chB(chB[3]), 
					.rst_n(rst_n), 
					.data(data_out3)
					);
					
	Encoder	encoder_4(
					.clk(clk_encoder), 
					.clk_rd(~RD), 
					.chA(chA[4]), 
					.chB(chB[4]), 
					.rst_n(rst_n), 
					.data(data_out4)
					);
			
	Encoder	encoder_5(
					.clk(clk_encoder), 
					.clk_rd(~RD), 
					.chA(chA[5]), 
					.chB(chB[5]), 
					.rst_n(rst_n), 
					.data(data_out5)
					);
					
	Encoder	encoder_6(
					.clk(clk_encoder), 
					.clk_rd(~RD), 
					.chA(chA[6]), 
					.chB(chB[6]), 
					.rst_n(rst_n), 
					.data(data_out6)
					);

	endmodule
	