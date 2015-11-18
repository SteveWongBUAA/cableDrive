module AbsEncoder(clk, 
						clk_rd, 
						rst_n, data_in, cs0_n, cs1_n, busy0, convst, enc_rst, 
						data_out0, data_out1, data_out2, data_out3, data_out4, data_out5, data_out6, data_out7, 
						data_out8, data_out9, data_outA, data_outB, data_outC, data_outD, data_outE, data_outF 
						);

	input clk, rst_n, busy0;
	input clk_rd;
	input[15:0] 	data_in;

	output 			convst, enc_rst, cs0_n, cs1_n;
	output[15:0]	data_out0;
	output[15:0]	data_out1;
	output[15:0]	data_out2;
	output[15:0]	data_out3;
	output[15:0]	data_out4;
	output[15:0]	data_out5;
	output[15:0]	data_out6;
	output[15:0]	data_out7;
	output[15:0]	data_out8;
	output[15:0]	data_out9;
	output[15:0]	data_outA;
	output[15:0]	data_outB;
	output[15:0]	data_outC;
	output[15:0]	data_outD;
	output[15:0]	data_outE;
	output[15:0]	data_outF;

	reg[15:0]		data_out0;	
	reg[15:0]		data_out1;
	reg[15:0]		data_out2;
	reg[15:0]		data_out3;
	reg[15:0]		data_out4;
	reg[15:0]		data_out5;
	reg[15:0]		data_out6;
	reg[15:0]		data_out7;
	reg[15:0]		data_out8;
	reg[15:0]		data_out9;
	reg[15:0]		data_outA;
	reg[15:0]		data_outB;
	reg[15:0]		data_outC;
	reg[15:0]		data_outD;
	reg[15:0]		data_outE;
	reg[15:0]		data_outF;

	
	reg				convst, enc_rst, cs0_n, cs1_n;
	
	reg[2:0]		enc_rst_cnt;
	reg[7:0]		data_cnt;
	reg[3:0]		convst_cnt;

	wire        outputflag;                               //indicate the output state;
	reg         clk_rd_r1;
	reg		   clk_rd_r0;
  
	assign outputflag = (clk_rd_r0 & (~clk_rd_r1));

	// 从复位开始计时8个时钟周期用于复位AD芯片
	always@ (posedge clk or negedge rst_n)
	begin
		if(!rst_n)
		begin
			enc_rst <= 1'b1;
			enc_rst_cnt <= 0;
		end
		else
		begin
			if(enc_rst_cnt == 3'b111)
				enc_rst <= 0;
			else
				enc_rst_cnt <= enc_rst_cnt + 3'b001;
		end
	end
	
	reg busy0_r;
	reg rd_start;
	wire busy0_neg;
	assign busy0_neg = (busy0_r & (~busy0));	// 提取busy下降沿
	
	always@ (posedge clk or negedge rst_n)
	begin
		if(!rst_n)
		begin
			busy0_r <= 1'b0;
			clk_rd_r0 <= 1'b0;
			clk_rd_r1 <= 1'b0;
		end
		else
		begin
			busy0_r <= busy0;
         clk_rd_r0 <= clk_rd;
         clk_rd_r1 <= clk_rd_r0;
		end
	end
				
	
	// 数据采集计数器
	always@ (posedge clk or negedge rst_n)
	begin
		if(!rst_n)
		begin
			data_cnt <= 0;
		end
		else
		begin
			if(rd_start)
				data_cnt <= data_cnt + 8'h1;
			else
				data_cnt <= 8'd0;
		end
	end
	
	// convst低电平计数器
	always@ (posedge clk or negedge rst_n)
	begin
		if(!rst_n)
		begin
			convst_cnt <= 0;
		end
		else
		begin
			if(!convst)
				convst_cnt <= convst_cnt + 4'h1;
			else
				convst_cnt <= 4'd0;
		end
	end	
	
	always@ (posedge clk or negedge rst_n)
	begin
		if(!rst_n)
		begin
			data_out0 <= 0;
			data_out1 <= 0;
			data_out2 <= 0;
			data_out3 <= 0;
			data_out4 <= 0;
			data_out5 <= 0;
			data_out6 <= 0;
			data_out7 <= 0;
			data_out8 <= 0;
			data_out9 <= 0;
			data_outA <= 0;
			data_outB <= 0;
			data_outC <= 0;
			data_outD <= 0;
			data_outE <= 0;
			data_outF <= 0;
			convst <= 1'b1;			// 转换置高
			cs0_n <= 1'b1;
			cs1_n <= 1'b1;
			rd_start <= 1'b0;
		end
		else
		begin
			if(outputflag && (!enc_rst))
			begin
				convst <= 1'b0;		// convst 拉低
			end
			
			if(convst_cnt == 4'd4)
				convst <= 1'b1;		// convst 置高
			
			if(busy0_neg)				// busy下降沿，开始读取数据
			begin
				rd_start <= 1'b1;
			end
			
			if(rd_start)
			begin
				case(data_cnt)
					8'd0:	
					begin
						cs0_n <= 0;
						cs1_n <= 1;
					end
					8'd1:
					begin
						cs0_n <= 1;
						cs1_n <= 1;
						data_out0 <= data_in;
					end
					8'd2:	
					begin
						cs0_n <= 0;
						cs1_n <= 1;
					end
					8'd3:
					begin
						cs0_n <= 1;
						cs1_n <= 1;
						data_out1 <= data_in;
					end
					8'd4:	
					begin
						cs0_n <= 0;
						cs1_n <= 1;
					end
					8'd5:
					begin
						cs0_n <= 1;
						cs1_n <= 1;
						data_out2 <= data_in;
					end
					8'd6:	
					begin
						cs0_n <= 0;
						cs1_n <= 1;
					end
					8'd7:
					begin
						cs0_n <= 1;
						cs1_n <= 1;
						data_out3 <= data_in;
					end
					8'd8:	
					begin
						cs0_n <= 0;
						cs1_n <= 1;
					end
					8'd9:
					begin
						cs0_n <= 1;
						cs1_n <= 1;
						data_out4 <= data_in;
					end
					8'd10:	
					begin
						cs0_n <= 0;
						cs1_n <= 1;
					end
					8'd11:
					begin
						cs0_n <= 1;
						cs1_n <= 1;
						data_out5 <= data_in;
					end
					8'd12:
					begin
						cs0_n <= 0;
						cs1_n <= 1;
					end
					8'd13:
					begin
						cs0_n <= 1;
						cs1_n <= 1;
						data_out6 <= data_in;
					end
					8'd14:	
					begin
						cs0_n <= 0;
						cs1_n <= 1;
					end
					8'd15:
					begin
						cs0_n <= 1;
						cs1_n <= 1;
						data_out7 <= data_in;
						//rd_start <= 0;
					end
					
					8'd16:
					begin
						cs0_n <= 1;
						cs1_n <= 0;				
					end
					8'd17:
					begin
						cs0_n <= 1;
						cs1_n <= 1;
						data_out8 <= data_in;
					end
					8'd18:
					begin
						cs0_n <= 1;
						cs1_n <= 0;
					end
					8'd19:
					begin
						cs0_n <= 1;
						cs1_n <= 1;
						data_out9 <= data_in;
					end
					8'd20:	
					begin
						cs0_n <= 1;
						cs1_n <= 0;
					end
					8'd21:
					begin
						cs0_n <= 1;
						cs1_n <= 1;
						data_outA <= data_in;
					end
					8'd22:
					begin
						cs0_n <= 1;
						cs1_n <= 0;
					end
					8'd23:
					begin
						cs0_n <= 1;
						cs1_n <= 1;
						data_outB <= data_in;
					end
					8'd24:
					begin
						cs0_n <= 1;
						cs1_n <= 0;
					end
					8'd25:
					begin
						cs0_n <= 1;
						cs1_n <= 1;
						data_outC <= data_in;
					end
					8'd26:
					begin
						cs0_n <= 1;
						cs1_n <= 0;					
					end
					8'd27:
					begin
						cs0_n <= 1;
						cs1_n <= 1;
						data_outD <= data_in;
					end
					8'd28:
					begin
						cs0_n <= 1;
						cs1_n <= 0;	
					end
					8'd29:
					begin
						cs0_n <= 1;
						cs1_n <= 1;	
						data_outE <= data_in;
					end
					8'd30:
					begin
						cs0_n <= 1;
						cs1_n <= 0;
					end
					8'd31:
					begin
						cs0_n <= 1;
						cs1_n <= 1;	
						data_outF <= data_in;
						rd_start <= 0;
					end
					
					default:
					begin
						cs0_n <= 1;
						cs1_n <= 1;
					end
				endcase
			end
		end
	end


endmodule

// No more