//简化的光电编码器模块
module Encoder(clk, clk_rd, chA, chB, rst_n, data);
  input       clk;
  input       rst_n;
  input       chA, chB;
  input       clk_rd;

  output[15:0] data;
  
  reg[15:0]    data;                                     
  reg[15:0]    databuf;
  reg[1:0]     state;                                    //input state
  reg[1:0]	   state_r1, state_r2;
  wire         outputflag;                               //indicate the output state;
  reg          clk_rd_r1;
  reg		   	clk_rd_r0;
  wire [1:0]	state_pre, state_last;
  
  assign outputflag = (clk_rd_r0 & (~clk_rd_r1));
  
  assign state_pre = state & state_r1;
  assign state_last = state_r1 & state_r2;
  
  always@(posedge clk or negedge rst_n)
  begin
    if(!rst_n)
    begin
      state <= 0;
      databuf <= 0;
      data <= 0;
      clk_rd_r0 <= 0;
      clk_rd_r1 <= 0;
      state_r1 <= 0;
      state_r2 <= 0;
    end
    else
      begin
        state[0] <= chA;
        state[1] <= chB;
        clk_rd_r0 <= clk_rd;
        clk_rd_r1 <= clk_rd_r0;
        state_r1 <= state;
        state_r2 <= state_r1;
        if(outputflag)  
        begin
          data <= databuf;
          databuf <= 0;
          //state <= 0;
          //state_r1 <= 0;
          //state_r2 <= 0;
        end
        else
        begin
			case({state_last, state_pre})
			4'b0010:  databuf <= databuf - 16'd1;
			4'b1011:  databuf <= databuf - 16'd1;
			4'b1101:  databuf <= databuf - 16'd1;
			4'b0100:  databuf <= databuf - 16'd1;
			4'b0001:  databuf <= databuf + 16'd1;
			4'b0111:  databuf <= databuf + 16'd1;
			4'b1110:  databuf <= databuf + 16'd1;
			4'b1000:  databuf <= databuf + 16'd1;
			default:  databuf <= databuf;
			endcase
		end
      end
  end
    
  endmodule



