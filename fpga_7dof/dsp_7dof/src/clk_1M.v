	module clk_1M(clk0, rst_n, clk_1M);
	
	input clk0, rst_n;
	output clk_1M;
	
	reg [7:0] cnt;
	reg clk_1M;
	always@(posedge clk0 or negedge rst_n)
	begin
		if(!rst_n)	
		begin
			cnt <= 0;
			clk_1M <= 0;
		end
		else
		begin
			if(cnt == 8'd19) 
			begin
				cnt <= 0;
				clk_1M <= ~clk_1M;
			end
			else	cnt <= cnt + 8'h1;
		end
	end
	
	endmodule
	