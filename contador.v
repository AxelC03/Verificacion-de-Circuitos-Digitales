module contador#(parameter N=8)(clk, sel, reset, stop, q);
	input clk, sel, reset, stop;
	output reg [N-1:0] q;
	
	// Sel = 0 => Asc
	// Sel = 1 => Desc
	always@(posedge clk, posedge reset, posedge sel)
		if(reset)
			q = 0;
		else if(!stop) begin
			if(sel)
				q = q - 1;
			else
				q = q + 1;
		end
		else
			q = q;

endmodule

module contador_tb#(parameter N=8);
	reg clk, sel, reset, stop;
	wire [N-1:0] q;

	integer i = 0;

	contador DUT(clk, sel, reset, stop, q);

	initial begin
		clk=0; #5;
		forever begin
			clk=~clk; #1;
		end
	end
	
	initial begin
		reset = 1;
		stop = 0;
		$display("Contador arranque: %b", q);
		for(i=0; i<10; i=i+1) begin
			//$display("%b", clk); #5;
			reset = 1;
			$display("%b", q); #2;
		end
		$display("Reset = 0");
		for(i=0; i<10; i=i+1) begin
			//$display("%b", clk); #5;
			reset = 0;
			$display("%b", q); #2;
		end
		$display("sel = 1");
		for(i=0; i<10; i=i+1) begin
			//$display("%b", clk); #5;
			sel = 1;
			$display("%b", q); #2;	
		end
		$display("stop = 1");
		for(i=0; i<10; i=i+1) begin
			//$display("%b", clk); #5;
			stop = 1;
			$display("%b", q); #2;
		end
		$stop;
	end
endmodule 