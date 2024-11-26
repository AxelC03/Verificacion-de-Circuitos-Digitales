module ALU8bit(a, b, opCode, res, ovFl);
	input [7:0] a, b; 	// Bits de entrada
	input [3:0] opCode; 	// Codigo de operacion
	output [7:0] res; 	// Resultado
	output ovFl; 		// Bandera de desbordamiento

	reg [7:0] auxRes;	// Variable auxiliar para el resultado
	wire [8:0] fullRes;	// Variable auxiliar para el resultado
				// incluyendo el desbordamiento
	always@(*) begin
		case(opCode)
			// Operaciones aritmeticas
			4'b0000: auxRes = a + b;
			4'b0001: auxRes = a - b;
			4'b0010: auxRes = a * b;
			4'b0011: auxRes = a / b;
			// Operaciones a nivel bit
			4'b0100: auxRes = a << 1; // Desplazamiento a la izquierda
			4'b0101: auxRes = a >> 1; // Desplazamiento a la derecha
			4'b0110: auxRes = {a[6:0], a[7]}; // Rotar a la izquierda
			4'b0111: auxRes = {a[0], a[7:1]}; // Rotar a la derecha
			// Operaciones logicas
			4'b1000: auxRes = a & b;
			4'b1001: auxRes = a | b;
			4'b1010: auxRes = a ^ b;
			4'b1011: auxRes = ~(a & b);
			4'b1100: auxRes = ~(a | b);
			4'b1101: auxRes = ~(a ^ b);
			4'b1110: auxRes = ~a;
			// Comparacion de igualdad
			4'b1111: auxRes = (a == b) ? 8'd1 : 8'd0; // Operador ternario
			// para verificar si a y b son iguales
			default: auxRes = 8'bxxxx_xxxx;
		endcase
	end

	assign res = auxRes;
	assign fullRes = {1'b0, a} + {1'b0, b};
	assign ovFl = fullRes[8];
endmodule

module ALU8bit_tb;
	reg [7:0] a, b;
	reg [3:0] opCode;
	wire [7:0] res;
	wire ovFl;
	
	ALU8bit DUT(a, b, opCode, res, ovFl);

	// Se inicializan las variables de entrada
	initial begin
		a = 8'b00110011;
		b = 8'b11001100;
		opCode = 4'b0000;
		#100; $stop; // Detiene la simulacion al cabo de 100 ns
	end
	
	// Maquina de estados para generar valores en las entradas
	// y mostrarlos a la salida
	always begin
		a = $random;
		b = $random;
		opCode = $random;
		$display("a = %b, b = %b, opCode = %b", a, b, opCode);
		$display("Res = %b", res);
		#10;
	end
endmodule 