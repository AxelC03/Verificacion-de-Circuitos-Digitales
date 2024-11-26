module pruebaProyecto
#(parameter ADDR_WIDTH = 6, parameter DATA_WIDTH = 11, parameter OPCODE_WIDTH = 5)
// Ancho de memoria    // Ancho de palabra        // Ancho de OpCode
(clk, reset, wr, address, port_sel, data_out, portA, portB, portC, portD);

input wire clk, reset,wr;
input wire [5:0] address;
reg [10:0] data_in;
input wire [3:0] port_sel;
output reg [10:0] data_out;
inout reg [10:0] portA, portB, portC, portD;

// Definición de códigos de operación
localparam OP_MOV = 5'b0_0000;
localparam OP_STORE = 5'b0_0001;
localparam OP_LOAD = 5'b0_0010;
localparam OP_PUSH = 5'b0_0011;
localparam OP_POP = 5'b0_0100;
localparam OP_ADD = 5'b0_0101;
localparam OP_SUB = 5'b0_0110;
localparam OP_MUL = 5'b0_0111;
localparam OP_DIV = 5'b0_1000;
localparam OP_AND = 5'b0_1001;
localparam OP_OR = 5'b0_1010;
localparam OP_XOR = 5'b0_1011;
localparam OP_NOT = 5'b0_1100;
localparam OP_JUMP = 5'b0_1101;
localparam OP_JUMPC = 5'b0_1110;
localparam OP_NOP = 5'b0_1111; // No Operation
localparam OP_MAG = 5'b1_0000; // Comparacion de magnitud
localparam OP_EQU = 5'b1_0001; // Comparacion de igualdad
localparam OP_BSD = 5'b1_0010; // Desplazamiento a la derecha
localparam OP_BSI = 5'b1_0011; // Desplazamiento a la izquierda
localparam OP_DEL = 5'b1_0100; // Eliminar dato de la memoria
localparam OP_NOR = 5'b1_0101;
localparam OP_NAND = 5'b1_0110;
localparam OP_XNOR = 5'b1_0111;
localparam OP_SIMA = 5'b1_1000; // Escribir en la memoria A (Store In Memory A)
localparam OP_SIMB = 5'b1_1001; // Escribir en la memoria B 
localparam OP_SIMC = 5'b1_1010; // Escribir en la memoria C
localparam OP_SIMD = 5'b1_1011; // Escribir en la memoria D
localparam OP_INC = 5'b1_1100; // Incrementar el acumulador
localparam OP_DEC = 5'b1_1101; // Decrementar el acumulador
localparam OP_RTD = 5'b1_1110; // Rotar el acumulador a la derecha
localparam OP_RTI = 5'b1_1111; // Rotar el acumulador a la izquierda

// Definicion de los codigos de operacion para los puertos
localparam PA_IN = 3'b000;
localparam PA_OUT = 3'b001;
localparam PB_IN = 3'b010;
localparam PB_OUT = 3'b011;
localparam PC_IN = 3'b100;
localparam PC_OUT = 3'b101;
localparam PD_IN = 3'b110;
localparam PD_OUT = 3'b111;

// Definición de registros
reg [DATA_WIDTH-1:0] accumulator;
reg [ADDR_WIDTH-1:0] program_counter;
reg [3:0] instruction_register;
reg [ADDR_WIDTH-1:0] stack_pointer;
reg [DATA_WIDTH-1:0] memory [0:63];
reg [DATA_WIDTH-1:0] rx;

// Registros especificos de puertos
reg [DATA_WIDTH-1:0] memA [0:63];
reg [DATA_WIDTH-1:0] memB [0:63];
reg [DATA_WIDTH-1:0] memC [0:63];
reg [DATA_WIDTH-1:0] memD [0:63];

// Seleccion y definicion del comportamiento de los puertos
always @(posedge clk) begin
	case(port_sel)
		PA_IN: data_in <= portA;
		PA_OUT: data_out <= accumulator;
		PB_IN: data_in <= portB;
		PB_OUT: data_out <= accumulator;
		PC_IN: data_in <= portC;
		PC_OUT: data_out <= accumulator;
		PD_IN: data_in <= portD;
		PD_OUT: data_out <= accumulator;
	endcase
end

// Asignación de pines de salida
// assign data_out = accumulator;

// Comportamiento del procesador
always @(posedge clk or posedge reset) begin
    if (reset) begin
        accumulator <= 0;
        program_counter <= 0;
        instruction_register <= 0;
        stack_pointer <= 0;
    end else begin
        if (wr) begin
            memory[address] <= data_in;
			case(port_sel)
				PA_IN: memA[address] <= data_in;
				PB_IN: memB[address] <= data_in;
				PC_IN: memC[address] <= data_in;
				PD_IN: memD[address] <= data_in;
				default: memA[address] <= data_in;
			endcase
        end else begin
            // Selecciona la instrucción de la memoria según el contador de programa
            instruction_register <= memory[program_counter][DATA_WIDTH-1:DATA_WIDTH-OPCODE_WIDTH];
	    case(port_sel)
		PA_IN: instruction_register <= memA[program_counter][DATA_WIDTH-1:DATA_WIDTH-OPCODE_WIDTH];
		PB_IN: instruction_register <= memB[program_counter][DATA_WIDTH-1:DATA_WIDTH-OPCODE_WIDTH];
		PC_IN: instruction_register <= memC[program_counter][DATA_WIDTH-1:DATA_WIDTH-OPCODE_WIDTH];
		PD_IN: instruction_register <= memD[program_counter][DATA_WIDTH-1:DATA_WIDTH-OPCODE_WIDTH];
		default: instruction_register <= memA[program_counter][DATA_WIDTH-1:DATA_WIDTH-OPCODE_WIDTH];
	    endcase
            
            // Extraer el dato de la instrucción
            rx <= memory[program_counter][DATA_WIDTH-OPCODE_WIDTH-1:0];
	    rx <= memory[program_counter][DATA_WIDTH-OPCODE_WIDTH-1:0];
	    case(port_sel)
		PA_IN: rx <= memA[program_counter][DATA_WIDTH-OPCODE_WIDTH-1:0];
		PB_IN: rx <= memB[program_counter][DATA_WIDTH-OPCODE_WIDTH-1:0];
		PC_IN: rx <= memC[program_counter][DATA_WIDTH-OPCODE_WIDTH-1:0];
		PD_IN: rx <= memD[program_counter][DATA_WIDTH-OPCODE_WIDTH-1:0];
	    endcase
            
            // Decodifica la instrucción
            case(instruction_register)
                OP_MOV: accumulator <= data_in;
                OP_STORE: memory[rx] <= accumulator;
                OP_LOAD: accumulator <= memory[rx];
                OP_PUSH: begin
                            stack_pointer <= stack_pointer + 1;
                            memory[stack_pointer] <= accumulator;
                         end
                OP_POP: begin
                           accumulator <= memory[stack_pointer];
                           stack_pointer <= stack_pointer - 1;
                       	end
                OP_ADD: accumulator <= accumulator + memory[rx];
                OP_SUB: accumulator <= accumulator - memory[rx];
                OP_MUL: accumulator <= accumulator * memory[rx];
                OP_DIV: accumulator <= accumulator / memory[rx];
                OP_AND: accumulator <= accumulator & memory[rx];
                OP_OR: accumulator <= accumulator | memory[rx];
                OP_XOR: accumulator <= accumulator ^ memory[rx];
                OP_NOT: accumulator <= ~accumulator;
                OP_JUMP: program_counter <= rx;
                OP_JUMPC: if (accumulator == 0) program_counter <= rx;
		OP_NOP: accumulator <= accumulator;
		OP_MAG: begin
				if(accumulator > memory[rx])
					accumulator <= 5'b1;
				else
					accumulator <= 5'b0;
			end
		OP_EQU: begin
				if(accumulator == memory[rx])
					accumulator <= 5'b1;
				else
					accumulator <= 5'b0;
			end
		OP_BSD: accumulator <= accumulator >> 1;
		OP_BSI: accumulator <= accumulator << 1;
		OP_DEL: memory[rx] <= 0; // Este no se si este bien declarado, a ver si no borra el bootloader
		OP_NOR: accumulator <= ~(accumulator | memory[rx]);
		OP_NAND: accumulator <= ~(accumulator & memory[rx]);
		OP_XNOR: accumulator <= ~(accumulator ^ memory[rx]);
		OP_SIMA: memA[rx] <= accumulator;
		OP_SIMB: memB[rx] <= accumulator;
		OP_SIMC: memC[rx] <= accumulator;
		OP_SIMD: memD[rx] <= accumulator;
		OP_INC: accumulator <= accumulator + 1;
		OP_DEC: accumulator <= accumulator - 1;
		OP_RTD: accumulator <= {accumulator[4:0], accumulator[5]};
		OP_RTI: accumulator <= {accumulator[0], accumulator[5:1]};
            endcase
            
            // Incrementa el contador de programa
            program_counter <= program_counter + 1;
        end
    end
end

endmodule: pruebaProyecto

// ********************************************************************************************** //

module pruebaProyecto_TB();

    // Parámetros de prueba
    localparam ADDR_WIDTH = 6;
    localparam DATA_WIDTH = 10;
    localparam periodo=2;
    // Señales del testbench
    reg clk, wr;
    reg reset;
    reg [10:0] data_in;
    reg [5:0] address;
    reg [3:0] port_sel;
    wire [10:0] data_out;
    wire [10:0] portA, portB, portC, portD;
    integer i, j;

    reg [10:0] auxA, auxB, auxC, auxD;

    assign portA = auxA;
    assign portB = auxB;
    assign portC = auxC;
    assign portD = auxD;

    // Instancia del procesador
    pruebaProyecto dut (
        .clk(clk),
        .reset(reset),
        .wr(wr),
        .address(address),
        //.data_in(data_in),
		.port_sel(port_sel),
        	.data_out(data_out),
		.portA(portA),
		.portB(portB),
		.portC(portC),
		.portD(portD)
    );

    // Generación de señales de clock
    always #(periodo/2) clk = ~clk;

    // Inicialización de señales
    initial begin
        clk = 0;
        reset = 1;
        data_in = 0;
        wr = 1; //escribir instrucciones en memoria

        // Esperar un poco para asegurarnos de que el reset sea efectivo
        #(periodo*4);
        reset = 0;
	    
        //inicializa memoria con 0

	port_sel = 3'b000;
	for (i=0; i< 64; i=i+1) begin
		address=i; auxA=0;#(periodo*3);
	end
	port_sel = 3'b010;
	for (i=0; i< 64; i=i+1) begin
		address=i; auxB=0;#(periodo*3);
	end
	#5;
	// Cargar programa en memoria 
        // Instrucción 1: LOAD 34
        // Instrucción 2: ADD 35
        // Instrucción 3: STORE 36
	port_sel = 3'b000; #5;
        address = 34; auxA = 5; #(periodo*3); // Cargar el primer número (5) en la dirección 34
        address = 35; auxA = 7; #(periodo*3); // Cargar el segundo número (7) en la dirección 35
        address = 33; auxA = 6; #(periodo*3);
	address = 0; auxA = {5'b0_0010, 6'b100010}; #(periodo*3); // LOAD 34
        address = 1; auxA = {5'b0_0101, 6'b100011}; #(periodo*3); // ADD 35
        address = 2; auxA = {5'b0_0001, 6'b100100}; #(periodo*3); // STORE 36
	address = 3; auxA = {5'b0_0110, 6'b100010}; #(periodo*3);
        address = 2; auxA = {5'b0_0001, 6'b100001}; #(periodo*3); // STORE 36
	reset = 0;wr=0; #(periodo*3);
        // Asegurar que el procesador complete la ejecución
        #(periodo*100);

        // Detener simulación
        $stop;
    end

    // Monitorear el registro de acumulador
    always @(posedge clk) begin
        $display("Acumulador: %d", data_out);
    end

endmodule: pruebaProyecto_TB

