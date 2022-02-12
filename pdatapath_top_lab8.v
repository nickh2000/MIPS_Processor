`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Majid Sabbagh (sabbagh.m@husky.neu.edu)
// 
// Create Date: 08/17/2014 02:18:36 PM
// Design Name: 
// Module Name: eightbit_alu_top
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module eight_bit_MUX(input[7:0] a, input [7:0] b, input s, output reg [7:0] c);
    always @*
        c <= s ? b : a;
endmodule

module nine_bit_MUX(input[8:0] a, input [8:0] b, input s, output reg [8:0] c);
    always @*           
        c <= s ? b : a;
endmodule

module pc_counter(input clock, input reset, input take_branch, input[7:0] immediate, output reg [7:0] addr);
    always @(posedge clock, posedge reset) begin
        if (reset) addr = 0;
        else if (take_branch) addr = addr + (immediate);
        else addr = addr + 1;
    end
endmodule




module pdatapath_top(
		input wire clk,				// General clock input
		input wire top_pb_clk,		// PBN1 clock input
        input wire rst_general,		// PBN0 clock reset for memory blocks
		output [7:0] led,			// add-on board led[5:0], + LD0, LD1
		output wire ovf_ctrl,    	// LD3 for overflow
		output [3:0] disp_en,		// 7-Segment display enable
		output [6:0] seg7_output	// 7-segment display output
    );
	
	// ALU inteface
    wire [7:0] alu_input1, alu_input2;
    wire [7:0] alu_output;
    wire [2:0] ALUOp;
    wire       alu_ovf;
    wire       take_branch;
    
    wire [15:0] instruction;
    //insturction fields
    wire [3:0] opcode;
    wire [1:0] rs_addr;
    wire [1:0] rt_addr;
    wire [1:0] rd_addr;
    wire [7:0] immediate;
    //control signals
    wire RegDst;
    wire RegWrite;
    wire ALUSrc1;
    wire ALUSrc2;
    wire MemWrite;
    wire MemToReg;

    wire [1:0] regfile_WriteAddress;//destination register address
    wire [8:0] regfile_WriteData;//result data
    wire [8:0] regfile_ReadData1;//source register1 data
    wire [8:0] regfile_ReadData2;//source register2 data

    wire [8:0] alu_result;
    wire [8:0] Data_Mem_Out;
	wire [7:0] zero_register;
	
	// PC and debouce clock
	wire [7:0] pc;
	wire pb_clk_debounced;

	assign zero_register = 8'b0;	//ZERO constant
	assign alu_result = {alu_ovf, alu_output};
	
	// Assign LEDs
    assign led = alu_output;
	assign ovf_ctrl = alu_ovf;

	// Debounce circuit
    debounce debounce_clk(
        .clk_in(clk),
        .rst_in(rst_general),
        .sig_in(top_pb_clk),
        .sig_debounced_out(pb_clk_debounced)
    );
	
	// 7-Segment display module
	Adaptor_display display(
		.clk(clk), 					// system clock
		.input_value(alu_output),	// 8-bit input [7:0] value to display
		.disp_en(disp_en),			// output [3:0] 7 segment display enable
		.seg7_output(seg7_output)	// output [6:0] 7 segment signals
	);
	
    //Instantiate Your PC Register here
    pc_counter COUNT(.reset(rst_general), .clock(pb_clk_debounced), .addr(pc), .take_branch(take_branch), .immediate(immediate));
	//Instantiate Your instruction Memory here
    instr_mem instruction_mem(.a(pc), .spo(instruction));
	//Instantiate Your instruction decoder here
    inst_decoder DEC(.instruction(instruction), .opcode(opcode), .rs_addr(rs_addr), .rt_addr(rt_addr), .rd_addr(rd_addr), .immediate(immediate), .RegDst(RegDst), .RegWrite(RegWrite), .ALUSrc1(ALUSrc1), .ALUSrc2(ALUSrc2), .ALUOp(ALUOp), .MemWrite(MemWrite), .MemToReg(MemToReg));
	//Instantiate Your alu-regfile here
    eight_bit_MUX MUX1(regfile_ReadData1[7:0], zero_register, ALUSrc1, alu_input1);
    
    eight_bit_MUX MUX2(regfile_ReadData2[7:0], immediate, ALUSrc2, alu_input2);
	    
	eightbit_alu ALU(.a(alu_input1), .b(alu_input2), .s(ALUOp), .f(alu_output), .ovf(alu_ovf), .take_branch(take_branch));
    
    regfile REGFILE(.rd0_addr(rs_addr), .rd1_addr(rt_addr), .wr_en(RegWrite), .rst(right_pb_rst_general), .clk(pb_clk_debounced), .wr_addr(regfile_WriteAddress),
	  .wr_data(regfile_WriteData), .rd0_data(regfile_ReadData1), .rd1_data(regfile_ReadData2)); 
 	//Instantiate Your data memory here
	data_memory mem(.clk(clk), .we(MemWrite), .a(alu_output), .d(regfile_ReadData2), .spo(Data_Mem_Out));
	//Mux for regfile_writedata
	nine_bit_MUX MUX3(alu_result, Data_Mem_Out, MemToReg, regfile_WriteData);
	//Mux for RegDST
    eight_bit_MUX MUX4(rt_addr, rd_addr, RegDst, regfile_WriteAddress);
	//Instantiate Your VIO core here
	
	vio_0 VIO(.clk(clk), 
	   .probe_in0(alu_output),
	   .probe_in1(alu_ovf),
	   .probe_in2(take_branch),
	   .probe_in3(regfile_ReadData1),
	   .probe_in4(regfile_ReadData2),
	   .probe_in5(alu_input1),
	   .probe_in6(alu_input2),
	   .probe_in7(regfile_WriteData),
	   .probe_in8(Data_Mem_Out),
	   .probe_in9(opcode),
	   .probe_in10(rs_addr),
	   .probe_in11(rt_addr),
	   .probe_in12(rd_addr),
	   .probe_in13(immediate),
	   .probe_in14(RegDst),
	   .probe_in15(RegWrite),
	   .probe_in16(ALUSrc1),
	   .probe_in17(ALUSrc2),
	   .probe_in18(ALUOp),
	   .probe_in19(MemWrite),
	   .probe_in20(MemToReg),
	   .probe_in21(pc),
	   .probe_in22(instruction));
	   
	
	
endmodule