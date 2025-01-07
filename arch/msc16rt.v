/* SPDX-License-Identifier: GPL-2.0-or-later */
`timescale 1ns / 1ps
module msc16rt(
	input wire clk,
	input wire rstn,
	output wire mem_we,
	output wire mem_en,
	output wire [15:0] mem_addr,
	output wire [15:0] mem_out,
	input wire [15:0] mem_in
	);

	localparam ALU_ADD = 3'b000;
	localparam ALU_SUB = 3'b001;
	localparam ALU_AND = 3'b010;
	localparam ALU_OR = 3'b011;
	localparam ALU_XOR = 3'b100;
	localparam ALU_LSH = 3'b101;
	localparam ALU_RSH = 3'b110;

	localparam S_CMP = 8'h00;
	localparam S_CMP_2 = 8'h10;
	localparam S_ADD = 8'h01;
	localparam S_SUB = 8'h02;
	localparam S_JNZ = 8'h03;
	localparam S_PUSH = 8'h04;
	localparam S_POP = 8'h05;
	localparam S_ST = 8'h06;
	localparam S_ST_2 = 8'h16;
	localparam S_ST_SB = 8'h26;
	localparam S_LD = 8'h07;
	localparam S_LD_2 = 8'h17;
	localparam S_OR = 8'h08;
	localparam S_AND = 8'h09;
	localparam S_XOR = 8'h0A;
	localparam S_LSH = 8'h0B;
	localparam S_RSH = 8'h0C;
	localparam S_CLI = 8'h0D;
	localparam S_STI = 8'h0E;
	localparam S_INT = 8'h0F;

	localparam S_FETCH = 8'hf0;
	localparam S_DECODE = 8'hf1;
	localparam S_LOAD = 8'hf2;
	localparam S_STORE = 8'hf3;

	localparam S_IMM = 8'hf2;
	localparam S_REG = 8'hf3;
	localparam S_REGPTR = 8'hf4;

	localparam S_ARITH_2 = 8'hf5;
	localparam S_RESULT = 8'hf7;

	localparam F_ZERO = 1;
	localparam F_NEG = 2;
	localparam F_CARRY = 4;
	localparam F_IE = 8;

	wire [15:0] data_bus;

	/* Internal registers */
	reg i_alu_en = 0;
	reg [2:0] i_alu_op = 0;
	reg [15:0] i_alu_a = 0;
	reg [15:0] i_alu_b = 0;

	reg [15:0] data_bus = 0;

	reg i_mem_we = 0;
	reg [15:0] i_mem_out = 0;
	reg [15:0] i_mem_addr = 0;
	reg [15:0] i_mem_in = 0;

	reg [15:0] i_opcode = 0;

	reg i_single_byte = 0;
	reg i_immediate = 0;
	reg [7:0] i_next_state = 0;
	reg [7:0] i_cur_state = S_FETCH;

	reg [15:0] i_result = 0;

	reg [1:0] i_reg_sel_1 = 0;
	reg [1:0] i_reg_sel_2 = 0;

	reg [1:0] i_reg_sel_mem = 0;

	reg [15:0] i_r1 = 0;
	reg [15:0] i_r2 = 0;

	reg [3:0] i_instr = 0;

	reg i_data_we = 0;

	/* CPU registers */
	reg [15:0] r_a = 0;
	reg [15:0] r_b = 0;
	reg [15:0] r_c = 0;
	reg [15:0] r_d = 0;

	reg [15:0] r_ip = 0;
	reg [15:0] r_sp = 0;
	reg [15:0] r_flags = 0;

	assign mem_we = i_mem_we;
	assign mem_en = 1;
	assign mem_addr = i_mem_addr;
	assign mem_out = i_mem_out;

	/* Internal hardware */

	alu alu_inst(
		.clk(clk),
		.rstn(rstn),
		.en(i_alu_en),
		.a(i_alu_a),
		.b(i_alu_b),
		.op(i_alu_op),
		.out(data_bus)
	);

	always @(posedge clk) begin
		i_mem_in <= mem_in;

		if (!rstn) begin
			i_mem_addr <= 16'h0000;
			i_cur_state <= S_FETCH;
		end else begin
			case(i_cur_state)
			S_FETCH: begin
				/* Read from IP */
				i_data_we <= 0;
				i_mem_we <= 0;

				i_single_byte <= 0;

				i_mem_addr <= r_ip;

				/* Advance IP */
				i_alu_en <= 1;
				i_alu_op <= ALU_ADD;
				i_alu_a <= r_ip;
				i_alu_b <= 16'h2;

				i_cur_state <= S_DECODE;
			end
			S_DECODE: begin
				/* read from ALU */
				
				r_ip <= data_bus;

				/* read from memory */
				i_opcode <= i_mem_in;

				i_instr <= i_mem_in[15:12];
				i_reg_sel_1 <= i_mem_in[7:6];
				i_reg_sel_2 <= i_mem_in[5:4];
				i_immediate <= i_mem_in[3];
				i_single_byte <= i_mem_in[2];

				i_mem_addr <= r_ip;

				case (i_reg_sel_1)
				2'b00: i_r1 <= r_a;
				2'b01: i_r1 <= r_b;
				2'b10: i_r1 <= r_c;
				2'b11: i_r1 <= r_d;
				endcase

				case (i_reg_sel_2)
				2'b00: i_r2 <= r_a;
				2'b01: i_r2 <= r_b;
				2'b10: i_r2 <= r_c;
				2'b11: i_r2 <= r_d;
				endcase

				i_cur_state <= i_instr;
			end
			S_CMP: begin
				
				r_ip <= data_bus;
				i_alu_en <= 1;
				i_alu_op <= ALU_SUB;
				i_alu_a <= i_r1;
				i_alu_b <= i_r2;

				i_cur_state <= S_CMP_2;
			end
			S_CMP_2: begin
				i_result <= data_bus;
				i_alu_en <= 0;

				i_cur_state <= S_RESULT;
			end
			S_ADD: begin
				i_alu_en <= 1;
				i_alu_op <= ALU_ADD;
				i_alu_a <= i_r1;
				i_alu_b <= i_r2;

				i_next_state <= S_ARITH_2;
			end
			S_SUB: begin
				i_alu_en <= 1;
				i_alu_op <= ALU_SUB;
				i_alu_a <= i_r1;
				i_alu_b <= i_r2;

				i_next_state <= S_ARITH_2;
			end
			S_AND: begin
				i_alu_en <= 1;
				i_alu_op <= ALU_AND;
				i_alu_a <= i_r1;
				i_alu_b <= i_r2;

				i_next_state <= S_ARITH_2;
			end
			S_OR: begin
				i_alu_en <= 1;
				i_alu_op <= ALU_OR;
				i_alu_a <= i_r1;
				i_alu_b <= i_r2;

				i_next_state <= S_ARITH_2;
			end
			S_XOR: begin
				i_alu_en <= 1;
				i_alu_op <= ALU_XOR;
				i_alu_a <= i_r1;
				i_alu_b <= i_r2;

				i_next_state <= S_ARITH_2;
			end
			S_LSH: begin
				i_alu_en <= 1;
				i_alu_op <= ALU_LSH;
				i_alu_a <= i_r1;
				i_alu_b <= i_r2;

				i_next_state <= S_ARITH_2;
			end
			S_RSH: begin
				i_alu_en <= 1;
				i_alu_op <= ALU_RSH;
				i_alu_a <= i_r1;
				i_alu_b <= i_r2;

				i_next_state <= S_ARITH_2;
			end
			S_ARITH_2: begin
				case (i_reg_sel_1)
				2'b00: r_a <= data_bus;
				2'b01: r_b <= data_bus;
				2'b10: r_c <= data_bus;
				2'b11: r_d <= data_bus;
				endcase

				i_result <= data_bus;
				i_alu_en <= 0;
			end
			S_JNZ: begin
			end
			S_ST: begin
				i_alu_en <= 1;
				i_alu_a <= r_ip;
				i_alu_op <= ALU_ADD;
				i_mem_addr <= r_ip;

				if (i_immediate) begin
					i_alu_b <= 16'h2;
				end else begin
					i_alu_b <= 16'h0;
				end

				i_mem_we <= 0;
				i_cur_state <= S_ST_2;
			end
			S_ST_2: begin
				
				r_ip <= data_bus;
				i_alu_en <= 0;

				if (i_immediate) begin
					i_mem_addr <= i_mem_in;
					i_r1 <= i_mem_in;
				end else begin
					i_mem_addr <= i_r1;
				end

				if (i_single_byte) begin
					i_mem_we <= 0;
					i_cur_state <= S_ST_SB;
				end else begin
					i_mem_we <= 1;
					i_mem_out <= i_r2;
					i_result <= i_r2;

					i_cur_state <= S_RESULT;
				end

			end
			S_ST_SB: begin
				i_mem_addr <= i_r1;
				i_mem_we <= 1;

				i_mem_out <= { i_mem_in[7:0], i_r2[7:0] };
				i_result <= { 8'h0, i_r2[7:0] };
				i_cur_state <= S_RESULT;
			end
			S_LD: begin
				i_alu_en <= 1;
				i_alu_op <= ALU_ADD;
				i_alu_a <= r_ip;

				if (i_immediate) begin
					i_mem_addr <= r_ip;
					i_alu_b <= 16'h2;
				end else begin
					i_mem_addr <= i_r1;
					i_alu_b <= 16'h0;
				end

				i_cur_state <= S_LD_2;
			end
			S_LD_2: begin
				
				r_ip <= data_bus;
				i_alu_en <= 0;

				if(i_single_byte) begin
					case(i_reg_sel_1)
					2'b00: r_a <= { r_a[15:8], i_mem_in[7:0] };
					2'b01: r_b <= { r_b[15:8], i_mem_in[7:0] };
					2'b10: r_c <= { r_c[15:8], i_mem_in[7:0] };
					2'b11: r_d <= { r_d[15:8], i_mem_in[7:0] };
					endcase
				end else begin
					case(i_reg_sel_1)
					2'b00: r_a <= i_mem_in;
					2'b01: r_b <= i_mem_in;
					2'b10: r_c <= i_mem_in;
					2'b11: r_d <= i_mem_in;
					endcase
				end

				i_result <= i_mem_in;
				i_cur_state <= S_RESULT;
			end
			S_CLI: begin
				r_flags <= r_flags & ~F_IE;
				i_result <= r_flags;
				i_cur_state <= S_RESULT;
			end
			S_STI: begin
				r_flags <= r_flags | F_IE;
				i_result <= r_flags;
				i_cur_state <= S_RESULT;
			end
			S_INT: begin
				/* behavior still not defined!! */
			end
			S_RESULT: begin
				i_mem_we <= 0;
				i_alu_en <= 0;

				r_flags <= r_flags ^ (r_flags & (~F_IE));

				if (i_result[15]) begin
					r_flags <= r_flags | F_NEG;
				end else if(i_result == 0) begin
					r_flags <= r_flags | F_ZERO;
				end

				i_cur_state <= S_FETCH;
			end
			endcase
		end
	end
endmodule

module alu(
	input wire clk,
	input wire rstn,
	input wire en,
	input wire [15:0] a,
	input wire [15:0] b,
	input wire [2:0] op,
	output wire [15:0] out
	);

	localparam OP_ADD = 3'b000;
	localparam OP_SUB = 3'b001;
	localparam OP_AND = 3'b010;
	localparam OP_OR = 3'b011;
	localparam OP_XOR = 3'b100;
	localparam OP_LSH = 3'b101;
	localparam OP_RSH = 3'b110;

	reg [15:0] result = 0;
	reg [15:0] i_a = 0;
	reg [15:0] i_b = 0;
	reg [2:0] i_op = 0;

	assign out = result;

	always @(*) begin
		if (!rstn) begin
			result <= 16'b0;
		end

		i_a <= a;
		i_b <= b;
		i_op <= op;

		case(i_op)
		OP_ADD: result <= i_a + i_b;
		OP_SUB: result <= i_a - i_b;
		OP_AND: result <= i_a & i_b;
		OP_OR: result <= i_a | i_b;
		OP_XOR: result <= i_a ^ i_b;
		OP_LSH: result <= i_a << i_b[3:0];
		OP_RSH: result <= i_a >> i_b[3:0];
		default: result <= 16'b0;
		endcase
	end
endmodule
