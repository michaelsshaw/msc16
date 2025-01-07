/* SPDX-License-Identifier: GPL-2.0-or-later */

/* Memory controller interface for the MSC-16 CPU
 *
 * This module interfaces with the BRAM generator
 */
`timescale 1ns / 1ps
module memctl(
	input wire clk,
	input wire rstn,
	input wire we,
	input wire en,
	input wire [15:0] addr,
	input wire [15:0] data_in,
	output wire [15:0] data_out,

	output wire [31:0] bram_addr,
	input wire [31:0] bram_data_in,
	output wire [31:0] bram_data_out,
	output wire bram_en,
	output wire [3:0] bram_we
	);

	reg [31:0] bram_data_t;
	reg [31:0] bram_data_wr;

	assign bram_data_out = bram_data_wr;
	assign bram_addr[15:0] = addr[15:0];
	assign bram_en = en;
	assign bram_we = we ? 4'b1100 : 4'b0;

	assign data_out = en ? bram_data_t[15:0] : 16'bz;


	always @(posedge clk) begin
		if (!rstn) begin
			bram_data_t <= 32'b0;
			bram_data_wr = 32'b0;
		end else begin
			bram_data_t <= { bram_data_in[7:0], bram_data_in[15:8] };
			bram_data_wr <= { data_in[7:0], data_in[15:8], 16'b0 };
		end
	end
endmodule
