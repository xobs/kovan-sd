////////////////////////////////////////////////
// Copyright (c) 2012, Andrew "bunnie" Huang  
// (bunnie _aht_ bunniestudios "dote" com)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//     Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in
//     the documentation and/or other materials provided with the
//     distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////

`timescale 1 ns / 1 ps

module nand_fifo (
		input wire		CLK,

		/* Tap Board SD connections */
		output wire		TB_SD_CS,
		output wire		TB_SD_DI,
		output wire		TB_SD_CLK,
		output wire		TB_SD_TURNON,
		input wire		TB_SD_DO,
		output wire		TB_SD_DAT1,
		output wire		TB_SD_DAT2,

		/* Application Processor SD connections */
		input wire		AP_SD_CS,
		input wire		AP_SD_DI,
		input wire		AP_SD_CLK,
		input wire		AP_SD_TURNON,
		output wire		AP_SD_DO,
		input wire		AP_SD_DAT1,
		input wire		AP_SD_DAT2,

		/* 16-bit bus from FPGA to Application Processor */
		output wire [15:0]	AP_BUS,
		output wire		AP_BUS_EMPTY,
		output wire		AP_BUS_FULL,
		input wire  [1:0]	AP_BUS_BANK,
		input wire		AP_BUS_READ,
		input wire		AP_BUS_RESET,
		input wire		AP_BUS_PAUSE,
		input wire  [31:0]	AP_BLOCK_SKIP,

		/* Wires and pins on the SD card side */
		input wire [7:0]	NAND_D,
		input wire [9:0]	NAND_UK,
		input wire		NAND_ALE,
		input wire		NAND_CLE,
		input wire		NAND_WE,
		input wire		NAND_CS,
		input wire		NAND_RE,
		input wire		NAND_RB,

		/* Miscellaneous outputs */
		output wire [31:0]	FREE_COUNTER,
		output wire		FREE_COUNTER_OVERFLOW,
		output wire [12:0]	BUFFER_READ_LEVEL,
		output wire [12:0]	BUFFER_WRITE_LEVEL,
		output wire [3:0]	DIAGNOSTICS
	);

	assign TB_SD_CS		= AP_SD_CS;
	assign TB_SD_DI		= AP_SD_DI;
	assign TB_SD_CLK	= AP_SD_CLK;
	assign TB_SD_TURNON	= AP_SD_TURNON;
	assign TB_SD_DAT1	= AP_SD_DAT1;
	assign TB_SD_DAT2	= AP_SD_DAT2;
	assign AP_SD_DO		= TB_SD_DO;
 

	/* This set of wires comes out of the FIFO, and feeds into a mux */
	wire [63:0]	mem_output;


	/* Standard FPGA counter */
	reg  [32:0]	free_counter;

	/* Used as part of a rising-edge pulse-generator */
	reg		do_write;
	reg		did_read;
	reg		do_read;
	reg		do_read_s1;
	reg		do_read_s2;
	reg		do_read_buf_last;
 
	/* Master chunk of BRAM.  When a sample is taken, it's stored here. */
	reg  [63:0]	mem_input;
	reg  [63:0]	mem_input_d; /* Buffer one sample back in time */
	reg		we_buf1, we_buf2, we_buf3;
	reg		re_buf1, re_buf2, re_buf3, re_buf4;
	reg		ale_buf1, ale_buf2, ale_buf3;
	reg		cle_buf1, cle_buf2, cle_buf3;

	reg  [31:0]	byte_counter;
	reg  [31:0]	block_skip;
	reg  [31:0]	block_skip_target;

	reg		fifo_drain_state;
	reg		fifo_has_drained;

	reg  [15:0]	bus_output;



	assign FREE_COUNTER		= free_counter[31:0];
	assign FREE_COUNTER_OVERFLOW	= free_counter[32];

	assign AP_BUS			= bus_output;



	/* Mux the output values */
	always @(mem_output or AP_BUS_BANK) begin
		if (AP_BUS_BANK == 2'b00) begin
			bus_output[15:0] <= mem_output[15:0];

		end else if (AP_BUS_BANK == 2'b01) begin
			bus_output[15:0] <= mem_output[31:16];

		end else if (AP_BUS_BANK == 2'b10) begin
			bus_output[15:0] <= mem_output[47:32];

		end else if (AP_BUS_BANK == 2'b11) begin
			bus_output[15:0] <= mem_output[63:48];
		end
	end


	/* Captured samples temporarily get stored here */
	fifo sample_buffer(
		.rst(1'b0),

		.wr_en(do_write),
		.wr_clk(CLK),

		.rd_en(do_read),
		.rd_clk(CLK),

		.wr_data_count(BUFFER_WRITE_LEVEL),
		.rd_data_count(BUFFER_READ_LEVEL),

		.din(mem_input_d),
		.dout(mem_output),

		.full(AP_BUS_FULL),
		.empty(AP_BUS_EMPTY)
	);


	/* Once the buffer has filled up completely, let it drain first */
	always @(posedge CLK) begin

		/* "Filling" state.  Wait until AP_BUS_FULL is set. */
		if (fifo_drain_state == 1'b0) begin
			fifo_has_drained <= 1'b1;
			if (AP_BUS_FULL)
				fifo_drain_state <= 1'b1;
		end

		/* "Draining" state.  Wait until AP_BUS_EMPTY is set. */
		else if (fifo_drain_state == 1'b1) begin
			fifo_has_drained <= 1'b0;
			if (AP_BUS_EMPTY)
				fifo_drain_state <= 1'b0;
		end

		else begin
			fifo_drain_state <= 1'b0;
			fifo_has_drained <= 1'b0;
		end
	end


	always @(posedge CLK) begin
		do_read_s1 <= AP_BUS_READ;
		do_read_s2 <= do_read_s1;

		/* Three-state machine to do reads */
		if (do_read_s2 == do_read_buf_last) begin
			did_read <= 0;
			do_read <= 0;
			do_read_buf_last <= do_read_s2;
		end
		else if (!did_read) begin
			did_read <= 1;
			do_read <= 1;
			do_read_buf_last <= do_read_s2;
		end else begin
			did_read <= 1;
			do_read <= 0;
			do_read_buf_last <= do_read_s2;
		end
	end

	/* Promary edge detector loop */
	always @(posedge CLK) begin

		/* Pipeline the data one deep so we can 'reach back in time' */
		mem_input[31:0]  <= free_counter;
		mem_input[35:32] <= 4'b0000;
		mem_input[43:36] <= NAND_D[7:0];
		mem_input[44]    <= NAND_ALE;
		mem_input[45]    <= NAND_CLE;
		mem_input[46]    <= NAND_WE;
		mem_input[47]    <= NAND_RE;
		mem_input[48]    <= NAND_CS;
		mem_input[49]    <= NAND_RB;
		mem_input[59:50] <= NAND_UK[9:0];
		mem_input[63:60] <= 0;

		we_buf1		<= NAND_WE;
		we_buf2		<= we_buf1;
		we_buf3		<= we_buf2;

		re_buf1		<= NAND_RE;
		re_buf2		<= re_buf1;
		re_buf3		<= re_buf2;
		re_buf4 	<= re_buf3;

		ale_buf1	<= NAND_ALE;
		ale_buf2	<= ale_buf1;
		ale_buf3	<= ale_buf2;

		cle_buf1	<= NAND_CLE;
		cle_buf2	<= cle_buf1;
		cle_buf3	<= cle_buf2;


		/* Always tick the clock (or reset it) */
		if (AP_BUS_RESET) begin
			free_counter <= 0;
			do_write <= 0;
			byte_counter <= 0;
			mem_input_d[63:0]  <= 0;
			block_skip <= block_skip;
		end
		else begin
			if (((re_buf3) && !re_buf4)
			|| (((we_buf2) && !we_buf3)) ) begin
				if (we_buf2 && !we_buf3) begin
					// Grab from 'reach back' so
					// *before* edge
					mem_input_d[45:0]  <= mem_input[45:0];
					mem_input_d[46]    <= 1'b1; // WE
					mem_input_d[47]    <= 1'b0; // RE
					mem_input_d[63:48] <= mem_input[63:48];
				end

				// grab two cycles after falling edge,
				// to give time for NAND to produce data
				else if(re_buf3 && !re_buf4) begin
					mem_input_d[31:0]  <= free_counter;
					mem_input_d[35:32] <= 4'b0000;
					mem_input_d[43:36] <= NAND_D[7:0];
					mem_input_d[44]    <= NAND_ALE;
					mem_input_d[45]    <= NAND_CLE;
					mem_input_d[46]    <= 1'b0; // WE
					mem_input_d[47]    <= 1'b1; // RE
					mem_input_d[48]    <= NAND_CS;
					mem_input_d[49]    <= NAND_RB;
					mem_input_d[59:50] <= NAND_UK[9:0];
					mem_input_d[63:60] <= 0;
				end
				else begin
					mem_input_d[63:48] <=mem_input_d[63:48];
					mem_input_d[47]    <= 1'b0;
					mem_input_d[46]    <= 1'b0;
					mem_input_d[45:0]  <= mem_input_d[45:0];
				end

				if (block_skip > 0) begin
					block_skip <= block_skip-1;
					do_write <= 0;
				end
				else begin
					block_skip <= 0;
					do_write <= 1
						  & fifo_has_drained
						  & (!AP_BUS_PAUSE);
				end
				byte_counter <= byte_counter+1;
				block_skip_target <= block_skip_target;
			end
			else begin
				do_write		<= 0;
				byte_counter		<= byte_counter;
				mem_input_d[63:48]	<= mem_input_d[63:48];
				mem_input_d[47]		<= 1'b0;
				mem_input_d[46]		<= 1'b0;
				mem_input_d[45:0]	<= mem_input_d[45:0];

				if (AP_BLOCK_SKIP != block_skip_target) begin
					block_skip        <= AP_BLOCK_SKIP;
					block_skip_target <= AP_BLOCK_SKIP;
				end
				else begin
					block_skip <= block_skip;
					block_skip_target <= block_skip_target;
				end
			end
			free_counter <= free_counter+1;
		end
	end

	/* Debug */
	assign DIAGNOSTICS[0] = 1'b0;
	assign DIAGNOSTICS[1] = free_counter[3];
	assign DIAGNOSTICS[2] = 1'b0;
	assign DIAGNOSTICS[3] = 1'b0;

endmodule // kovan
