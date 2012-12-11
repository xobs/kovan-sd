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

module kovan (
		// power management
		input wire        CHG_ACP, // reports presence of AC power

		// "HDMI" -- now connected to SD
		input wire        DDC_SDA_LV_N,
		output wire       DDC_SDA_PU,
		output wire       DDC_SDA_PD,
		input wire        DDC_SCL_LV_N,
		output wire       HPD_NOTIFY,
		output wire       HPD_OVERRIDE,

		input wire        UK9_BUF_T,
		input wire        SD_CD,

		output wire       SD_CS_T,
		output wire       SD_DI_T,
		output wire       SD_SCLK_T,
		output wire       SD_TURNON_T,
		input wire        UK7_BUF_T,
		output wire       SD_DAT2_T,
		output wire       SD_DAT1_T,
		input wire        SD_DO_T,

		output wire [7:0] CAM_D,
		input wire        CAM_HSYNC,
		input wire        CAM_MCLKO,
		input wire        CAM_VCLKI,
		input wire        CAM_VCLKO,
		input wire        CAM_VSYNC,

		// i/o controller digital interfaces
		output wire [1:0] DIG_ADC_CS,
		output wire       DIG_ADC_IN,
		input wire        DIG_ADC_OUT,
		output wire       DIG_ADC_SCLK,
		output wire       DIG_ADC_CLR,
		output wire       DIG_IN,
		input wire        DIG_OUT,
		output wire       DIG_RCLK,
		output wire       DIG_SAMPLE,
		output wire       DIG_SCLK,
		output wire       DIG_SRLOAD,
		output wire       DIG_CLR_N,

		// motor direct drive interfaces
		output wire [3:0] MBOT,
		output wire [3:0] MTOP,
		output wire       MOT_EN,
		output wire [3:0] M_SERVO,

		// optional uart to outside world
		input wire        EXT_TO_HOST_UART, // Not currently used
		input wire        HOST_TO_EXT_UART,

		// infrared receiver 
		input wire        IR_RX,

		// switch
		input wire        INPUT_SW0,

		// audio pass-through
		input wire        I2S_CDCLK0, // master reference clock to audi
		output wire       I2S_CDCLK1,
		output wire       I2S_CLK0, // return sample clock to CPU
		input wire        I2S_CLK1,
		input wire        I2S_DI0, // audio data to playback
		output wire       I2S_DI1,
		output wire       I2S_DO0, // audio data from record
		input wire        I2S_DO1,
		output wire       I2S_LRCLK0, // left/right clock to codec
		input wire        I2S_LRCLK1,

		// "LCD" output to SD card
		input wire        DA,
		input wire        DB,
		input wire        DC,
		input wire        DD,
		input wire        DE,
		input wire        DF,
		input wire        DG,
		input wire        DH,

		input wire        UK0,
		input wire        UK1,
		input wire        UK2,
		input wire        UK3,
		input wire        UK4,
		input wire        UK5,
		input wire        UK6,
		/* UK7 wired elsewhere */
		input wire        UK8,
		/* UK9 wired elsewhere */

		input wire        ALE,
		input wire        CLE,
		input wire        WE,
		input wire        CS,
		input wire        RE,
		input wire        RB,

		// LCD input from CPU
		output wire [5:0] LCD_B, // note no truncation of data in
		output wire [5:0] LCD_G,
		output wire [5:0] LCD_R,
		input wire        LCD_DEN,
		input wire        LCD_HS,
		output wire [5:0] LCD_SUPP,
		input wire        LCD_VS,
		output wire       LCD_CLK_T, // clock is sourced from the FPGA
		// for forward compatibility with HDMI-synced streams

		// SSP interface to the CPU
		output wire       FPGA_MISO,
		input wire        FPGA_MOSI,
		input wire        FPGA_SCLK,
		input wire        FPGA_SYNC,

		// I2C interfaces
		input wire        PWR_SCL, // we listen on this one
		inout wire        PWR_SDA,

		input wire        XI2CSCL, // our primary interface
		inout wire        XI2CSDA,

		// LED
		output wire       FPGA_LED,

		input wire        OSC_CLK   // 26 mhz clock from CPU
	);

	/* Clock buffers */
	wire      clk26;
	wire      clk26ibuf;
 
	assign clk26 = OSC_CLK;
	IBUFG clk26buf_ibuf(.I(clk26), .O(clk26ibuf));

	/* This set of wires comes out of the FIFO, and feeds into a mux */
	wire [63:0]  mem_output;


	/* Wires and pins on the SD card side */
	wire [7:0]    NAND_D;
	wire [9:0]    NAND_UK;
	wire          NAND_ALE;
	wire          NAND_CLE;
	wire          NAND_WE;
	wire          NAND_CS;
	wire          NAND_RE;
	wire          NAND_RB;

	wire          NAND_TYPE0;
	wire          NAND_TYPE1;
	wire          NAND_RW;

	/* Wires and pins on the CPU side */
	wire          SD_CLK_CPU;
	wire          SD_MOSI_CPU;
	wire          SD_CS_CPU;
	wire          SD_TURNON_CPU;

	/* Bog-standard blinky LED counter */
	reg  [32:0]   free_timer;


	/* Convenience renaming of signals (mapping from tap board names to
	 * informative meanings)
	 */
	assign NAND_D[7:0] = {DH, DG, DF, DE, DD, DC, DB, DA};
	assign NAND_UK[9:0] = {UK9_BUF_T, UK8, UK7_BUF_T, UK6, UK5, UK4, UK3, UK2, UK1, UK0};
	assign NAND_ALE = ALE;
	assign NAND_CLE = CLE;
	assign NAND_WE = WE;
	assign NAND_CS = CS;
	assign NAND_RE = RE;
	assign NAND_RB = RB;


	/* Assign nice names to the SD input pins */
	assign SD_CLK_CPU = CAM_VSYNC;
	assign SD_MOSI_CPU = CAM_HSYNC;
	assign SD_CS_CPU = CAM_VCLKO;
	assign SD_TURNON_CPU = CAM_VCLKI;

	/* Wire up outputs from the CPU directly to SD pins */
	assign SD_SCLK_T = SD_CLK_CPU;
	assign SD_DI_T = SD_MOSI_CPU;
	assign SD_CS_T = SD_CS_CPU;
	assign SD_TURNON_T = SD_TURNON_CPU;

	/* This is a special case.  It's miswired to USB_OTG_TYPE */
	assign CAM_D[5] = 1;

	/* These outputs are wired to vestigial hardware, and are unused */
	assign DIG_SCLK = 1'b0;
	assign DIG_ADC_CS = 1'b0;
	assign DIG_ADC_SCLK = 1'b0;
	assign MBOT = 1'b0;
	assign MTOP = 1'b0;
	assign M_SERVO = 1'b0;
	assign DIG_RCLK = 1'b0;
	assign MOT_EN = 1'b0;
	assign DIG_CLR_N = 1'b0;
	assign DIG_SAMPLE = 1'b0;
	assign DIG_SRLOAD = 1'b0;
	assign DIG_IN = 1'b0;
	assign DIG_ADC_IN = 1'b0;

	/* These are currently unused, but may be used in the future */
	assign SD_DAT1_T = 1'b0;
	assign SD_DAT2_T = 1'b0;
	assign FPGA_MISO = 1'b0;
	assign I2S_DI1 = 1'b0;
	assign DDC_SDA_PD = 1'b0;
	assign HPD_OVERRIDE = 1'b0;
	assign I2S_CDCLK1 = 1'b0;
	assign I2S_DO0 = 1'b0;
	assign I2S_LRCLK0 = 1'b0;
	assign DDC_SDA_PU = 1'b0;
	assign I2S_CLK0 = 1'b0;
	assign LCD_CLK_T = 1'b0;



	/* Multiply the incoming 26 MHz clock up to 125 MHz so we can build our
	 * own edge detector.  We want to trigger an event on a rising edge of
	 * NAND_WE or the falling edge of NAND_RE.
	 */
	wire clk125;
	fast_clock fast_clock(
		.CLK_IN1(clk26ibuf),
		.CLK_OUT1(clk125)
	);


	/* Master chunk of BRAM.  When a sample is taken, it's stored here. */
	reg  [63:0]  mem_input;
	wire         is_full;
	wire         is_empty;

	reg          do_write;
	wire         do_read;
	wire         reset_clock;

	/* These allow us to do bank selection for the output register value */
	wire [1:0]   output_bank;
	assign output_bank[0] = LCD_HS;
	assign output_bank[1] = LCD_VS;
	assign reset_clock = LCD_DEN;

	/* The actual output pins that go from the mux and feed to the CPU */
	reg [15:0]   output_reg;
	assign CAM_D[0]   = output_reg[0];
	assign CAM_D[1]   = output_reg[1];
	assign CAM_D[2]   = output_reg[2];
	assign CAM_D[3]   = output_reg[3];
	assign CAM_D[4]   = output_reg[4];
	assign LCD_G[2]   = output_reg[5];  // Pin 5 is miswired
	assign CAM_D[6]   = output_reg[6];
	assign CAM_D[7]   = output_reg[7];
	assign LCD_R[3]   = output_reg[8];
	assign LCD_R[4]   = output_reg[9];
	assign LCD_R[5]   = output_reg[10];
	assign LCD_G[0]   = output_reg[11];
	assign LCD_G[1]   = output_reg[12];
	assign LCD_G[3]   = output_reg[13];
	assign LCD_G[4]   = output_reg[14];
	assign LCD_G[5]   = output_reg[15];
	assign LCD_B[0]   = free_timer[32]; // Timer overflow bit
	assign LCD_B[5:1] = 0;
	assign LCD_SUPP   = 0;

	/* Mux the output values */
	always @(mem_output or output_bank) begin
		if (output_bank == 2'b00) begin
			output_reg[15:0] <= mem_output[15:0];

		end else if (output_bank == 2'b01) begin
			output_reg[15:0] <= mem_output[31:16];

		end else if (output_bank == 2'b10) begin
			output_reg[15:0] <= mem_output[47:32];

		end else if (output_bank == 2'b11) begin
			output_reg[15:0] <= mem_output[63:48];
		end
	end


	assign do_read = CAM_MCLKO;
	assign LCD_R[2] = SD_DO_T;
	assign LCD_R[1] = !is_empty;
	assign LCD_R[0] = is_full;


	/* Value used to determine if a new sample should be read */
	reg get_new_sample;

	fifo fifo(
		.wr_clk(clk125),
		.rd_clk(clk125),
		.rst(1'b0),
		.din(mem_input),
		.wr_en(do_write),
		.rd_en(get_new_sample),
		.dout(mem_output),
		.full(is_full),
		.empty(is_empty)
	);



	/* Everything happens in relation to this 125 MHz clock */
	assign FPGA_LED = is_empty;

	reg previous_nand_we, previous_nand_re;
	reg previous_do_read;
	reg previous_sd_clk;

	reg [7:0] sd_accumulator;
	reg [3:0] sd_accumulator_ptr;
	reg [7:0] sd_register_number;

	always @(posedge clk125) begin

		/* Always tick the clock (or reset it) */
		if (reset_clock) begin
			free_timer <= 0;
		end
		else begin
			free_timer <= free_timer+1;
		end

		/* Compare the NAND read/write pins to determine if we have to
		 * capture a sample and put it in the buffer
		 */
		if ((!previous_nand_we &&  NAND_WE)
		  || (previous_nand_re && !NAND_RE)) begin
			mem_input[31:0]  <= free_timer;
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
			do_write         <= 1;
		end
		else begin
			do_write <= 0;
		end

		/* If the do_read wire has gone high, queue a read */
		if (!previous_do_read && do_read) begin
			get_new_sample <= 1;
		end
		else begin
			get_new_sample <= 0;
		end

		previous_nand_we <= NAND_WE;
		previous_nand_re <= NAND_RE;
		previous_do_read <= do_read;
	end

endmodule // kovan
