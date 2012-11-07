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
		input wire 	      CLE,
		input wire 	      WE,
		input wire 	      CS,
		input wire 	      RE,
		input wire 	      RB,

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

	/* This chunk of memory comes out of the FIFO, and feeds directly into
	 * the output pins.
	 */
	wire [63:0]  mem_output;


	/* Wires and pins on the SD card side */
	wire [7:0] 	  NAND_D;
	wire [9:0] 	  NAND_UK;
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
	reg  [25:0]   LED_COUNTER;
	wire [25:0]   LED_COUNTER_OUT;

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


	/* Wire up outputs from the CPU directly to SD pins */
	assign SD_SCLK_T = CAM_VSYNC;
	assign SD_DI_T = CAM_HSYNC;
	assign SD_CS_T = CAM_VCLKO;
	assign SD_TURNON_T = CAM_VCLKI;

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



	/* Multiply the incoming 26 MHz clock up to 251 MHz so we can build our
	 * own edge detector.  We want to trigger an event on a rising edge of
	 * NAND_WE or the falling edge of NAND_RE.
	 */
	wire clk251;
	fast_clock fast_clock(
		.CLK_IN1(clk26ibuf),
		.CLK_OUT1(clk251)
	);


   /* Master chunk of BRAM.  When a sample is taken, it's stored here. */
	wire [63:0]  mem_input;
	wire         is_full;
	wire         is_empty;
	wire         data_is_valid;

	reg          do_write;
	wire         do_read;

	assign mem_input[7:0] = NAND_D[7:0];
	assign mem_input[8] = NAND_ALE;
	assign mem_input[9] = NAND_CLE;
	assign mem_input[10] = NAND_CS;
	assign mem_input[11] = NAND_WE;
	assign mem_input[12] = NAND_RE;
	assign mem_input[13] = NAND_RB;
	assign mem_input[23:14] = NAND_UK[9:0];
	assign mem_input[49:24] = LED_COUNTER;

	assign CAM_D = mem_output[7:0];
	assign LCD_R[3] = mem_output[8];
	assign LCD_R[4] = mem_output[9];
	assign LCD_R[5] = mem_output[10];
	assign LCD_G[0] = mem_output[11];
	assign LCD_G[1] = mem_output[12];
	assign LCD_G[5:2] = mem_output[27:24];
	assign LCD_B = mem_output[33:28];
	assign LCD_SUPP = mem_output[39:34];

	assign do_read = CAM_MCLKO;
	assign LCD_R[2] = SD_DO_T;
	assign LCD_R[1] = !is_empty;
	assign LCD_R[0] = data_is_valid;

	fifo fifo(
		.clk(clk251),
		.rst(1'b0),
		.din(mem_input),
		.wr_en(do_write),
		.rd_en(do_read),
		.dout(mem_output),
		.full(is_full),
		.empty(is_empty),
		.valid(data_is_valid)
	);



	/* Everything happens in relation to this 251 MHz clock */
	assign FPGA_LED = LED_COUNTER[25];
//	assign FPGA_LED = CAM_VCLKI;
//	assign SD_TURNON_T = LED_COUNTER[25];

	reg previous_nand_we, previous_nand_re;

	always @(posedge clk251) begin
		LED_COUNTER <= LED_COUNTER+1;
		if ((!previous_nand_we && NAND_WE)
		 || (previous_nand_re && !NAND_RE)) begin
			do_write = 1;
		end else begin
			do_write = 0;
		end

		previous_nand_we = NAND_WE;
		previous_nand_re = NAND_RE;
	end
endmodule // kovan
