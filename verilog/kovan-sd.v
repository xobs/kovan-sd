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
	wire		clk26;
	wire		clk26ibuf;
	wire		clk26buf;
	wire		clk125;
 

	/* This set of wires comes out of the FIFO, and feeds into a mux */
	wire [63:0]	mem_output;


	/* Wires and pins on the SD card side */
	wire [7:0]	nand_d;
	wire [9:0]	nand_uk;
	wire		nand_ale;
	wire		nand_cle;
	wire		nand_we;
	wire		nand_cs;
	wire		nand_re;
	wire		nand_rb;

	wire		ap_bus_empty;
	wire		ap_bus_full;
	wire		ap_bus_read;
	wire		ap_bus_reset;

	wire		ap_sd_cs;
	wire		ap_sd_di;
	wire		ap_sd_clk;
	wire		ap_sd_turnon;
	wire		ap_sd_do;
	wire		ap_sd_dat1;
	wire		ap_sd_dat2;

	/* Standard blinky LED counter */
	wire [12:0]	wr_data_count;
	wire [12:0]	rd_data_count;

	wire [1:0]    output_bank;
	wire [15:0]   output_reg;

	wire          SDA_pd;
	wire          SDA_int;

	reg           previous_nand_we;
	reg           previous_nand_re;
	reg           previous_previous_nand_re;

	reg  [31:0]   byte_counter;
	reg  [31:0]   block_skip;
	reg  [31:0]   block_skip_target;
	wire [31:0]   block_skip_input;

	/* Convenience renaming of signals (mapping from tap board names to
	 * informative meanings)
	 */
	assign nand_d[7:0] = {DH, DG, DF, DE, DD, DC, DB, DA};
	assign nand_uk[9:0] = {UK9_BUF_T, UK8, UK7_BUF_T, UK6, UK5, UK4, UK3, UK2, UK1, UK0};
	assign nand_ale = ALE;
	assign nand_cle = CLE;
	assign nand_we = WE;
	assign nand_cs = CS;
	assign nand_re = RE;
	assign nand_rb = RB;


	/* This is a special case.  It's miswired to USB_OTG_TYPE */
	assign CAM_D[5] = 1'b1;

	/* These outputs are wired to vestigial hardware, and are unused */
	assign MBOT = 1'b0;
	assign MTOP = 1'b0;
	assign MOT_EN = 1'b0;
	assign DIG_SCLK = 1'b0;
	assign DIG_ADC_CS = 1'b0;
	assign DIG_ADC_SCLK = 1'b0;
	assign DIG_RCLK = 1'b0;
	assign DIG_CLR_N = 1'b0;
	assign DIG_SAMPLE = 1'b0;
	assign DIG_SRLOAD = 1'b0;
	assign DIG_IN = 1'b0;
	assign DIG_ADC_IN = 1'b0;
	assign DDC_SDA_PD = 1'b0;
	assign DDC_SDA_PU = 1'b0;
	assign HPD_OVERRIDE = 1'b0;

	/* These are currently unused, but may be used in the future */
	assign FPGA_MISO = 1'b0;
	assign I2S_DI1 = 1'b0;
	assign I2S_CDCLK1 = 1'b0;
	assign I2S_DO0 = 1'b0;
	assign I2S_LRCLK0 = 1'b0;
	assign I2S_CLK0 = 1'b0;
	assign LCD_CLK_T = 1'b0;



	/* These allow us to do bank selection for the output register value */
	assign output_bank[0] = LCD_HS;
	assign output_bank[1] = LCD_VS;

	/* The actual output pins that go from the mux and feed to the CPU */
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
	assign LCD_B[5:1] = 0;
	assign LCD_SUPP   = 0;



	/* Poor-man's diagnostics */
	wire [3:0] diag;
	assign M_SERVO[0]	= !diag[0];
	assign M_SERVO[1]	= !diag[1];
	assign M_SERVO[2]	= !diag[2];
	assign M_SERVO[3]	= !diag[3];
	assign FPGA_LED		= !diag[2];

	assign LCD_R[1]		= !ap_bus_empty;
	assign LCD_R[0]		= ap_bus_full;
	assign ap_bus_read	= CAM_MCLKO;
	assign ap_bus_reset	= LCD_DEN;

	assign ap_sd_cs		= CAM_VCLKO;
	assign ap_sd_di		= CAM_HSYNC;
	assign ap_sd_clk	= CAM_VSYNC;
	assign ap_sd_turnon	= CAM_VCLKI;

	assign LCD_R[2]		= ap_sd_do;

	assign clk26 = OSC_CLK;
	IBUFG clk26buf_ibuf(.I(clk26), .O(clk26ibuf));
	BUFG clk26buf_buf (.I(clk26ibuf), .O(clk26buf));

	/* Multiply the incoming 26 MHz clock up to 125 MHz so we can build our
	 * own edge detector.  We want to trigger an event on a rising edge of
	 * NAND_WE or the falling edge of NAND_RE.
	 */
	fast_clock fast_clock(
		.CLK_IN1(clk26ibuf),
		.CLK_OUT1(clk125)
	);


	nand_fifo nand_fifo(
		.CLK(clk125),

		/* Tap Board SD connections */
		.TB_SD_CS(SD_CS_T),
		.TB_SD_DI(SD_DI_T),
		.TB_SD_CLK(SD_SCLK_T),
		.TB_SD_TURNON(SD_TURNON_T),
		.TB_SD_DO(SD_DO_T),
		.TB_SD_DAT1(SD_DAT1_T),
		.TB_SD_DAT2(SD_DAT2_T),

		/* Application Processor SD connections */
		.AP_SD_CS(ap_sd_cs),
		.AP_SD_DI(ap_sd_di),
		.AP_SD_CLK(ap_sd_clk),
		.AP_SD_TURNON(ap_sd_turnon),
		.AP_SD_DO(ap_sd_do),
		.AP_SD_DAT1(ap_sd_dat1),
		.AP_SD_DAT2(ap_sd_dat2),

		/* 16-bit bus from FPGA to Application Processor */
		.AP_BUS(output_reg),
		.AP_BUS_EMPTY(ap_bus_empty),
		.AP_BUS_FULL(ap_bus_full),
		.AP_BUS_BANK(output_bank),
		.AP_BUS_READ(ap_bus_read),
		.AP_BUS_RESET(ap_bus_reset),
		.AP_BUS_PAUSE(pause_writing),
		.AP_BLOCK_SKIP(block_skip_input),

		/* Wires and pins on the SD card side */
		.NAND_D(nand_d),
		.NAND_UK(nand_uk),
		.NAND_ALE(nand_ale),
		.NAND_CLE(nand_cle),
		.NAND_WE(nand_we),
		.NAND_CS(nand_cs),
		.NAND_RE(nand_re),
		.NAND_RB(nand_rb),

		/* Miscellaneous outputs */
		.FREE_COUNTER_OVERFLOW(LCD_B[0]),
		.BUFFER_READ_LEVEL(rd_data_count),
		.BUFFER_WRITE_LEVEL(wr_data_count),
		.DIAGNOSTICS(diag)
	);


	/* I2C driver */
	IOBUF #(.DRIVE(8), .SLEW("SLOW")) IOBUF_sda (
		.IO(XI2CSDA),
		 .I(1'b0),
		 .T(!SDA_pd),
		 .O(SDA_int)
	);
	i2c_slave host_i2c(
		.SCL(XI2CSCL),
		.SDA(SDA_int),
		.SDA_pd(SDA_pd),

		.clk(clk26buf),
		.glbl_reset(1'b0),

		.i2c_device_addr(8'h3C),

		/* Input bank 0 */
		.reg_2(pause_writing),

		/* Output bank 0 */
		.reg_8(byte_counter[31:24]),
		.reg_9(byte_counter[23:16]),
		.reg_a(byte_counter[15:8]),
		.reg_b(byte_counter[7:0]),

		.reg_c(0),
		.reg_d(0),
		.reg_e(0),
		.reg_f(8'hBE),

		/* Input bank 1 */
		.reg_10(block_skip_input[31:24]),
		.reg_11(block_skip_input[23:16]),
		.reg_12(block_skip_input[15:8]),
		.reg_13(block_skip_input[7:0]),

		/* Output bank 1 */
		.reg_18(block_skip[31:24]),
		.reg_19(block_skip[23:16]),
		.reg_1a(block_skip[15:8]),
		.reg_1b(block_skip[7:0]),
		.reg_1c(rd_data_count[12:8]),
		.reg_1d(rd_data_count[7:0]),
		.reg_1e(wr_data_count[12:8]),
		.reg_1f(wr_data_count[7:0])
	);

endmodule // kovan
