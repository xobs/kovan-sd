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
		output wire       SD_CLK_T,
		output wire       SD_TURNON_T,
		input wire        UK7_BUF_T,
		output wire       SD_DAT2_T,
		output wire       SD_DAT1_T,
		output wire       SD_DO_T,

		output wire [7:0] CAM_D,
		output wire       CAM_HSYNC,
		output wire       CAM_MCLKO,
		output wire       CAM_PCLKI,
		output wire       CAM_VCLKO,
		output wire       CAM_VSYNC,

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
		input wire 	DA,
		input wire 	DB,
		input wire 	DC,
		input wire 	DD,
		input wire 	DE,
		input wire 	DF,
		input wire 	DG,
		input wire 	DH,

		input wire 	UK0,
		input wire 	UK1,
		input wire 	UK2,
		input wire 	UK3,
		input wire 	UK4,
		input wire 	UK5,
		input wire 	UK6,
		/* UK7 wired elsewhere */
		input wire 	UK8,
		/* UK9 wired elsewhere */

		input wire 	ALE,
		input wire 	CLE,
		input wire 	WE,
		input wire 	CS,
		input wire 	RE,
		input wire 	RB,

		// LCD input from CPU
		input wire [5:0]  LCD_B, // note no truncation of data in
		input wire [5:0]  LCD_G,
		input wire [5:0]  LCD_R,
		input wire        LCD_DEN,
		input wire        LCD_HS,
		input wire [5:0]  LCD_SUPP,
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

	///////// clock buffers
	wire      clk26;
	wire      clk26ibuf;
	wire      clk26buf;
	wire      clk13buf;
	wire      clk3p2M;
	wire      clk208M;
	wire      clk1M;  // wired up in the serial number section
 
	assign clk26 = OSC_CLK;
	IBUFG clk26buf_ibuf(.I(clk26), .O(clk26ibuf));
	BUFG clk26buf_buf (.I(clk26ibuf), .O(clk26buf));

	wire [7:0] 	  NAND_D;
	wire [9:0] 	  NAND_UK;
	wire          NAND_ALE;
	wire          NAND_CLE;
	wire          NAND_WE;
	wire          NAND_CS;
	wire          NAND_RE;
	wire          NAND_RB;

	wire          SD_CS;
	wire          SD_CLK;
	wire          SD_DO;
	wire          SD_DAT1;
	wire          SD_DAT2;
	wire          SD_TURNON;
	wire          SD_DI;

	// convenience renaming of signals (mapping from pcb tap names to
	// informative meanings)
	assign NAND_D[7:0] = {DH, DG, DF, DE, DD, DC, DB, DA};
	assign NAND_UK[9:0] = {UK9_BUF_T, UK8, UK7_BUF_T, UK6, UK5, UK4, UK3, UK2, UK1, UK0};
	assign NAND_ALE = ALE;
	assign NAND_CLE = CLE;
	assign NAND_WE = WE;
	assign NAND_CS = CS;
	assign NAND_RE = RE;
	assign NAND_RB = RB;

	assign SD_CS = SD_CS_T;
	assign SD_CLK = SD_CLK_T;
	assign SD_DO = SD_DO_T;
	assign SD_DAT2 = SD_DAT2_T;

	assign SD_DO_T = LCD_B[0];
	assign SD_CS_T = LCD_B[1];
	assign SD_CLK_T = LCD_B[2];
	assign SD_DAT2_T = LCD_B[3];

	// Assign dummy values for now, to get it to build
	assign SD_DI_T = 1'b1;
	assign DIG_SCLK = 1'b1;
	assign FPGA_LED = 1'b1;
	assign DIG_ADC_CS = 1'b1;
	assign DIG_ADC_SCLK = 1'b1;
	assign DIG_ADC_OUT = 1'b1;
	assign MBOT = 1'b1;
	assign MTOP = 1'b1;
	assign M_SERVO = 1'b1;
	assign DIG_RCLK = 1'b1;
	assign SD_DI = 1'b1;
	assign SD_DAT1_T = 1'b1;
	assign MOT_EN = 1'b1;
	assign DIG_CLR_N = 1'b1;
	assign DIG_SAMPLE = 1'b1;
	assign FPGA_MISO = 1'b1;
	assign DIG_SRLOAD = 1'b1;
	assign SD_TURNON_T = 1'b1;
	assign DIG_IN = 1'b1;
	assign DIG_ADC_IN = 1'b1;
    assign CAM_D = 1'b1;
	assign CAM_HSYNC = 1'b1;
	assign CAM_MCLKO = 1'b1;
	assign CAM_PCLKI = 1'b1;
	assign CAM_VCLKO = 1'b1;
	assign CAM_VSYNC = 1'b1;

	assign I2S_DI1 = 1'b1;
	assign DDC_SDA_PD = 1'b1;
	assign HPD_OVERRIDE = 1'b1;
	assign I2S_CDCLK1 = 1'b1;
	assign I2S_DO0 = 1'b1;
	assign I2S_LRCLK0 = 1'b1;
	assign DDC_SDA_PU = 1'b1;
	assign I2S_CLK0 = 1'b1;

   
	////////// reset
	reg       glbl_reset; // to be used sparingly
	wire      glbl_reset_edge;
	reg       glbl_reset_edge_d;

	always @(posedge clk1M) begin
		glbl_reset_edge_d <= glbl_reset_edge;
		glbl_reset <= !glbl_reset_edge_d & glbl_reset_edge; // just pulse reset for one cycle of the slowest clock in the system
	end
   
	////////// loop-throughs
	// lcd runs at a target of 6.41 MHz (156 ns cycle time)
	// i.e., 408 x 262 x 60 Hz (408 is total H width, 320 active, etc.)
	wire            qvga_clkgen_locked;
   
   
	// low-skew clock mirroring to an output pin requires this hack
	ODDR2 qvga_clk_to_lcd (.D0(1'b1), .D1(1'b0), 
			.C0(clk_qvga), .C1(!clk_qvga), 
			.Q(LCDO_DOTCLK), .CE(1'b1), .R(1'b0), .S(1'b0) );

	ODDR2 qvga_clk_to_cpu (.D0(1'b1), .D1(1'b0), 
			.C0(clk_qvga), .C1(!clk_qvga), 
			.Q(LCD_CLK_T), .CE(1'b1), .R(1'b0), .S(1'b0) );
	wire [63:0]  mem_data;
	wire [63:0]  mem_buffer;
	wire         is_full;
	wire         is_empty;

	fifo fifo(
		.clk(1'b1),
		.rst(1'b1),
		.din(mem_buffer),
		.wr_en(1'b1),
		.rd_en(1'b1),
		.dout(mem_data),
		.full(is_full),
		.empty(is_empty)
	);


endmodule // kovan
