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
	wire          clk26;
	wire          clk26ibuf;
	wire          clk26buf;
	wire          reset_clock;
	wire          clk125;
 

	/* This set of wires comes out of the FIFO, and feeds into a mux */
	wire [63:0]   mem_output;


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

	/* Standard blinky LED counter */
	reg  [32:0]   free_timer;

	wire [11:0]   wr_data_count;
	wire [11:0]   rd_data_count;

	/* Used as part of a rising-edge pulse-generator */
	reg           did_write_i2c;
	reg           do_write_i2c;
	reg           do_write;
	wire          do_write_i2c_buf;
 
	reg           did_read;
	reg           last_read;
	reg           do_read;
	wire          do_read_buf;
 
	/* Master chunk of BRAM.  When a sample is taken, it's stored here. */
	reg  [63:0]   mem_input;
	reg  [63:0]   mem_input_d; /* Buffer one sample back in time */
	reg           we_s1, we_s2;
	reg           rd_s1, rd_s2;
	wire          is_full;
	wire          is_empty;

	wire [1:0]    output_bank;
	reg  [15:0]   output_reg;

	wire          SDA_pd;
	wire          SDA_int;

	reg           previous_nand_we;
	reg           previous_nand_re;
	reg           previous_previous_nand_re;

	reg  [31:0]   byte_counter;

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
	//assign M_SERVO = 1'b0;
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


	assign clk26 = OSC_CLK;
	IBUFG clk26buf_ibuf(.I(clk26), .O(clk26ibuf));
	BUFG clk26buf_buf (.I(clk26ibuf), .O(clk26buf));


	/* These allow us to do bank selection for the output register value */
	assign output_bank[0] = LCD_HS;
	assign output_bank[1] = LCD_VS;
	assign reset_clock = LCD_DEN;

	/* The actual output pins that go from the mux and feed to the CPU */
	assign CAM_D[5]   = 1'b1; // Set OTG connected (due to miswiring)
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


	assign LCD_R[2] = SD_DO_T;
	assign LCD_R[1] = !is_empty;
	assign LCD_R[0] = is_full;


	/* Poor-man's diagnostics */
	wire diag_1, diag_2, diag_3, diag_4;
	assign M_SERVO[0] = !diag_1;
	assign M_SERVO[1] = !diag_2;
	assign M_SERVO[2] = !diag_3;
	assign M_SERVO[3] = !diag_4;
	assign FPGA_LED = 1'b1;
	assign diag_1 = do_write_i2c;
	assign diag_2 = free_timer[3];
	assign diag_3 = did_write_i2c;
	assign diag_4 = do_write_i2c_buf;



	/* Multiply the incoming 26 MHz clock up to 125 MHz so we can build our
	 * own edge detector.  We want to trigger an event on a rising edge of
	 * NAND_WE or the falling edge of NAND_RE.
	 */
	fast_clock fast_clock(
		.CLK_IN1(clk26ibuf),
		.CLK_OUT1(clk125)
	);


	/* Turn do_read into a clock */
	assign do_read_buf = CAM_MCLKO;

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

		/* Inputs */
		.reg_0(do_write_i2c_buf),
		.reg_1(do_read_i2c_buf),

		/* Outputs */
		.reg_8(byte_counter[7:0]),
		.reg_9(byte_counter[15:8]),
		.reg_a(byte_counter[23:16]),
		.reg_b(byte_counter[31:24]),

		.reg_c(free_timer[7:0]),
		.reg_d(is_full),
		.reg_e(is_empty),
		.reg_f(8'hBE),

		.reg_18(8'h42)
	);


	/* Captured samples temporarily get stored here */
	fifo sample_buffer(
		.rst(1'b0),

		.wr_en(do_write),
		.wr_clk(clk125),

		.rd_en(do_read),
		.rd_clk(clk26buf),

		.wr_data_count(wr_data_count),
		.rd_data_count(rd_data_count),

		.din(mem_input_d),
		.dout(mem_output),

		.full(is_full),
		.empty(is_empty)
	);


	/* Promary edge detector loop */

	always @(posedge clk26buf) begin
		if (!do_read_buf) begin
			did_read <= 0;
			do_read <= 0;
		end
		else if (!did_read) begin
			did_read <= 1;
			do_read <= 1;
		end else begin
			did_read <= 1;
			do_read <= 0;
		end
	end


	always @(posedge clk125) begin
		if (!do_write_i2c_buf) begin
			did_write_i2c <= 0;
			do_write_i2c <= 0;
		end
		else if (!did_write_i2c) begin
			did_write_i2c <= 1;
			do_write_i2c <= 1;
		end else begin
			did_write_i2c <= 1;
			do_write_i2c <= 0;
		end

		/* Always tick the clock (or reset it) */
		if (reset_clock)
			free_timer <= 0;
		else
			free_timer <= free_timer+1;

		/* Pipeline the data one deep so we can 'reach back in time' */
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

		we_s1            <= WE;
		we_s2            <= we_s1;
		previous_nand_we <= we_s2;

		rd_s1                     <= RE;
		rd_s2                     <= rd_s1;
		previous_nand_re          <= rd_s2;
		previous_previous_nand_re <= previous_nand_re;

		//do_write <= (((!previous_nand_re) & previous_previous_nand_re))
		//	  | (((!previous_nand_we) & we_s2));
		if (((!previous_nand_re) & previous_previous_nand_re)
			  | (((!previous_nand_we) & we_s2)) ) begin
			do_write <= 1;
			byte_counter <= byte_counter+1;
		end
		else begin
			do_write <= 0;
			byte_counter <= byte_counter;
		end

		/* Compare the NAND read/write pins to determine if we have to
		 * capture a sample and put it in the buffer
		 */
		if ((!previous_nand_we) && we_s2) begin
			// grab from 'reach back' so *before* edge
			mem_input_d[45:0]  <= mem_input[45:0];
			mem_input_d[46]    <= 1'b1; // WE
			mem_input_d[47]    <= 1'b0; // RE
			mem_input_d[63:48] <= mem_input[63:48];
		end
		else if((!previous_nand_re) && previous_previous_nand_re) begin
			// grab two cycles after falling edge,
			// to give time for NAND to produce data
			mem_input_d[31:0]  <= free_timer;
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
			mem_input_d[63:48] <= mem_input_d[63:48];
			mem_input_d[47]    <= 1'b0; // clear these
			mem_input_d[46]    <= 1'b0;
			mem_input_d[45:0]  <= mem_input_d[45:0];
		end

	end

endmodule // kovan
