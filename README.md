Kovan SD Project
================

FPGA and server files for Kovan-based SD tap board


Pin Mapping (LCD header)
-----------

The following is a mapping of pins from the tap board to the FPGA's LCD header:

DA LCD\_R7
DB LCD\_R6
DC LCD\_R5
DD LCD\_R4
DE LCD\_R3
DF LCD\_R2
DG LCD\_G7
DH LCD\_G6

ALE LCD\_G5
CLE LCD\_G4
WE LCD\_G3
CS LCD\_G2
RE LCD\_B7
RB LCD\_B6
UK0 LCD\_B5
UK1 LCD\_B4

UK2 LCD\_B3
UK3 LCDO\_VSYNC
UK4 LCDO\_HSYNC
UK5 LCDO\_DEN
UK6 LCDO\_DOTCLK
UK8 LCDO\_RESET\_N


Pin Mapping (HDMI connector)
----------------------------

The following is a mapping of pins from the tap board to the FPGA's HDMI header:


UK7 UK7\_BUF
UK9 UK9\_BUF
SD\_CS
SD\_SCLK
SD\_TURNON
SD\_DAT1
SD\_DAT1
SD\_DI
SD\_DO
SD\_CD


Pin Mapping (FPGA as passthrough)
---------------------------------

When the FPGA firmware is loaded, the followng pins pass directly through:

CAM\_VSYNC SD\_SCLK
CAM\_HSYNC SD\_MOSI
CAM\_VCLKO SD\_MISO
CAM\_VCLKI SD\_CS
CAM\_MCLKO SD\_TURNON
CAM\_D0 D\_READY # Output, toggles when data is present on pins
CAM\_D1 D\_AVAIL # Output, goes high when new data is available
CAM\_D2 D\_NEXT # Input, toggle to present new data on pins
CAM\_D3 CLK\_13
CAM\_D4 CLK\_14
CAM\_D5 CLK\_15
CAM\_D6 CLK\_16
CAM\_D7 CLK\_17

LCD\_R0 NAND\_DA
LCD\_R1 NAND\_DB
LCD\_R2 NAND\_DC
LCD\_R3 NAND\_DD
LCD\_R4 NAND\_DE
LCD\_R5 NAND\_DF
LCD\_G0 NAND\_DG
LCD\_G1 NAND\_DH
LCD\_G2 NAND\_RW
LCD\_G3 NAND\_TYPE0
LCD\_G4 NAND\_TYPE1
LCD\_G5 CLK\_0
LCD\_B0 CLK\_1
LCD\_B1 CLK\_2
LCD\_B2 CLK\_3
LCD\_B3 CLK\_4
LCD\_B4 CLK\_5
LCD\_B5 CLK\_6
LCD\_SUPP0 CLK\_7
LCD\_SUPP1 CLK\_8
LCD\_SUPP2 CLK\_9
LCD\_SUPP3 CLK\_10
LCD\_SUPP4 CLK\_11
LCD\_SUPP5 CLK\_12

I2S\_DI0 CLK\_18
I2S\_DO0 CLK\_19
I2S\_LRCLK0 CLK\_20
I2S\_CDCLK0 CLK\_21
I2S\_CLK0 CLK\_22

FPGA\_MISO CLK\_23
FPGA\_MOSI CLK\_24
FPGA\_SYNC 
FPGA\_DIN 
FPGA\_CCLK
PWR\_SCL
PWR\_SDA
Xi2cSDA
Xi2cSCL


Programming Model
=================

Every time the WE or RE pin rises, the FPGA captures the values present on
DA-DH, whether it was a read or a write, and whether it was a command,
address, or data.  Additionally, a 26-bit timestamp will be captured, along
with 10 bits of the "unknown" data pins.

When data is available, the D\_AVAIL pin goes high.  When the data buffer
is empty, D\_AVAIL returns low.

In order to read data, toggle D\_NEXT.  The FPGA will load the new values
onto the output pins, and will toggle D\_READY when it has done so.  When
D\_READY equals D\_NEXT, then the data may be read.  Note that if you
attempt to read data when D\_AVAIL is low, then the output will be read as
zero.


