project = kovan-sd
vendor = xilinx
family = spartan6
part = xc6slx9csg324-2
top_module = kovan
isedir = /opt/Xilinx/14.3/ISE_DS
xil_env = . $(isedir)/settings64.sh

vfiles = ../verilog/kovan-sd.v ../verilog/fifo.v ../verilog/fast_clock.v \
	 ../verilog/i2c_slave.v ../verilog/sync_reset.v ../verilog/nand_fifo.v

include xilinx.mk
