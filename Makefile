all:
	make -C synth
%:
	make -C synth $*
