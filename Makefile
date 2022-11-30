BOARD=openmote-b
SERIAL_PORT=/dev/ttyUSB1
VERBOSE=1

%:
	scons board=$(BOARD) project=$(@) compiler=gcc verbose=$(VERBOSE) bootload=$(SERIAL_PORT) VERBOSE=1 -Q
