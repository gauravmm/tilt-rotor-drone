CC      = ../arduino-1.6.12/arduino
CCFLAGS = -v --board arduino:avr:nano:cpu=atmega328
MAINFILE= tilt-controller/tilt-controller.ino

all: upload

verify:
	$(CC) --verify $(CCFLAGS) $(MAINFILE)

upload:
	$(CC) --upload $(CCFLAGS) $(MAINFILE)

