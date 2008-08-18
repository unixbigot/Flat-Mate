AVRDUDE_PORT=/dev/ttyUSB0
#MCU = atmega168
MCU = attiny45

#F_CPU = 8000000
F_CPU = 10000000

TARGET = limon

SRC = limon.c

include ../avr-tmpl.mk


