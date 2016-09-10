BINARY = pwm-profiler
SOURCES = pwm-profiler.c
LDSCRIPT = $(BINARY).ld
include libopencm3.target.mk
