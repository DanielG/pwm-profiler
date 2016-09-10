CC ?= gcc

CFLAGS=-g -ggdb3 -Wall --std=gnu99 # -Werror
LDFLAGS=

SRCS=$(wildcard *.c)
OBJS=$(patsubst %.c,%.o,$(SRCS))
DEPS=$(patsubst %.o,%.d,$(OBJS))

all: fuzcl

install: all

fuzcl: $(OBJS)
	@echo "\tLD $@"
	$(CC) $(LDFLAGS) -o $@ $+

%.o: %.c
	@echo "\tCC $@"
	@$(CC) $(CFLAGS) -c $< -o $@
	@$(CC) -MM $(CFLAGS) $*.c > $*.d

clean:
	rm -f *.o *.d
