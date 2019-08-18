NANOCBOR_DIR = ../..

include ../../Makefile

SRCS += main.c

bin/fuzztest: $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -o $@

test : CFLAGS += -g3

test: bin/fuzztest
