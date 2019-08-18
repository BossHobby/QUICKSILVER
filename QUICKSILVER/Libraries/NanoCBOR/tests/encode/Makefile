NANOCBOR_DIR = $(CURDIR)/../..

include ../../Makefile

SRCS += main.c

bin/encode: $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -o $@

test : CFLAGS += -g3

test: bin/encode
