NANOCBOR_DIR = ../..

include ../../Makefile

SRCS += main.c  test_decoder.c test_encoder.c
LDFLAGS += -Wl,$(shell pkg-config --libs cunit || echo -lcunit)
CFLAGS += $(shell pkg-config --cflags cunit)

$(CURDIR)/bin/test: $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) $(LDFLAGS) -o $@

test : CFLAGS += -g3

test: $(CURDIR)/bin/test
	$(CURDIR)/bin/test
	
