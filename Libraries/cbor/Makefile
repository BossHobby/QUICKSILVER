CC ?= gcc

CFLAGS ?= -Wall \
    -D_FORTIFY_SOURCE=2 \
    -Wextra -Wcast-align -Wcast-qual -Wpointer-arith \
    -Waggregate-return -Wunreachable-code -Wfloat-equal \
    -Wformat=2 -Wredundant-decls -Wundef \
    -Wdisabled-optimization -Wshadow -Wmissing-braces \
    -Wstrict-aliasing=2 -Wstrict-overflow=5 -Wconversion \
    -Wno-unused-parameter \
    -pedantic -std=c11 -Isrc -Itest

CFLAGS_DEBUG := -g -DDEBUG
CFLAGS_RELEASE := -O3

all: debug

debug: CFLAGS += $(CFLAGS_DEBUG)
debug: cbor

release: CFLAGS += $(CFLAGS_RELEASE)
release: cbor

cbor: build/test/main.o build/test/cjson/cJSON.o build/src/cbor.o
	$(CC) $^ -o $@ $(CFLAGS)

build/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) -o $@ -c $< $(CFLAGS)

.PHONY: debug release

clean:
	rm -rf build

