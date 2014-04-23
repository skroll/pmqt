CC ?= gcc
AR ?= ar
PREFIX = /usr/local

CFLAGS = -O3 -std=c99 -Wall -pedantic

SRC = src/pmqt.c
OBJ = $(SRC:.c=.o)

all: build/libpmqt.a

build/libpmqt.a: $(OBJ)
	mkdir -p build
	$(AR) rcs $@ $^

clean:
	rm -rf bin *.o src/*.o

%.o: %.c
	$(CC) $< $(CFLAGS) -c -o $@

.PHONY: clean

