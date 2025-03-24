CFLAGS= -O3 -pthread 

LIBS= -lfftw3 -lm -lrt

INCLUDE= 

OBJS=clear_frequency_server.o clear_freq_search.o ini_parser.o misc_read_writes.o read_config.o

TARGET= ../cf_server

all: 
	build

.c.o:
	gcc $(CFLAGS) $(INCLUDE) -c -o $@ $<

build:	$(OBJS)
	gcc -o $(TARGET) $(CFLAGS) $(OBJS) $(LIBS)

clean:
	-rm -rf $(OBJS) $(TARGET)

.PHONY: all build clean
