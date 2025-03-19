CFLAGS= -O3 -pthread 

<<<<<<< HEAD
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
=======
LIBS= -lfftw3 -lm 

INCLUDE= 

.c.o:
	gcc $(CFLAGS) $(INCLUDE) -c -o $@ $<

OBJS=clear_frequency_server.o clear_freq_search.o ini_parser.o misc_read_writes.o read_config.o

cf_server:	$(OBJS)
	gcc -o ../cf_server $(CFLAGS) $(OBJS) $(LIBS)


>>>>>>> 39feef2 (Moved c files out of include statements, added typedeffs to header)
