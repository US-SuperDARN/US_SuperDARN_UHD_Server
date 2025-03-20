CFLAGS= -O3 -pthread 

LIBS= -lfftw3 -lm -lrt

INCLUDE= 

.c.o:
	gcc $(CFLAGS) $(INCLUDE) -c -o $@ $<

OBJS=clear_frequency_server.o clear_freq_search.o ini_parser.o misc_read_writes.o read_config.o

cf_server:	$(OBJS)
	gcc -o ../cf_server $(CFLAGS) $(OBJS) $(LIBS)

clean:
	rm -rf *.o

