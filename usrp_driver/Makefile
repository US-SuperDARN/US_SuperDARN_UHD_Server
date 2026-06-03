OS:=$(shell uname -s)

CCpp=g++
CFLAGS=-c -Wall -pthread -std=c++0x -O3
LFLAGS=-L"${HOME}/repos/uhd/host/build/lib"
common_libs=-lm -lrt -luhd
linux_libs=-lboost_thread -lboost_system -pthread -l argtable2 -lboost_chrono

CPP_SOURCES=usrp_driver.cpp dio.cpp tx_worker.cpp rx_worker.cpp usrp_utils.cpp

CPP_OBJECTS=$(CPP_SOURCES:.cpp=.o)

EXECUTABLE=usrp_driver

all: $(EXECUTABLE)

build: $(EXECUTABLE)

$(EXECUTABLE): $(CPP_OBJECTS)
	$(CCpp) -o $@ $(CPP_OBJECTS) $(LFLAGS) $(common_libs) $(linux_libs)

.cpp.o:
	$(CCpp) $(CFLAGS) $< -o $@ $(LFLAGS)

clean:
	rm -rf *.o *.pyc $(EXECUTABLE)

debug:
	gdb --args $(EXECUTABLE) --host usrp1 --ant 1 --intclk

cudebug:
	cuda-gdb --args python3 -m pycuda.debug cuda_driver.py
