OS:=$(shell uname -s)

CCpp=g++
CFLAGS=-c -Wall -pthread -std=c++0x -g3
LFLAGS=-L"${HOME}/repos/iniparser/build" -L"${HOME}/repos/uhd/host/build/lib"
common_libs=-lm -liniparser -lrt -luhd -lfftw3
linux_libs=-lboost_thread -lboost_system -pthread -l argtable2 -lboost_date_time

INCLUDES=-I"${HOME}/repos/uhd/host/include" \
	 -I"${HOME}/repos/uhd/host/include/uhd/types" \
	 -I"${HOME}/repos/uhd/host/include/uhd/usrp" \
	 -I"${HOME}/repos/rst/include/base" \
	 -I"${HOME}/repos/rst/include/superdarn"

CPP_SOURCES=usrp_driver.cpp dio.cpp tx_worker.cpp  rx_worker.cpp usrp_utils.cpp

CPP_OBJECTS=$(CPP_SOURCES:.cpp=.o)

EXECUTABLE=usrp_driver

all: $(EXECUTABLE)


$(EXECUTABLE): $(CPP_OBJECTS)
	$(CCpp) -o $@ $(CUDA_OBJECT0) $(CUDA_OBJECT1) $(CPP_OBJECTS) $(LFLAGS) $(common_libs) $(linux_libs) ${INCLUDES}

.cpp.o:
	$(CCpp) $(CFLAGS) $< -o $@ $(LFLAGS) $(INCLUDES)

clean:
	rm -rf *.o *.pyc $(EXECUTABLE)

debug:
	gdb --args $(EXECUTABLE) --host usrp1 --ant 1 --intclk

cudebug:
	 cuda-gdb --args python3 -m pycuda.debug cuda_driver.py 
