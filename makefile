# Top level Makefile

SUBDIRS= c_include usrp_driver

all: build

build:
	for dir in $(SUBDIRS); do \
			$(MAKE) -C $$dir build; \
	done

clean:
	for dir in $(SUBDIRS); do \
			$(MAKE) -C $$dir clean; \
	done

.PHONY: all clean $(SUBDIRS)