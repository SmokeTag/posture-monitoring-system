# Host unit tests for firmware/libraries/PostureLink: make -C firmware/test
CXX      ?= g++
CXXFLAGS ?= -std=gnu++11 -O1 -g -Wall -Wextra -Werror -fsanitize=address,undefined
LIB      := ../libraries/PostureLink

test: test_posturelink
	./test_posturelink

test_posturelink: test_posturelink.cpp $(LIB)/PostureLink.h $(LIB)/SerialOut.h
	$(CXX) $(CXXFLAGS) -I$(LIB) -o $@ $<

clean:
	rm -f test_posturelink

.PHONY: test clean
