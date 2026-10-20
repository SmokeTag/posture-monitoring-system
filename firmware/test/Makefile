# Host tests (no board): make -C firmware/test
#   test_posturelink: PostureLink.h / SerialOut.h units
#   test_chair, test_body: the real sketches on host stubs (stub/)
#   test_system: both sketches together, over a fake radio
CXX      ?= g++
CXXFLAGS ?= -std=gnu++11 -O1 -g -Wall -Wextra -Werror -fsanitize=address,undefined
LIB      := ../libraries/PostureLink
DEPS     := check.h $(LIB)/PostureLink.h $(LIB)/SerialOut.h $(LIB)/RandomId.h $(LIB)/Lsm6Accel.h
STUBS    := $(wildcard stub/*.h)
TESTS    := test_lsm6 test_posturelink test_chair test_body test_system

test: $(TESTS)
	@for t in $(TESTS); do ./$$t || exit 1; done
	./test_body init

test_posturelink: test_posturelink.cpp $(DEPS)
	$(CXX) $(CXXFLAGS) -I$(LIB) -o $@ $<

test_chair: test_chair.cpp ../chair/chair.ino $(DEPS) $(STUBS)
	$(CXX) $(CXXFLAGS) -Istub -I$(LIB) -o $@ $<

test_body: test_body.cpp ../body/body.ino $(DEPS) $(STUBS)
	$(CXX) $(CXXFLAGS) -Istub -I$(LIB) -o $@ $<

test_system: test_system.cpp ../chair/chair.ino ../body/body.ino $(DEPS) $(STUBS)
	$(CXX) $(CXXFLAGS) -Istub -I$(LIB) -o $@ $<

clean:
	rm -f $(TESTS)

.PHONY: test clean

test_lsm6: test_lsm6.cpp $(DEPS) stub/Wire.h
	$(CXX) $(CXXFLAGS) -I$(LIB) -o $@ $<
