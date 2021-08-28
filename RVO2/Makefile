CXX = g++
RM = rm -f
INCLUDES = -I./src
LIBS= ./src/libRVO.a
CXXFLAGS = ${INCLUDES} -Wall -g -O2 -std=c++11
PROGRAMS = RVO2

all: $(PROGRAMS)

RVO2: RVO2.cpp
	$(RM) RVO2
	$(CXX) RVO2.cpp $(CXXFLAGS) -o RVO2 $(LIBS)

clean:
	$(RM) $(PROGRAMS)
	$(RM) *.o

.PHONY: all clean
