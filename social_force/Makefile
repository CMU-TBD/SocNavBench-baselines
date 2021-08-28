CXX = g++
RM = rm -f
INCLUDES = -I./src
LIBS= -L./src
CXXFLAGS = ${INCLUDES} -lpedsim -std=c++11
PROGRAMS = social_force example01

all: $(PROGRAMS)

social_force: export LD_LIBRARY_PATH=./src
social_force: social_force.cpp
	$(RM) social_force
	$(CXX) social_force.cpp $(CXXFLAGS) -o social_force $(LIBS)

example01: export LD_LIBRARY_PATH=./src
example01: example01.cpp
	$(RM) example01
	$(CXX) example01.cpp $(CXXFLAGS) -o example01 $(LIBS)

clean:
	$(RM) $(PROGRAMS)
	$(RM) *.o

.PHONY: all clean
