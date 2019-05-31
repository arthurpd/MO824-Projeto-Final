CXX= g++

CONCORDE=/mnt/c/Users/arthu/git/MO824-Projeto-Final/concorde
QSOPT=/mnt/c/Users/arthu/git/MO824-Projeto-Final/QS

CPPFILES = $(shell find $(SRCDIR) -maxdepth 1 -name '*.cpp')
OBJFILES = $(patsubst %.cpp,%.o,$(CPPFILES))

CPPFLAGS= -g -std=c++11 -march=native -O2 -w -m64 -no-pie -Wall -Wextra -I$(QSOPT) -I$(CONCORDE)
LDFLAGS= -L$(CONCORDE) -L$(QSOPT) -lconcorde -lqsopt -lm -lpthread -ldl

SRCDIR = . packing

EXEFILE = main.exe

all: $(EXEFILE)

$(EXEFILE): $(OBJFILES)
	$(CXX) $(OBJFILES) -o $@ $(LDFLAGS)

%.o: %.cpp
	$(CXX) -c $< -o $@ $(CFLAGS)

clean:
	rm -f $(OBJFILES) $(EXEFILE)

run:
	./main.exe instances/2l_cvrp0101.txt
