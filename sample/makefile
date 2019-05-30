CXX= g++
CONCORDE=/mnt/c/Users/arthu/git/MO824-Projeto-Final/concorde
QSOPT=/mnt/c/Users/arthu/git/MO824-Projeto-Final/QS
CPPFLAGS= -O2 -w -m64 -no-pie
LIBFLAGS= -I$(QSOPT) -I$(CONCORDE) -L$(CONCORDE) -L$(QSOPT) -lconcorde -lqsopt -lm -lpthread -ldl
all:
	$(CXX) main.cpp -o main.exe $(LIBFLAGS) $(CPPFLAGS)
	@echo ‘‘-- DONE --’’
run:
	./main.exe

