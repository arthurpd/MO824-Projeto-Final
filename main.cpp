#include "instance.hpp"

int main(int argc, char **argv)
{
	assert(argc == 2);
	CVRP2L::instance cvrp2l(argv[1]);
	cvrp2l.print();
}