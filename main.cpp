#include "instance.hpp"
#include "packing/packing.hpp"

int main(int argc, char **argv)
{
	using namespace std;
	assert(argc == 2);
	CVRP2L::instance cvrp2l(argv[1]);
	cvrp2l.print();

	CVRP2L::packing_2d_solver packing_solver(cvrp2l);
	std::vector<int> idx = {1, 2, 11, 3, 10, 7};
	cout << packing_solver.feasible(idx) << endl;
}