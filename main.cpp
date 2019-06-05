#include "tabu_search/tabu_search.hpp"
#include "input_reader/instance.hpp"
#include "packing/packing.hpp"
#include "tsp/tsp.hpp"
#include "utils.hpp"

std::ofstream utils::log;
double utils::total_tsp_time = 0;
double utils::total_packing_time = 0;

int main(int argc, char **argv)
{
	using namespace std;
	using namespace utils;
	int verbose = 1;

	// To hide concorde output.
	freopen("/dev/null", "w", stdout);

	// Write to file since we can't use stdout.
	utils::log.open("2lcvrp_log.txt", std::ios::out | std::ios::trunc);

	assert(argc == 3);
	CVRP2L::instance cvrp2l(argv[1]);
	if (verbose)
		cvrp2l.print();

	CVRP2L::packing_2d_solver packing_solver(cvrp2l);

	if (verbose)
	{
		// Trial packing.
		packing_solver.verbose = true;
		std::vector<int> idx = {0, 1, 2, 3, 4};
		utils::log << packing_solver.feasible(idx) << endl;
		packing_solver.verbose = false;
	}

	CVRP2L::tsp_solver tsp_solver(cvrp2l);
	if (verbose)
	{
		// Trial tsp.
		tsp_solver.verbose = true;
		std::vector<int> idx = {0, 1, 2, 3, 4};
		utils::log << tsp_solver.solve(idx) << endl;
		tsp_solver.verbose = false;
	}

	CVRP2L::tabu_search search(cvrp2l, tsp_solver, packing_solver);
	CVRP2L::solution best_sol = search.run(atof(argv[2]), ceil(cvrp2l.n / 3.0));
	best_sol.print();

	// Following code checks solution (independently from the tabu search code).

	if (best_sol.v > cvrp2l.v)
	{
		utils::log << "Infeasible Solution (vehicle number)" << endl;
	}
	else if (best_sol.v < cvrp2l.v)
	{
		utils::log << "Warning: using less vehicles" << endl;
	}

	tsp_solver.verbose = false;

	int tmp = 0;
	for (int i = 0; i < best_sol.routes.size(); i++)
		tmp += tsp_solver.solve(best_sol.routes[i]);

	if (tmp != best_sol.val)
	{
		utils::log << "Solution value does not match" << endl;
	}

	for (int i = 0; i < best_sol.routes.size(); i++)
	{
		if (!packing_solver.feasible(best_sol.routes[i]))
			utils::log << "Infeasible packing" << endl;

		tmp = 0;
		for (int j = 0; j < best_sol.routes[i].size(); j++)
			tmp += cvrp2l.d[best_sol.routes[i][j]];
		if (tmp > cvrp2l.vc)
			utils::log << "Vehicles capacity violated" << endl;
	}

	// Print time stats.
	utils::log << "Total tsp time " << utils::total_tsp_time << endl;
	utils::log << "Total packing time " << utils::total_packing_time << endl;

	// Close log file.
	utils::log.close();

	return 0;
}