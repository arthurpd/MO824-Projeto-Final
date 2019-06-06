#ifndef TABU_SEARCH_HPP
#define TABU_SEARCH_HPP

#include "../input_reader/instance.hpp"
#include "../packing/packing.hpp"
#include "../tsp/tsp.hpp"
#include "../utils.hpp"

namespace CVRP2L
{
const double infd = 1e30;

struct solution
{
	// Vehicle routes description. All routes start with depot (0).
	std::vector<std::vector<int>> routes;

	std::vector<int> route_demand;

	// Sum of TSP solution for all routes.
	int val;

	// If the solution uses the required amount of vehicles. (The solution is always packing feasible)
	bool feasible;

	// Number of vehicles being used
	int v;

	void print();

	static int evaluate_sol(solution &sol, tsp_solver &tsp_sol, instance &cvrp2l);

	static int get_over_route_limit_penalty(int i, instance &cvrp2l);
};

class tabu_search
{
	instance &cvrp2l;

	packing_2d_solver &packing_solver;

	tsp_solver &tsp_sol;

	solution cur_sol;

	solution best_sol;

	// tabu_tab[i] is the number of occurrences of element i in the tabu_list (could be more than 1 due to aspiration criteria).
	std::vector<int> tabu_tab;

	std::list<pii> tabu_list;

	int iteration;

	void remove_old_tabu(int tabu_tenure);

	solution initial_solution();

	void init_run();

	double evaluate_relocation_move(int i, int j, int k);

	void apply_relocation_move(int i, int j, int k);

	bool neighborhood_move();

public:
	// Penalty for using more vehicles than allowed.
	double over_route_limit_penalty;

	tabu_search(instance &inst, tsp_solver &tsp, packing_2d_solver &packing);

	bool is_tabu(int i, int j);

	void insert_tabu(int x);

	solution run(double time_limit, int tabu_tenure);
};

} // namespace CVRP2L

#endif