#ifndef NEIGHBORHOOD_HPP
#define NEIGHBORHOOD_HPP

#include "tabu_search.hpp"

namespace CVRP2L
{
class neighborhood
{
protected:
	instance &cvrp2l;

	packing_2d_solver &packing_solver;

	tsp_solver &tsp_sol;

	solution &cur_sol;

	tabu_search &ts;

public:
	neighborhood(tabu_search &s, solution &sol, instance &inst, tsp_solver &tsp, packing_2d_solver &packing) : ts(s), cur_sol(sol), cvrp2l(inst), tsp_sol(tsp), packing_solver(packing) {}

	virtual double best_move_improvement() = 0;

	virtual void apply_best_move() = 0;
};

class relocation_neighborhood : public neighborhood
{

	std::vector<std::tuple<int, int, int>> moves;

	std::tuple<int, int, int> best_move;

	double best_move_delta;

	double evaluate_relocation_move(int i, int j, int k);

public:
	relocation_neighborhood(tabu_search &s, solution &sol, instance &inst, tsp_solver &tsp, packing_2d_solver &packing) : neighborhood(s, sol, inst, tsp, packing) {}

	double best_move_improvement();

	void apply_best_move();
};

class swap_neighborhood : public neighborhood
{

	std::vector<std::tuple<int, int, int, int>> moves;

	std::tuple<int, int, int, int> best_move;

	double best_move_delta;

	double evaluate_swap_move(int i, int j, int k, int l);

public:
	swap_neighborhood(tabu_search &s, solution &sol, instance &inst, tsp_solver &tsp, packing_2d_solver &packing) : neighborhood(s, sol, inst, tsp, packing) {}

	double best_move_improvement();

	void apply_best_move();
};

class eject_neighborhood : public neighborhood
{

	std::vector<std::tuple<int, int, int, int, int>> moves;

	std::tuple<int, int, int, int, int> best_move;

	double best_move_delta;

	double evaluate_eject_move(int i, int j, int k, int l, int m);

	bool relocation_infeasible(int i, int j, int k);

public:
	eject_neighborhood(tabu_search &s, solution &sol, instance &inst, tsp_solver &tsp, packing_2d_solver &packing) : neighborhood(s, sol, inst, tsp, packing) {}

	double best_move_improvement();

	void apply_best_move();
};

} // namespace CVRP2L

#endif