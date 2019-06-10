#include "neighborhood.hpp"

namespace CVRP2L
{
using namespace std;

bool random_choice(double p)
{
	assert(RAND_MAX == INT_MAX);
	return (rand() / (double) RAND_MAX) <= p;
}

int calc_demand_mult(int cur_c, int delta, int vc)
{
	return max(0, (cur_c + delta - vc)) - max(0, (cur_c - vc));
}

double relocation_neighborhood::evaluate_relocation_move(int i, int j, int k)
{
	assert(i != k);

	// if (cur_sol.route_demand[k] + cvrp2l.d[cur_sol.routes[i][j]] > cvrp2l.vc)
	// 	return infd;

	vector<int> to_route = cur_sol.routes[k];
	to_route.insert(lower_bound(to_route.begin(), to_route.end(), cur_sol.routes[i][j]), cur_sol.routes[i][j]);

	if (!packing_solver.feasible(to_route))
		return infd;

	vector<int> from_route = cur_sol.routes[i];
	from_route.erase(from_route.begin() + j);

	double retv = 0;
	retv += tsp_sol.solve(from_route) + tsp_sol.solve(to_route) - tsp_sol.solve(cur_sol.routes[i]) - tsp_sol.solve(cur_sol.routes[k]);

	retv += ts.over_route_limit_penalty * (solution::get_over_route_limit_penalty(k, cvrp2l) - solution::get_over_route_limit_penalty(i, cvrp2l));

	retv += ts.over_demand_limit_penalty * calc_demand_mult(cur_sol.route_demand[k], cvrp2l.d[cur_sol.routes[i][j]], cvrp2l.vc);
	retv += ts.over_demand_limit_penalty * calc_demand_mult(cur_sol.route_demand[i], -cvrp2l.d[cur_sol.routes[i][j]], cvrp2l.vc);

	return retv;
}

double relocation_neighborhood::best_move_improvement()
{
	best_move = {-1, -1, -1};
	best_move_delta = infd;

	for (int i = 0; i < cur_sol.v; i++)
		if (cur_sol.routes[i].size() > 2 || (i + 1) == cur_sol.v) // Only allow last route to be emptied.
			for (int j = 1; j < cur_sol.routes[i].size(); j++)
				if (!ts.is_tabu(i, j))
					for (int k = 0; k < cur_sol.v + 1; k++) // Relocate to any currently used or one after last used.
						if (i != k)
						{
							if (cur_sol.routes[i].size() == 2 && k == cur_sol.v)
								continue; // Never empty a route to open a new one.

							moves.push_back({i, j, k});
						}

	random_shuffle(moves.begin(), moves.end());

	for (auto move : moves)
	{
		double move_delta = evaluate_relocation_move(get<0>(move), get<1>(move), get<2>(move));

		if (move_delta < best_move_delta)
		{
			best_move = move;
			best_move_delta = move_delta;
		}
	}

	return best_move_delta;
}

void relocation_neighborhood::apply_best_move()
{
	int i = get<0>(best_move), j = get<1>(best_move), k = get<2>(best_move);

	ts.insert_tabu(cur_sol.routes[i][j]);
	cur_sol.routes[k].insert(lower_bound(cur_sol.routes[k].begin(), cur_sol.routes[k].end(), cur_sol.routes[i][j]), cur_sol.routes[i][j]);
	cur_sol.routes[i].erase(cur_sol.routes[i].begin() + j);
	assert(is_sorted(cur_sol.routes[k].begin(), cur_sol.routes[k].end()));

	solution::evaluate_sol(cur_sol, tsp_sol, cvrp2l);
}

double swap_neighborhood::evaluate_swap_move(int i, int j, int k, int l)
{
	assert(i != k);

	// if (cur_sol.route_demand[k] + cvrp2l.d[cur_sol.routes[i][j]] - cvrp2l.d[cur_sol.routes[k][l]] > cvrp2l.vc)
	// 	return infd;

	// if (cur_sol.route_demand[i] - cvrp2l.d[cur_sol.routes[i][j]] + cvrp2l.d[cur_sol.routes[k][l]] > cvrp2l.vc)
	// 	return infd;

	vector<int> ri = cur_sol.routes[i];
	ri.erase(ri.begin() + j);
	ri.insert(lower_bound(ri.begin(), ri.end(), cur_sol.routes[k][l]), cur_sol.routes[k][l]);

	vector<int> rk = cur_sol.routes[k];
	rk.erase(rk.begin() + l);
	rk.insert(lower_bound(rk.begin(), rk.end(), cur_sol.routes[i][j]), cur_sol.routes[i][j]);

	if (!packing_solver.feasible(ri) || !packing_solver.feasible(rk))
		return infd;

	double retv = 0;
	retv += tsp_sol.solve(ri) + tsp_sol.solve(rk) - tsp_sol.solve(cur_sol.routes[i]) - tsp_sol.solve(cur_sol.routes[k]);

	retv += ts.over_demand_limit_penalty * calc_demand_mult(cur_sol.route_demand[k], cvrp2l.d[cur_sol.routes[i][j]] - cvrp2l.d[cur_sol.routes[k][l]], cvrp2l.vc);
	retv += ts.over_demand_limit_penalty * calc_demand_mult(cur_sol.route_demand[i], - cvrp2l.d[cur_sol.routes[i][j]] + cvrp2l.d[cur_sol.routes[k][l]], cvrp2l.vc);

	return retv;
}

double swap_neighborhood::best_move_improvement()
{
	best_move = {-1, -1, -1, -1};
	best_move_delta = infd;

	for (int i = 0; i < cur_sol.v; i++)
		for (int j = 1; j < cur_sol.routes[i].size(); j++)
			if (!ts.is_tabu(i, j))
				if (random_choice(this->sample))
					for (int k = 0; k < cur_sol.v; k++)
						if (i != k)
							for (int l = 1; l < cur_sol.routes[k].size(); l++)
								if (!ts.is_tabu(k, l))
									moves.push_back({i, j, k, l});

	random_shuffle(moves.begin(), moves.end());

	for (auto move : moves)
	{
		double move_delta = evaluate_swap_move(get<0>(move), get<1>(move), get<2>(move), get<3>(move));

		if (move_delta < best_move_delta)
		{
			best_move = move;
			best_move_delta = move_delta;
		}
	}

	return best_move_delta;
}

void swap_neighborhood::apply_best_move()
{
	int i = get<0>(best_move), j = get<1>(best_move), k = get<2>(best_move), l = get<3>(best_move);

	ts.insert_tabu(cur_sol.routes[i][j]);
	ts.insert_tabu(cur_sol.routes[k][l]);

	int x = cur_sol.routes[i][j];
	int y = cur_sol.routes[k][l];

	cur_sol.routes[k].erase(cur_sol.routes[k].begin() + l);
	cur_sol.routes[k].insert(lower_bound(cur_sol.routes[k].begin(), cur_sol.routes[k].end(), x), x);

	cur_sol.routes[i].erase(cur_sol.routes[i].begin() + j);
	cur_sol.routes[i].insert(lower_bound(cur_sol.routes[i].begin(), cur_sol.routes[i].end(), y), y);

	assert(is_sorted(cur_sol.routes[i].begin(), cur_sol.routes[i].end()));
	assert(is_sorted(cur_sol.routes[k].begin(), cur_sol.routes[k].end()));

	solution::evaluate_sol(cur_sol, tsp_sol, cvrp2l);
}

double eject_neighborhood::evaluate_eject_move(int i, int j, int k, int l, int m)
{
	assert(i != k && i != m && m != k);

	// if (cur_sol.route_demand[k] + cvrp2l.d[cur_sol.routes[i][j]] - cvrp2l.d[cur_sol.routes[k][l]] > cvrp2l.vc)
	// 	return infd;

	// if (cur_sol.route_demand[m] + cvrp2l.d[cur_sol.routes[k][l]] > cvrp2l.vc)
	// 	return infd;

	vector<int> ri = cur_sol.routes[i];
	ri.erase(ri.begin() + j);

	vector<int> rk = cur_sol.routes[k];
	rk.erase(rk.begin() + l);
	rk.insert(lower_bound(rk.begin(), rk.end(), cur_sol.routes[i][j]), cur_sol.routes[i][j]);

	vector<int> rm = cur_sol.routes[m];
	rm.insert(lower_bound(rm.begin(), rm.end(), cur_sol.routes[k][l]), cur_sol.routes[k][l]);

	if (!packing_solver.feasible(rk) || !packing_solver.feasible(rm))
		return infd;

	double retv = 0;
	retv += tsp_sol.solve(ri) + tsp_sol.solve(rk) + tsp_sol.solve(rm) - tsp_sol.solve(cur_sol.routes[i]) - tsp_sol.solve(cur_sol.routes[k]) - tsp_sol.solve(cur_sol.routes[m]);

	retv += ts.over_route_limit_penalty * (solution::get_over_route_limit_penalty(m, cvrp2l) - solution::get_over_route_limit_penalty(i, cvrp2l));

	retv += ts.over_demand_limit_penalty * calc_demand_mult(cur_sol.route_demand[m], cvrp2l.d[cur_sol.routes[k][l]], cvrp2l.vc);
	retv += ts.over_demand_limit_penalty * calc_demand_mult(cur_sol.route_demand[k], cvrp2l.d[cur_sol.routes[i][j]] - cvrp2l.d[cur_sol.routes[k][l]], cvrp2l.vc);
	retv += ts.over_demand_limit_penalty * calc_demand_mult(cur_sol.route_demand[i], -cvrp2l.d[cur_sol.routes[i][j]], cvrp2l.vc);

	return retv;
}

bool eject_neighborhood::relocation_infeasible(int i, int j, int k)
{
	if (cur_sol.route_demand[k] + cvrp2l.d[cur_sol.routes[i][j]] > cvrp2l.vc)
		return true;

	vector<int> to_route = cur_sol.routes[k];
	to_route.insert(lower_bound(to_route.begin(), to_route.end(), cur_sol.routes[i][j]), cur_sol.routes[i][j]);

	if (!packing_solver.feasible(to_route))
		return true;

	return false;	
}

double eject_neighborhood::best_move_improvement()
{
	best_move = {-1, -1, -1, -1, -1};
	best_move_delta = infd;

	for (int i = 0; i < cur_sol.v; i++)
		if (cur_sol.routes[i].size() > 2 || (i + 1) == cur_sol.v) // Only allow last route to be emptied.
			for (int j = 1; j < cur_sol.routes[i].size(); j++)
				if (!ts.is_tabu(i, j))
					if (random_choice(this->sample))
						for (int k = 0; k < cur_sol.v; k++)
							if (i != k) 
								for (int l = 1; l < cur_sol.routes[k].size(); l++)
									if (!ts.is_tabu(k, l))
										for (int m = 0; m < cur_sol.v + 1; m++) // Relocate to any currently used or one after last used.
											if (i != m && k != m)
											{
												if (cur_sol.routes[i].size() == 2 && m == cur_sol.v)
													continue; // Never empty a route to open a new one.

												moves.push_back({i, j, k, l, m});
											}

	random_shuffle(moves.begin(), moves.end());

	for (auto move : moves)
	{
		double move_delta = evaluate_eject_move(get<0>(move), get<1>(move), get<2>(move), get<3>(move), get<4>(move));

		if (move_delta < best_move_delta)
		{
			best_move = move;
			best_move_delta = move_delta;
		}
	}

	return best_move_delta;
}

void eject_neighborhood::apply_best_move()
{
	int i = get<0>(best_move), j = get<1>(best_move), k = get<2>(best_move), l = get<3>(best_move), m = get<4>(best_move);

	ts.insert_tabu(cur_sol.routes[i][j]);
	ts.insert_tabu(cur_sol.routes[k][l]);

	int x = cur_sol.routes[i][j];
	int y = cur_sol.routes[k][l];

	cur_sol.routes[k].erase(cur_sol.routes[k].begin() + l);
	cur_sol.routes[k].insert(lower_bound(cur_sol.routes[k].begin(), cur_sol.routes[k].end(), x), x);

	cur_sol.routes[i].erase(cur_sol.routes[i].begin() + j);

	cur_sol.routes[m].insert(lower_bound(cur_sol.routes[m].begin(), cur_sol.routes[m].end(), y), y);

	assert(is_sorted(cur_sol.routes[i].begin(), cur_sol.routes[i].end()));
	assert(is_sorted(cur_sol.routes[k].begin(), cur_sol.routes[k].end()));
	assert(is_sorted(cur_sol.routes[m].begin(), cur_sol.routes[m].end()));

	solution::evaluate_sol(cur_sol, tsp_sol, cvrp2l);
}


}; // namespace CVRP2L