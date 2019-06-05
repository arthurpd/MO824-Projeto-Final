#include "neighborhood.hpp"

namespace CVRP2L
{
using namespace std;

double relocation_neighborhood::evaluate_relocation_move(int i, int j, int k)
{
	assert(i != k);

	if (cur_sol.route_demand[k] + cvrp2l.d[cur_sol.routes[i][j]] > cvrp2l.vc)
		return infd;

	vector<int> to_route = cur_sol.routes[k];
	to_route.insert(lower_bound(to_route.begin(), to_route.end(), cur_sol.routes[i][j]), cur_sol.routes[i][j]);

	if (!packing_solver.feasible(to_route))
		return infd;

	vector<int> from_route = cur_sol.routes[i];
	from_route.erase(from_route.begin() + j);

	double retv = 0;
	retv += tsp_sol.solve(from_route) + tsp_sol.solve(to_route) - tsp_sol.solve(cur_sol.routes[i]) - tsp_sol.solve(cur_sol.routes[k]);

	retv += ts.over_route_limit_penalty * (solution::get_over_route_limit_penalty(k, cvrp2l) - solution::get_over_route_limit_penalty(i, cvrp2l));

	return retv;
}

double relocation_neighborhood::best_move_improvement()
{
	best_move = {-1, -1, -1};
	best_move_delta = infd;

	for (int i = 0; i < cur_sol.v; i++)
		if (cur_sol.routes[i].size() > 2 || (i + 1) == cur_sol.v) // Only allow last route to be emptied.
			for (int j = 1; j < cur_sol.routes[i].size(); j++)
				if (ts.is_tabu(cur_sol.routes[i][j]))
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
}; // namespace CVRP2L