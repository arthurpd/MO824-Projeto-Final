#include "tabu_search.hpp"
#include "neighborhood.hpp"

namespace CVRP2L
{
using namespace std;
void solution::print()
{
	utils::log << "Value: " << val << " feasible: " << feasible << " v: " << v << endl;
	for (int i = 0; i < routes.size(); i++)
	{
		for (int j = 0; j < routes[i].size(); j++)
			utils::log << routes[i][j] << " ";
		utils::log << endl;
	}
}

int solution::evaluate_sol(solution &sol, tsp_solver &tsp_sol, instance &cvrp2l)
{
	int retv = 0;
	for (int i = 0; i < sol.routes.size(); i++)
		retv += tsp_sol.solve(sol.routes[i]);

	for (int i = 0; i < sol.routes.size(); i++)
	{
		sol.route_demand[i] = 0;
		for (int j = 0; j < sol.routes[i].size(); j++)
			sol.route_demand[i] += cvrp2l.d[sol.routes[i][j]];
	}

	for (int i = 0; i < sol.routes.size(); i++)
		if (sol.routes[i].size() > 1)
			sol.v = i + 1;

	if (sol.v <= cvrp2l.v)
		sol.feasible = true;
	else
		sol.feasible = false;

	return sol.val = retv;
}

int solution::get_over_route_limit_penalty(int i, instance &cvrp2l)
{
	if (i < cvrp2l.v)
		return 0;
	else
		return i - cvrp2l.v + 1;
}

void tabu_search::remove_old_tabu(int tabu_tenure)
{
	while (!tabu_list.empty() && iteration - tabu_list.front().first > tabu_tenure)
	{
		tabu_tab[tabu_list.front().second]--;
		tabu_list.pop_front();
	}
}

void tabu_search::insert_tabu(int x)
{
	tabu_tab[x]++;
	tabu_list.push_back(pii(iteration, x));
}

solution tabu_search::initial_solution()
{
	solution retv;
	retv.routes = vector<vector<int>>(cvrp2l.n, vector<int>(1, 0));
	retv.route_demand = vector<int>(cvrp2l.n, 0);

	// Last route is empty route.
	// Routes 0 to n-2 are {0, i}.
	vector<pair<double, int>> ord;
	for (int i = 1; i < cvrp2l.n; i++)
		ord.push_back({-cvrp2l.d[i] / (double)cvrp2l.vc, i});
	sort(ord.begin(), ord.end());

	for (int i = 1; i < cvrp2l.n; i++)
		retv.routes[i - 1].push_back(ord[i - 1].second);

	// This is the only trivially packing viable solution.
	solution::evaluate_sol(retv, tsp_sol, cvrp2l);

	return retv;
}

void tabu_search::init_run()
{
	cur_sol = best_sol = initial_solution();

	tabu_tab = vector<int>(cvrp2l.n, 0);
	tabu_list = list<pii>();

	iteration = 0;
	over_route_limit_penalty = 1e6;
}

double tabu_search::evaluate_relocation_move(int i, int j, int k)
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

	retv += over_route_limit_penalty * (solution::get_over_route_limit_penalty(k, cvrp2l) - solution::get_over_route_limit_penalty(i, cvrp2l));

	return retv;
}

void tabu_search::apply_relocation_move(int i, int j, int k)
{
	insert_tabu(cur_sol.routes[i][j]);
	cur_sol.routes[k].insert(lower_bound(cur_sol.routes[k].begin(), cur_sol.routes[k].end(), cur_sol.routes[i][j]), cur_sol.routes[i][j]);
	cur_sol.routes[i].erase(cur_sol.routes[i].begin() + j);
	assert(is_sorted(cur_sol.routes[k].begin(), cur_sol.routes[k].end()));

	solution::evaluate_sol(cur_sol, tsp_sol, cvrp2l);
}

bool tabu_search::neighborhood_move()
{
	relocation_neighborhood n1(*this, cur_sol, cvrp2l, tsp_sol, packing_solver);

	if (n1.best_move_improvement() < infd)
	{
		n1.apply_best_move();
		return true;
	}
	else
		return false;
}

tabu_search::tabu_search(instance &inst, tsp_solver &tsp, packing_2d_solver &packing) : cvrp2l(inst), tsp_sol(tsp), packing_solver(packing) {}

bool tabu_search::is_tabu(int x)
{
	return tabu_tab[x] == 0;
}

solution tabu_search::run(double time_limit, int tabu_tenure)
{
	init_run();
	utils::code_timer exec_time;

	while (exec_time.seconds() < time_limit)
	{
		remove_old_tabu(tabu_tenure);
		neighborhood_move();

		if ((best_sol.feasible && cur_sol.feasible && cur_sol.val < best_sol.val) || (!best_sol.feasible && !cur_sol.feasible && cur_sol.val < best_sol.val) || (!best_sol.feasible && cur_sol.feasible))
		{
			best_sol = cur_sol;
		}

		if (cur_sol.feasible)
			over_route_limit_penalty *= 0.95;
		else
			over_route_limit_penalty *= 1.05;
		over_route_limit_penalty = max(1e-12, min(over_route_limit_penalty, 1e12));

		utils::log << "iter " << iteration++ << " val " << best_sol.val << " feasible " << best_sol.feasible << " cur feas " << cur_sol.feasible << " penalty " << over_route_limit_penalty << endl;
	}

	return best_sol;
}

} // namespace CVRP2L