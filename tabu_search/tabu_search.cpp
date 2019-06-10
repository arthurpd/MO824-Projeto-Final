#include "tabu_search.hpp"
#include "neighborhood.hpp"

namespace CVRP2L
{
using namespace std;
void solution::print()
{
	utils::log << "Value: " << val << " vehicle_feasible: " << vehicle_feasible << " demand_feasible " << demand_feasible << " v: " << v << endl;
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

	sol.demand_feasible = true;
	for (int i = 0; i < sol.routes.size(); i++)
	{
		sol.route_demand[i] = 0;
		for (int j = 0; j < sol.routes[i].size(); j++)
			sol.route_demand[i] += cvrp2l.d[sol.routes[i][j]];
		if (sol.route_demand[i] > cvrp2l.vc)
			sol.demand_feasible = false;
	}

	for (int i = 0; i < sol.routes.size(); i++)
		if (sol.routes[i].size() > 1)
			sol.v = i + 1;

	if (sol.v <= cvrp2l.v)
		sol.vehicle_feasible = true;
	else
		sol.vehicle_feasible = false;

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

	over_demand_limit_penalty = 1e6;
}

bool tabu_search::neighborhood_move(bool include_swap, bool include_eject)
{
	relocation_neighborhood nr(*this, cur_sol, cvrp2l, tsp_sol, packing_solver);

	swap_neighborhood ns(*this, cur_sol, cvrp2l, tsp_sol, packing_solver, include_swap ? 1.0 : (2.0 / cvrp2l.n));

	eject_neighborhood ne(*this, cur_sol, cvrp2l, tsp_sol, packing_solver, include_eject ? 1.0 : 0.0);

	vector<pair<double, neighborhood *>> neighborhoods;
	neighborhoods.push_back({nr.best_move_improvement(), &nr});
	neighborhoods.push_back({ns.best_move_improvement(), &ns});
	neighborhoods.push_back({ne.best_move_improvement(), &ne});

	utils::log << neighborhoods.front().first << " " << (neighborhoods.size() > 1 ? neighborhoods[1] : neighborhoods[0]).first << " " << neighborhoods.back().first << endl;

	sort(neighborhoods.begin(), neighborhoods.end());

	if (neighborhoods.front().first < infd)
	{
		utils::log << cur_sol.val << " ";
		neighborhoods.front().second->apply_best_move();
		utils::log << cur_sol.val << endl;
		return true;
	}
	else
		return false;
}

tabu_search::tabu_search(instance &inst, tsp_solver &tsp, packing_2d_solver &packing) : cvrp2l(inst), tsp_sol(tsp), packing_solver(packing) {}

bool tabu_search::is_tabu(int i, int j)
{
	return tabu_tab[cur_sol.routes[i][j]] != 0;
}

int pick(int a, int b)
{
	return a + (rand() % (b - a + 1));
}

solution tabu_search::run(double time_limit, int tabu_tenure)
{
	init_run();
	utils::code_timer exec_time;

	while (exec_time.seconds() < time_limit)
	{
		if (iteration % 100 == 0)
			tabu_tenure = pick(ceil(cvrp2l.n/16.0), ceil(cvrp2l.n/4.0));

		remove_old_tabu(tabu_tenure);


		if (best_sol.feasible() && (iteration / 100) % 10 == 0)
		{
			if (iteration % 100 >= 90)
				neighborhood_move(true, true);
			else
			{
				neighborhood_move(true, false);
			}
		}
		else
		{
			neighborhood_move(false, false);
		}


		if ((best_sol.feasible() && cur_sol.feasible() && cur_sol.val < best_sol.val) || (!best_sol.feasible() && !cur_sol.feasible() && cur_sol.val < best_sol.val) || (!best_sol.feasible() && cur_sol.feasible()))
		{
			best_sol = cur_sol;
		}

		if (cur_sol.vehicle_feasible)
			over_route_limit_penalty *= 0.95;
		else
			over_route_limit_penalty *= 1.05;
		over_route_limit_penalty = max(1e-12, min(over_route_limit_penalty, 1e12));

		if (cur_sol.demand_feasible)
			over_demand_limit_penalty *= 0.95;
		else
			over_demand_limit_penalty *= 1.05;
		over_demand_limit_penalty = max(1e-12, min(over_demand_limit_penalty, 1e12));

		iteration++;
		utils::log << "iter " << iteration << " best_val " << best_sol.val << " feasible " << best_sol.feasible() << 
			" cur_val " << cur_sol.val << " cur_df " << cur_sol.demand_feasible << " cur vf " << cur_sol.vehicle_feasible<< " dpenalty " << over_demand_limit_penalty << " vpenalty " << over_route_limit_penalty << endl;
	}


	return best_sol;
}

} // namespace CVRP2L