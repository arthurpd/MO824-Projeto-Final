#include "instance.hpp"
#include "packing/packing.hpp"
#include "tsp/tsp.hpp"
#include "utils.hpp"

std::ofstream utils::log;
double utils::total_tsp_time = 0;
double utils::total_packing_time = 0;

namespace CVRP2L
{
using namespace std;

const double infd = 1e12;

struct solution
{
	vector<vector<int>> routes;

	// Sum of TSP solution for all routes.
	int val;

	// If the solution uses the required amount of vehicles. (The solution is always packing feasible)
	bool feasible;

	// Number of vehicles being used
	int v;

	void print()
	{
		utils::log << "Value: " << val << " feasible: " << feasible << " v: " << v << endl;
		for (int i = 0; i < routes.size(); i++)
		{
			for (int j = 0; j < routes[i].size(); j++)
				utils::log << routes[i][j] << " ";
			utils::log << endl;
		}
	}
};

class local_search
{
	instance cvrp2l;

	packing_2d_solver packing_solver;

	tsp_solver tsp_sol;

	solution cur_sol;

	solution best_sol;

	// tabu_tab[i] is the number of occurrences of element i in the tabu_list (could be more than 1 due to aspiration criteria).
	vector<int> tabu_tab;

	list<pii> tabu_list;

	int iteration;

	double over_route_limit_penalty = 1e6;

	void remove_old_tabu(int tabu_tenure)
	{
		while (!tabu_list.empty() && iteration - tabu_list.front().first > tabu_tenure)
		{
			tabu_tab[tabu_list.front().second]--;
			tabu_list.pop_front();
		}
	}

	void insert_tabu(int x)
	{
		tabu_tab[x]++;
		tabu_list.push_back(pii(iteration, x));
	}

	int evaluate_sol(solution &sol)
	{
		int retv = 0;
		for (int i = 0; i < sol.routes.size(); i++)
			retv += tsp_sol.solve(sol.routes[i]);

		for (int i = 0; i < sol.routes.size(); i++)
			if (sol.routes[i].size() > 1)
				sol.v = i + 1;

		if (sol.v <= cvrp2l.v)
			sol.feasible = true;
		else
			sol.feasible = false;

		return sol.val = retv;
	}

	solution initial_solution()
	{
		solution retv;
		retv.routes = vector<vector<int>>(cvrp2l.n, vector<int>(1, 0));

		// Last route is empty route.
		// Routes 0 to n-2 are {0, i}.
		for (int i = 1; i < cvrp2l.n; i++)
			retv.routes[i - 1].push_back(i);

		// This is the only trivially packing viable solution.
		evaluate_sol(retv);

		return retv;
	}

	void init_run()
	{
		cur_sol = best_sol = initial_solution();

		tabu_tab = vector<int>(cvrp2l.n, 0);
		tabu_list = list<pii>();

		iteration = 0;
	}

	int get_over_route_limit_penalty(int i)
	{
		if (i < cvrp2l.v)
			return 0;
		else
			return i - cvrp2l.v + 1;
	}

	double evaluate_relocation_move(int i, int j, int k)
	{
		assert(i != k);

		// Don't allow empty route in the vehicles which must be used.
		// if (i < cvrp2l.v && cur_sol.routes[i].size() == 2)
		// 	return infd;

		vector<int> to_route = cur_sol.routes[k];
		to_route.insert(lower_bound(to_route.begin(), to_route.end(), cur_sol.routes[i][j]), cur_sol.routes[i][j]);

		int route_load = 0;
		for (int i = 0; i < to_route.size(); i++)
			route_load += cvrp2l.d[to_route[i]];

		if (route_load > cvrp2l.vc)
			return infd;

		if (!packing_solver.feasible(to_route))
			return infd;

		vector<int> from_route = cur_sol.routes[i];
		from_route.erase(from_route.begin() + j);

		double retv = 0;
		retv += tsp_sol.solve(from_route) + tsp_sol.solve(to_route) - tsp_sol.solve(cur_sol.routes[i]) - tsp_sol.solve(cur_sol.routes[k]);

		retv += over_route_limit_penalty * (get_over_route_limit_penalty(k) - get_over_route_limit_penalty(i));

		return retv;
	}

	void apply_relocation_move(int i, int j, int k)
	{
		insert_tabu(cur_sol.routes[i][j]);
		cur_sol.routes[k].insert(lower_bound(cur_sol.routes[k].begin(), cur_sol.routes[k].end(), cur_sol.routes[i][j]), cur_sol.routes[i][j]);
		cur_sol.routes[i].erase(cur_sol.routes[i].begin() + j);
		assert(is_sorted(cur_sol.routes[k].begin(), cur_sol.routes[k].end()));

		evaluate_sol(cur_sol);
	}

	bool neighborhood_move()
	{
		vector<tuple<int, int, int>> neighborhood;
		tuple<int, int, int> best_move = {-1, -1, -1};
		double best_move_delta = infd;

		for (int i = 0; i < cur_sol.v; i++)
			for (int j = 1; j < cur_sol.routes[i].size(); j++)
				if (tabu_tab[cur_sol.routes[i][j]] == 0)
					for (int k = 0; k < cur_sol.v + 1; k++)
						if (i != k)
							neighborhood.push_back({i, j, k});

		random_shuffle(neighborhood.begin(), neighborhood.end());
		for (auto move : neighborhood)
		{
			double move_delta = evaluate_relocation_move(get<0>(move), get<1>(move), get<2>(move));
			// Relocate to any currently used or one after last used.
			if (move_delta < best_move_delta)
			{
				best_move = move;
				best_move_delta = move_delta;
			}
		}

		// for (int i = 0; i < cur_sol.v; i++)
		// 	for (int j = 1; j < cur_sol.routes[i].size(); j++)
		// 		if (tabu_tab[cur_sol.routes[i][j]] == 0)
		// 			for (int k = 0; k < cur_sol.v + 1; k++)
		// 				if (i != k)
		// 				{
		// 				}

		if (best_move_delta < infd)
		{
			apply_relocation_move(get<0>(best_move), get<1>(best_move), get<2>(best_move));
			return true;
		}
		else
			return false;
	}

public:
	local_search(instance &inst) : cvrp2l(inst), tsp_sol(inst), packing_solver(inst) { tsp_sol.verbose = false; }

	solution run(double time_limit, int tabu_tenure)
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
				over_route_limit_penalty *= 0.9;
			else
				over_route_limit_penalty *= 1.1;

			utils::log << "iter " << iteration++ << " val " << best_sol.val << " feasible " << best_sol.feasible << " cur feas " << cur_sol.feasible << " penalty " << over_route_limit_penalty << endl;
		}

		return best_sol;
	}
};

} // namespace CVRP2L

int main(int argc, char **argv)
{
	using namespace std;
	using namespace utils;

	// To hide concorde output.
	freopen("/dev/null", "w", stdout);
	utils::log.open("2lcvrp_log.txt", std::ios::out | std::ios::trunc);

	assert(argc == 3);
	CVRP2L::instance cvrp2l(argv[1]);
	cvrp2l.print();

	CVRP2L::packing_2d_solver packing_solver(cvrp2l);
	std::vector<int> idx = {0, 1, 2, 3, 4};
	utils::log << packing_solver.feasible(idx) << endl;

	CVRP2L::tsp_solver tsp_solver(cvrp2l);
	tsp_solver.verbose = false;
	utils::log << tsp_solver.solve(idx) << endl;

	CVRP2L::local_search search(cvrp2l);
	CVRP2L::solution best_sol = search.run(atol(argv[2]), ceil(cvrp2l.n / 8.0));
	best_sol.print();

	if (best_sol.v != cvrp2l.v)
	{
		utils::log << "Number of vehicles does not match" << endl;
	}

	tsp_solver.verbose = false;

	int tmp = 0;
	for (int i = 0; i < best_sol.routes.size(); i++)
		tmp += tsp_solver.solve(best_sol.routes[i]);

	if (tmp != best_sol.val)
	{
		utils::log << "Value does not match" << endl;
	}

	for (int i = 0; i < best_sol.routes.size(); i++)
	{
		bool ok = false;
		for (int j = 0; j < 1000; j++)
			if (packing_solver.feasible(best_sol.routes[i]))
				ok = true;
		if (!ok)
			utils::log << "Infeasible packing" << endl;

		tmp = 0;
		for (int j = 0; j < best_sol.routes[i].size(); j++)
			tmp += cvrp2l.d[best_sol.routes[i][j]];
		if (tmp > cvrp2l.vc)
			utils::log << "Vehicles capacity violated" << endl;
	}

	utils::log << "Total tsp time " << utils::total_tsp_time << endl;
	utils::log << "Total packing time " << utils::total_packing_time << endl;

	utils::log.close();

	return 0;
}