#include "tsp.hpp"
#include "../utils.hpp"

extern "C"
{
#include <concorde.h>
}

namespace CVRP2L
{

// Function that calls Concorde solver functions.
void solving_tsp_concorde(const std::vector<std::vector<int>> &dist, std::vector<int> &tour)
{
	// Creating a sequential tour.
	for (int i = 0; i < tour.size(); i++)
	{
		tour[i] = i;
	}

	assert(dist.size() > 4);

	if (dist.size() > 4)
	{ // TSP for more than 4 cities
		int rval = 0;
		int seed = rand();
		double szeit, optval, *in_val, *timebound;
		int ncount, success, foundtour, hit_timebound = 0;
		int *in_tour = (int *)NULL;
		int *out_tour = (int *)NULL;
		CCrandstate rstate;
		char *name = (char *)NULL;
		static int silent = 1;
		CCutil_sprand(seed, &rstate);
		in_val = (double *)NULL;
		timebound = (double *)NULL;
		ncount = dist.size();
		int ecount = (ncount * (ncount - 1)) / 2;
		int *elist = new int[ecount * 2];
		int *elen = new int[ecount];
		int edge = 0;
		int edgeWeight = 0;
		for (int i = 0; i < ncount; i++)
		{
			for (int j = i + 1; j < ncount; j++)
			{
				if (i != j)
				{
					elist[edge] = i;
					elist[edge + 1] = j;
					elen[edgeWeight] = dist[i][j];
					edgeWeight++;
					edge = edge + 2;
				}
			}
		}
		out_tour = CC_SAFE_MALLOC(ncount, int);
		name = CCtsp_problabel(" ");
		CCdatagroup dat;
		CCutil_init_datagroup(&dat);
		rval = CCutil_graph2dat_matrix(ncount, ecount, elist, elen, 1, &dat);
		rval = CCtsp_solve_dat(ncount, &dat, in_tour, out_tour, NULL, &optval, &success, &foundtour, name, timebound, &hit_timebound, silent, &rstate);
		for (int i = 0; i < ncount; i++)
		{
			tour[i] = out_tour[i];
		}

		assert(success); // Make sure optimal solution found.

		szeit = CCutil_zeit();
		CC_IFFREE(elist, int);
		CC_IFFREE(elen, int);
		CC_IFFREE(out_tour, int);
		CC_IFFREE(name, char);
	}
}

void solve_tsp_brute(const std::vector<std::vector<int>> &dist, std::vector<int> &tour)
{
	// Creating a sequential tour.
	for (int i = 0; i < tour.size(); i++)
		tour[i] = i;

	// No more than 4 vertices.
	std::vector<int> best_tour;
	int best_val = 0;
	do
	{
		int tmp = 0;
		for (int i = 0; i < tour.size(); i++)
			tmp += dist[tour[i]][tour[(i + 1) % tour.size()]];

		if (best_tour.size() == 0 || tmp < best_val)
		{
			best_val = tmp;
			best_tour = tour;
		}

	} while (next_permutation(tour.begin(), tour.end()));

	tour = best_tour;
}

int solve_tsp_dp(const std::vector<std::vector<int>> &dist, std::vector<int> &tour)
{
	using namespace std;
	int n = tour.size();
	vector<vector<int>> tab = vector<vector<int>>(n, vector<int>(1 << (n - 1), 0x3f3f3f3f));

	vector<int> order;
	for (int i = 0; i < (1 << (n - 1)); i++)
		order.push_back(i);
	sort(order.begin(), order.end(), [](int a, int b) -> bool { return __builtin_popcount(a) < __builtin_popcount(b); });

	for (int i = 0; i < order.size(); i++)
	{
		int mask = order[i];
		for (int a = 0; a < n; a++)
		{
			if (mask & (1 << a))
				continue;

			if (mask == 0)
				tab[a][mask] = dist[a][n - 1];

			for (int j = 0; j < n; j++)
				if (mask & (1 << j))
					tab[a][mask] = min(tab[j][mask - (1 << j)] + dist[a][j], tab[a][mask]);
		}
	}

	return tab[n - 1][(1 << (n - 1)) - 1];
}

tsp_solver::tsp_solver(instance &inst) : cvrp2l(inst)
{
}

int tsp_solver::solve(const std::vector<int> &vertices_idx)
{
	using namespace std;
	utils::code_timer t;

	assert(is_sorted(vertices_idx.begin(), vertices_idx.end()));
	int **x = root.get(0, vertices_idx);
	if (*x != nullptr)
	{
		utils::total_tsp_time += t.seconds();
		return **x;
	}

	int n = vertices_idx.size();
	vector<vector<int>> dist(n, vector<int>(n, 0));
	vector<int> tour(n, 0);

	for (int i = 0; i < n; i++)
		for (int j = 0; j < n; j++)
			dist[i][j] = cvrp2l.dist[vertices_idx[i]][vertices_idx[j]];

	int ans = -1;
	if (n >= 15)
	 	solving_tsp_concorde(dist, tour);
	else if (n >= 4)
	 	ans = solve_tsp_dp(dist, tour);
	else
		solve_tsp_brute(dist, tour);

	if (ans == -1)
	{
		ans = 0;
		for (int i = 0; i < tour.size(); i++)
		{
			if (verbose)
				utils::log << vertices_idx[tour[i]] << " ";
			ans += dist[tour[i]][tour[(i + 1) % tour.size()]];
		}
	}

	if (verbose)
		utils::log << endl
				   << ans << endl;

	if (verbose)
		utils::log << "TSP solve took: " << t.seconds() << "s" << endl;

	*x = new int(ans);
	utils::total_tsp_time += t.seconds();
	return ans;
}
} // namespace CVRP2L