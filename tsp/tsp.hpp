#ifndef TSP_HPP
#define TSP_HPP

#include "../instance.hpp"
#include "../utils.hpp"

namespace CVRP2L
{
class tsp_solver
{
	instance &cvrp2l;

	utils::trie_node<int> root;

public:
	bool verbose;

	tsp_solver(instance &inst);

	int solve(const std::vector<int> &vertices_idx);
};
} // namespace CVRP2L

#endif