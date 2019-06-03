#ifndef PACKING_HPP
#define PACKING_HPP

#include "../instance.hpp"
#include "../utils.hpp"

namespace CVRP2L
{
class packing_2d_solver
{
	instance &cvrp2l;

	int heuristic_pack(std::vector<rect> rl);

	utils::trie_node<int> root;

	static const bool verbose = false;

public:
	packing_2d_solver(instance &inst);

	bool feasible(const std::vector<int> &items_idx);
};
} // namespace CVRP2L

#endif