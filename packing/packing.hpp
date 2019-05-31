#ifndef PACKING_HPP
#define PACKING_HPP

#include "../instance.hpp"

namespace CVRP2L
{
class packing_2d_solver
{
	instance &cvrp2l;

	int heuristic_pack(std::vector<rect> rl);

public:
	packing_2d_solver(instance &inst);

	bool feasible(const std::vector<int> &items_idx);
};
} // namespace CVRP2L

#endif