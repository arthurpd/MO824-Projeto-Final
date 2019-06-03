#ifndef INSTANCE_HPP
#define INSTANCE_HPP

#include <bits/stdc++.h>

typedef std::pair<int, int> pii;

namespace CVRP2L
{

struct rect
{
	int w, h;

	int area() const { return w * h; }
};

class instance
{
	static void flush_line(FILE *fp);

	static const bool verbose = true;

public:
	// Number of vertices.
	int n;

	// Number of vehicles.
	int v;

	// Vehicle capacity.
	int vc;

	// Vehicle height.
	int vh;

	// Vehicle width.
	int vw;

	// Demand per customer (1 to n-1).
	std::vector<int> d;

	// Number of items per customer.
	std::vector<int> m;

	// List of items per customer.
	std::vector<std::vector<rect>> items;

	// Distance matrix.
	std::vector<std::vector<int>> dist;

	instance(std::string file_name);

	void print();
};
} // namespace CVRP2L

#endif
