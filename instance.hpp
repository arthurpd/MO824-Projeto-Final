#include <bits/stdc++.h>

typedef std::pair<int,int> pii;

namespace CVRP2L
{
class instance
{
	static void flush_line(FILE *fp);
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

	// Demand per customer.
	std::vector<int> d;

	// Number of items per customer.
	std::vector<int> m;

	// List of items per customer.
	std::vector<std::vector<pii>> items;

	// Distance matrix.
	std::vector<std::vector<int>> dist;

	instance(std::string file_name);

	void print();
};
} // namespace CVRP2L
