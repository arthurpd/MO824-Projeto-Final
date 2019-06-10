#include "packing.hpp"
#include "../utils.hpp"

namespace CVRP2L
{
struct mos
{
	int xi, yi, xf, yf;

	bool fits(rect r) const
	{
		return (xf - xi >= r.w) && (yf - yi >= r.h);
	}

	int area() const
	{
		return std::max(0, xf - xi) * std::max(0, yf - yi);
	}

	mos intersect(const mos &rhs) const
	{
		return {std::max(xi, rhs.xi), std::max(yi, rhs.yi), std::min(xf, rhs.xf), std::min(yf, rhs.yf)};
	}

	bool intersects(const mos &rhs) const
	{
		return this->intersect(rhs).area() > 0;
	}

	bool contains(const mos &rhs) const
	{
		return this->intersect(rhs).area() == rhs.area();
	}

	bool dominates(const mos &rhs) const
	{
		return this->contains(rhs);
	}

	bool operator<(const mos &rhs) const
	{
		return (yi != rhs.yi) ? (yi < rhs.yi) : (xi < rhs.xi);
	}
};

packing_2d_solver::packing_2d_solver(instance &inst) : cvrp2l(inst)
{
}

void insert_mos(std::set<mos> &space_list, mos m)
{
	std::vector<mos> removal_list;
	for (mos x : space_list)
	{
		if (x.dominates(m))
			return;

		if (m.dominates(x))
			removal_list.push_back(x);
	}

	for (mos x : removal_list)
	{
		space_list.erase(x);
	}

	space_list.insert(m);
}

void update_space_list(std::set<mos> &space_list, mos m, rect r)
{
	// We use the fact the m is the most bottom-left MOS.
	std::vector<mos> removal_list;
	std::vector<mos> insertion_list;
	insertion_list.push_back({m.xi + r.w, m.yi, m.xf, m.yf});
	insertion_list.push_back({m.xi, m.yi + r.h, m.xf, m.yf});

	mos m_rect = {m.xi, m.yi, m.xi + r.w, m.yi + r.h};
	for (mos x : space_list)
	{
		if (m_rect.intersects(x))
		{
			removal_list.push_back(x);
			insertion_list.push_back({x.xi, x.yi, m_rect.xi, x.yf});
			insertion_list.push_back({x.xi, m_rect.yf, x.xf, x.yf});
			insertion_list.push_back({m_rect.xf, x.yi, x.xf, x.yf});
		}
	}

	for (mos x : removal_list)
		space_list.erase(x);

	for (mos x : insertion_list)
		if (x.area() > 0)
			insert_mos(space_list, x);
}

int packing_2d_solver::heuristic_pack(std::vector<rect> rl)
{
	std::vector<std::vector<char>> mat(cvrp2l.vw, std::vector<char>(cvrp2l.vh, '.'));
	int covered_area = 0;
	std::set<mos> space_list;
	space_list.insert({0, 0, cvrp2l.vw, cvrp2l.vh});

	int cnt = 0;
	while (!space_list.empty() && !rl.empty())
	{
		mos m = *space_list.begin();
		space_list.erase(space_list.begin());

		int best_fit = -1;
		int fitness = 0x3f3f3f3f;
		for (int i = 0; i < rl.size(); i++)
		{
			if (m.fits(rl[i]))
			{
				std::set<mos> space_list_tmp = space_list;
				update_space_list(space_list_tmp, m, rl[i]);
				if (space_list_tmp.size() < fitness)
				{
					best_fit = i;
					fitness = space_list_tmp.size();
				}
			}
		}

		if (best_fit >= 0)
		{
			if (verbose)
				for (int xi = m.xi; xi < m.xi + rl[best_fit].w; xi++)
					for (int yi = m.yi; yi < m.yi + rl[best_fit].h; yi++)
						mat[xi][yi] = 'a' + cnt;

			cnt++;

			covered_area += rl[best_fit].area();
			update_space_list(space_list, m, rl[best_fit]);
			rl.erase(rl.begin() + best_fit);
		}
	}

	if (verbose)
	{
		for (int i = 0; i < cvrp2l.vw; i++)
		{
			for (int j = 0; j < cvrp2l.vh; j++)
				utils::log << mat[i][j];
			utils::log << std::endl;
		}

		utils::log << covered_area << std::endl;
	}

	return covered_area;
}

bool packing_2d_solver::feasible(const std::vector<int> &items_idx)
{
	utils::code_timer t;

	int total_area = 0;
	std::vector<rect> rl;
	for (int i : items_idx)
		for (rect r : cvrp2l.items[i])
		{
			rl.push_back(r);
			total_area += r.area();
		}

	if (total_area > cvrp2l.vw * cvrp2l.vh)
	{
		utils::total_packing_time += t.seconds();
		return false;
	}

	assert(is_sorted(items_idx.begin(), items_idx.end()));
	int **x = root.get(0, items_idx);
	if (*x != nullptr)
	{
		if (**x == 1)
		{
			utils::total_packing_time += t.seconds();
			return true;
		}
		else if (**x == 0)
		{
			utils::total_packing_time += t.seconds();
			return false;
		}
		else
			(**x)++;
	}
	else
	{
		*x = new int(-std::max((int)(2 * rl.size()), 200 - ((200 * total_area) / (cvrp2l.vw * cvrp2l.vh))));
		assert(**x < 0);
	}

	std::random_shuffle(rl.begin(), rl.end());
	int best_area = heuristic_pack(rl);

	std::vector<std::function<bool(rect, rect)>> sort_criteria = {[](rect a, rect b) { return a.area() > b.area(); }, [](rect a, rect b) { return a.w > b.w; }, [](rect a, rect b) { return a.h > b.h; }};

	for (auto criterion : sort_criteria)
	{
		std::vector<rect> rl_tmp = rl;
		sort(rl_tmp.begin(), rl_tmp.end(), criterion);
		int tmp_area = heuristic_pack(rl_tmp);
		if (tmp_area > best_area)
		{
			best_area = tmp_area;
			rl = rl_tmp;
		}
	}

	if (verbose)
		utils::log << "\nStarting random search\n\n";

	int max_iter = rl.size();
	int iter = 0;
	while (iter < max_iter && best_area < total_area)
	{
		iter++;

		std::vector<rect> rl_tmp = rl;
		std::swap(rl_tmp[rand() % rl_tmp.size()], rl_tmp[rand() % rl_tmp.size()]);
		int tmp_area = heuristic_pack(rl_tmp);
		if (tmp_area > best_area)
		{
			best_area = tmp_area;
			rl = rl_tmp;
			iter = 0;
		}
	}

	if (best_area == total_area)
		**x = 1;

	utils::total_packing_time += t.seconds();
	return best_area == total_area;
}

} // namespace CVRP2L
