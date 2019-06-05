#ifndef UTILS_HPP
#define UTILS_HPP

#include <bits/stdc++.h>

namespace utils
{
extern std::ofstream log;

extern double total_tsp_time;

extern double total_packing_time;

using namespace std::chrono;
class code_timer
{
	high_resolution_clock::time_point begin;

public:
	code_timer()
	{
		begin = high_resolution_clock::now();
	}

	double seconds()
	{
		return duration_cast<duration<double>>(high_resolution_clock::now() - begin).count();
	}
};

template <class T>
struct trie_node
{
	T *val;
	std::unordered_map<int, trie_node *> tab;

	trie_node() : val(nullptr)
	{
	}

	T **get(int i, const std::vector<int> &key)
	{
		if (i == key.size())
			return &val;

		trie_node *&prox = tab[key[i]];
		if (prox == NULL)
			prox = new trie_node();

		return prox->get(i + 1, key);
	}
};

} // namespace utils

#endif