#include "instance.hpp"

namespace CVRP2L
{
	void instance::flush_line(FILE *fp)
	{
		fscanf(fp, "%*[^\n]");
		fscanf(fp, "%*c");
	}

	instance::instance(std::string file_name)
	{
		FILE *fin = fopen(file_name.c_str(), "r");
		flush_line(fin);
		flush_line(fin);
		fscanf(fin, " %d", &n);
		n++;
		flush_line(fin);
		fscanf(fin, " %d", &v);
		flush_line(fin);
		fscanf(fin, "        ");
		flush_line(fin);
		fscanf(fin, "        ");
		flush_line(fin);

		fscanf(fin, " %d %d %d", &vc, &vh, &vw);
		flush_line(fin);
		fscanf(fin, "        ");
		flush_line(fin);
		fscanf(fin, "        ");

		d.resize(n);
		m.resize(n);
		dist = std::vector<std::vector<int>> (n, std::vector<int>(n, 0));
		items.resize(n);

		std::vector<double> x(n),  y(n);
		for (int i = 0; i < n; i++)
		{
			double tmp;
			fscanf(fin, " %*d %lf %lf %lf", &x[i], &y[i], &tmp);
			d[i] = tmp;
		}

		for (int i = 0; i < n; i++)
			for (int j = 0; j < n; j++)
				dist[i][j] = sqrt((x[i] - x[j])*(x[i] - x[j]) + (y[i] - y[j])*(y[i] - y[j]));

		flush_line(fin);
		fscanf(fin, "        ");
		flush_line(fin);
		fscanf(fin, "        ");

		for (int i = 0; i < n; i++)
		{
			fscanf(fin, " %*d %d", &m[i]);
			items[i].resize(m[i]);
			for (int j = 0; j < m[i]; j++)
				fscanf(fin, " %d %d", &items[i][j].first, &items[i][j].second);
		}

		fclose(fin);
	}

	void instance::print()
	{
		printf("%d %d\n", n, v);
		printf("%d %d %d\n", vc, vh, vw);
		for (int i = 0; i < n; i++, printf("\n"))
			for (int j = 0; j < n; j++)
				printf("%d ", dist[i][j]);

		for (int i = 0; i < n; i++, printf("\n"))
			for (int j = 0; j < m[i]; j++)
				printf("(%d %d) ", items[i][j].first, items[i][j].second);

		for (int i = 0; i < n; i++)
			printf("%d ", d[i]);
		printf("\n");
	}
}
