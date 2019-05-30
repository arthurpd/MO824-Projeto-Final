extern "C" { 
	#include <concorde.h> 
}

#include <time.h>
#include <vector>
#include <iostream>

//function that calls Concorde solver functions
void solving_tsp_concorde(const std::vector<std::vector<int>> &dist, std::vector<int> &tour){
	//creating a sequential tour
	for(int i = 0; i < tour.size(); i++){
		tour[i] = i;
	}

	if(dist.size() > 4 ){ //TSP for more than 4 cities
		int rval = 0;
		int seed = rand();
		double szeit, optval, *in_val, *timebound;
		int ncount, success, foundtour, hit_timebound = 0;
		int *in_tour = (int *) NULL;
		int *out_tour = (int *) NULL;
		CCrandstate rstate;
		char *name = (char *) NULL;
		static int silent = 2;
		CCutil_sprand(seed, &rstate);
		in_val = (double *) NULL;
		timebound = (double *) NULL;
		ncount = dist.size();
		int ecount = (ncount * (ncount - 1)) / 2;
		int *elist = new int[ecount * 2];
		int *elen = new int[ecount];
		int edge = 0;
		int edgeWeight = 0;
		for (int i = 0; i < ncount; i++) {
			for (int j = i + 1; j < ncount; j++) {
				if (i != j) {
					elist[edge] = i;
					elist[edge + 1] = j;
					elen[edgeWeight] = dist[i][j];
					edgeWeight++;
					edge = edge + 2;
				}
			}
		}
		out_tour = CC_SAFE_MALLOC (ncount, int);
		name = CCtsp_problabel(" ");
		CCdatagroup dat;
		CCutil_init_datagroup (&dat);
		rval = CCutil_graph2dat_matrix (ncount, ecount, elist, elen, 1, &dat);
		rval = CCtsp_solve_dat (ncount, &dat, in_tour, out_tour, NULL, &optval, &success, &foundtour, name, timebound, &hit_timebound, silent, &rstate);
		for (int i = 0; i < ncount; i++) {
			tour[i] = out_tour[i];
		}

		szeit = CCutil_zeit();
		CC_IFFREE (elist, int);
		CC_IFFREE (elen, int);
		CC_IFFREE (out_tour, int);
		CC_IFFREE (name, char);
	}
}


int main(int argc, char **argv){
	// freopen("/dev/null", "w", stdout);
	using namespace std;

	int n = atoll(argv[1]);
	vector<vector<int>> dist(n, vector<int>(n, 10));
	vector<int> tour(n, 0);

	vector<int> x(n, 0);
	vector<int> y(n, 0);
	for (int i = 0; i < n; i++)
	{
		x[i] = rand() % 1000;
		y[i] = rand() % 1000;
	}

	for (int i = 0; i < n; i++)
		for (int j = 0; j < n; j++)
			if (i != j)
				dist[i][j] = sqrt((x[i] - x[j])*(x[i] - x[j]) + (y[i] - y[j])*(y[i] - y[j]));

	for (int k = 0; k < n; k++)
		for (int i = 0; i < n; i++)
			for (int j = 0; j < n; j++)
				dist[i][j] = min(dist[i][j], dist[i][k] + dist[k][j]);
	
	cout << endl;
	cout << endl;

	clock_t begin = clock();
	solving_tsp_concorde(dist, tour);
	clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;	

	for (int i = 0; i < tour.size(); i++){
		cout << tour[i] << " ";
	}

	cout << endl;
	cout << "That took: " << elapsed_secs << "s" << endl;

	return 0;
}
