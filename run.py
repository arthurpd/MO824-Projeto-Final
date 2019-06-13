import subprocess

for i in range(21, 37):
	lp = []
	for j in range(2, 6):
	    lp.append(subprocess.Popen(f'./main.exe instances/2l_cvrp{i:02}{j:02}.txt 3600 results/2l_cvrp{i:02}{j:02}.log results/2l_cvrp{i:02}{j:02}.sol'.split()))
	for p in lp:
		p.wait()

