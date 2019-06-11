import subprocess

tsp_time = []
packing_time = []
res = []
n = 20
for i in range(1, n + 1):
	tsp_time.append([])
	packing_time.append([])
	res.append([])
	for j in range(1, 6):
		f = open(f'results/2l_cvrp{i:02}{j:02}.log', 'r')
		lines = f.readlines()
		tsp_time[-1].append(lines[-2][len('Total tsp time '):])
		packing_time[-1].append(lines[-1][len('Total packing time '):])
		f = open(f'results/2l_cvrp{i:02}{j:02}.sol', 'r')
		res[-1].append(f.read())
	print(i)

print('/n'.join([','.join(line) for line in tsp_time]))
print('/n'.join([','.join(line) for line in packing_time]))
print('/n'.join([','.join(line) for line in res]))
