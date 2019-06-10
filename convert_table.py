
for i in range(1, 37):
	x = input().split()
	assert(len(x) == 11)
	print(f'{min(float(x[1]),float(x[8]))}')
	# assert(len(x) == 10)
	# print(f'{x[1]},{x[4]}')
	# assert(len(x) == 13)
	# print(f'{min(float(x[2]), float(x[2]))},{min(float(x[5]), float(x[5]))},{min(float(x[8]), float(x[8]))},{min(float(x[11]), float(x[11]))}')
	# print(f'{min(float(x[1]), float(x[2]))},{min(float(x[4]), float(x[5]))},{min(float(x[7]), float(x[8]))},{min(float(x[10]), float(x[11]))}')
