'''
res

278.735,284.529,284.529,294.259,278.735
334.974,334.974,352.167,334.974,334.974
358.413,387.714,394.732,364.466,358.413
430.896,430.896,445.495,447.384,430.896
375.292,375.292,381.701,383.887,388.98
495.862,495.862,499.09,498.33,495.862
568.574,725.471,701.09,700.727,658.653
,716.719,741.128,700.385,621.856
607.665,607.665,607.665,625.11,607.665
535.812,692.964,639.836,727.108,687.832
505.026,711.098,714.431,802.3,656.046
610.02,610.587,610.02,614.251,610.251
2006.36,2603.95,2468.08,2610.45,2557.22
837.689,1041.64,1145.67,1122.63,1203.87
904.566,1020.5,1170.03,1242.62,1182.72
698.626,698.626,698.626,703.636,698.626
861.811,870.881,861.811,861.811,861.811
723.56,1015.75,1084.06,1125.43,925.653
524.639,762.743,790.793,796.056,656.335
265.039,533.933,541.392,549.013,508.406
'''

'''
TSP

1315.37,1144.9,1154.64,1151.48,1209.03
1307.3,1256.45,1227.75,1271.88,1283.07
1335.36,1099.76,1087.59,1168.52,1241.67
1337.81,1218.02,1274.87,1230.55,1291.13
1340.87,1182.95,1177.69,1182.98,1302.45
1337.93,1288.99,1235.42,1152.26,1305.87
1960.3,1033.75,1047.14,1053.58,1063.87
,1129.2,1145.76,1050.06,1017.12
1343.88,1240.82,1230.43,1223.51,1255.4
1814.29,845.034,1022.39,834.527,1017.53
1545.23,871.084,835.39,815.053,906.13
1349.92,1147.27,1299.47,1170.4,1185.47
3411.45,888.752,901.979,723.882,945.584
3233.82,647.715,774.915,1132.32,887.343
2974.65,530.244,872.118,487.145,386.749
1351.28,1267.46,1257.09,1180.86,1283.55
1349.56,1288.3,1319.9,1259.89,1298.34
3569.14,717.05,780.286,567.182,210.282
2968.71,612.712,651.978,511.996,438.061
3620.36,337.727,417.984,307.614,305.35
'''

'''
Packing

576.009,740.797,739.186,724.96,704.351
528.228,592.228,625.441,590.316,582.361
610.132,821.415,847.959,813.528,765.047
546.766,677.088,640.73,677.363,637.483
597.401,773.652,779.639,770.833,648.139
547.312,613.159,672.346,757.409,616.719
474.638,883.067,990.864,1043.76,1089
,850.446,822.914,964.735,1270.33
550.085,682.417,711.965,732.292,726.274
572.111,1216.76,1095.43,1228.87,1132.99
624.948,1122.48,1172.23,1190.02,1374.16
575.057,834.103,668.82,846.527,902.952
85.1044,1255.19,1257.28,1645.37,1407.59
130.554,1914.3,1680.71,938.974,1616.6
203.696,2157.95,1251.61,2195.6,2544.42
577.101,697.008,737.777,824.191,719.201
581.31,680.795,638.788,747.445,698.092
24.7586,1684,1485.28,2062.92,3141.94
331.931,1841.64,1736.64,2138.6,2574.18
17.9316,2465.64,2439.86,2648.79,2873.15
'''

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
		tsp_time[-1].append(lines[-2][len('Total tsp time '):-1])
		packing_time[-1].append(lines[-1][len('Total packing time '):-1])
		f = open(f'results/2l_cvrp{i:02}{j:02}.sol', 'r')
		res[-1].append(f.read()[:-1])
	print(i)

print('\n'.join([','.join(line) for line in tsp_time]))
print('\n'.join([','.join(line) for line in packing_time]))
print('\n'.join([','.join(line) for line in res]))


