import matplotlib.pyplot as plt
import os

estimations = []
measurements = []
ground_truth = []

with open('../data/out-1.txt', 'r') as f:

    for line in f:
        data = line.split()
        estimations.append([float(data[0]), float(data[1]), float(data[2]), float(data[3])])
        measurements.append([float(data[4]), float(data[5])])
        ground_truth.append([float(data[6]), float(data[7]), float(data[8]), float(data[9])])

x_estimation = []
y_estimation = []
x_truth = []
y_truth = []
x_meas = []
y_meas = []

for i in range(len(estimations)):
    x_estimation.append(estimations[i][0])
    y_estimation.append(estimations[i][1])
    x_truth.append(ground_truth[i][0])
    y_truth.append(ground_truth[i][1])
    x_meas.append(measurements[i][0])
    y_meas.append(measurements[i][1])


plt.figure()
plt.plot(x_truth, y_truth, c='k', label='Ground Truth')
plt.scatter(x_estimation, y_estimation, c='b', marker= 'o', label='Estimation', s=20)
plt.scatter(x_meas, y_meas, c='g', marker= '^', label='Measurements', s=15)
plt.legend(loc='best')

plt.show()





