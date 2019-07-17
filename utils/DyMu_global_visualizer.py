# -*- coding: utf-8 -*-

#========================DyMu Results Visualizer===============================
#           Visualization Tool to graphically represent maps and path
#            Author: J. Ricardo Sanchez Ibanez (ricardosan@uma.es)
#==============================================================================

import sys
import numpy as np
import matplotlib.pyplot as plt

input_number = sys.argv[1]

costMap = np.loadtxt(open("log/GlobalCostMap_" + str(input_number) + ".txt"), skiprows=0)
costMap[np.where(costMap==-1)] = np.nan

total_cost = np.loadtxt(open("log/TotalCostMap_" + str(input_number) + ".txt"), skiprows=0)
total_cost[np.where(total_cost==-1)] = np.nan

hazard_density = np.loadtxt(open("log/HazardDensityMap_" + str(input_number) + ".txt"), skiprows=0)
hazard_density[np.where(hazard_density==-1)] = np.nan

trafficability = np.loadtxt(open("log/TrafficabilityMap_" + str(input_number) + ".txt"), skiprows=0)
trafficability[np.where(trafficability==-1)] = np.nan

path = np.loadtxt(open("log/Path_" + str(input_number) + ".txt"), skiprows=0)

fig1, (ax1, ax2) = plt.subplots(1, 2, tight_layout=True)
plot1 = ax1.contourf(costMap, 40, cmap = 'Reds')
ax1.set_aspect('equal')
ax1.plot(path[:,0],path[:,1],'b')
ax1.set_xlabel('X-axis')
ax1.set_ylabel('Y-axis')
cb1 = fig1.colorbar(plot1, ax = ax1, orientation = 'horizontal')
cb1.ax.set_title('Cost')

plot2 = ax2.contourf(total_cost, 100, cmap = 'viridis', alpha = .5)
ax2.contour(total_cost, 100, cmap = 'viridis')
ax2.plot(path[:,0],path[:,1],'r')
ax2.set_aspect('equal')
ax2.set_xlabel('X-axis')
ax2.set_ylabel('Y-axis')
cb2 = fig1.colorbar(plot2, ax = ax2, orientation = 'horizontal')
cb2.ax.set_title('Total Cost')

fig2, (ax3, ax4) = plt.subplots(1, 2, tight_layout=True)
plot3 = ax3.contourf(hazard_density, 20, cmap = 'Reds')
ax3.set_aspect('equal')
ax3.set_xlabel('X-axis')
ax3.set_ylabel('Y-axis')
cb3 = fig2.colorbar(plot3, ax = ax3, orientation = 'horizontal')
cb3.ax.set_title('Hazard Density')
plot4 = ax4.contourf(trafficability, 20, cmap = 'Reds')
ax4.set_aspect('equal')
ax4.set_xlabel('X-axis')
ax4.set_ylabel('Y-axis')
cb4 = fig2.colorbar(plot4, ax = ax4, orientation = 'horizontal')
cb4.ax.set_title('Trafficability')

plt.show()
