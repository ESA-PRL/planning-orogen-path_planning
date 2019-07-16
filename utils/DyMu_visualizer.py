# -*- coding: utf-8 -*-

#========================DyMu Results Visualizer===============================
#           Visualization Tool to graphically represent maps and path
#            Author: J. Ricardo Sanchez Ibanez (ricardosan@uma.es)
#==============================================================================

import numpy as np
import matplotlib.pyplot as plt

costMap = np.loadtxt(open("GlobalCostMap.txt", "rb"), skiprows=0)
costMap[np.where(costMap==-1)] = np.nan

total_cost = np.loadtxt(open("TotalCostMap.txt", "rb"), skiprows=0)
total_cost[np.where(total_cost==-1)] = np.nan

path = np.loadtxt(open("Path.txt", "rb"), skiprows=0)

risk_map = np.loadtxt(open("RiskMap.txt", "rb"), skiprows=0)

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
plot3 = ax3.contourf(risk_map, 20, cmap = 'Reds')
ax3.set_aspect('equal')
ax3.set_xlabel('X-axis')
ax3.set_ylabel('Y-axis')
cb3 = fig2.colorbar(plot3, ax = ax3, orientation = 'horizontal')
cb3.ax.set_title('Risk')

plt.show()
