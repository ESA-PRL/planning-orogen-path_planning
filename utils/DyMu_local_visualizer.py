# -*- coding: utf-8 -*-

#========================DyMu Results Visualizer===============================
#           Visualization Tool to graphically represent maps and path
#            Author: J. Ricardo Sanchez Ibanez (ricardosan@uma.es)
#==============================================================================

import sys
import numpy as np
import matplotlib.pyplot as plt

input_number = sys.argv[1]

risk_map = np.loadtxt(open("log/RiskMap_" + str(input_number) + ".txt", "rb"), skiprows=0)

deviation_map = np.loadtxt(open("log/DeviationMap_" + str(input_number) + ".txt", "rb"), skiprows=0)
deviation_map[np.where(deviation_map==-1)] = 0

fig2, (ax3, ax4) = plt.subplots(1, 2, tight_layout=True)
plot3 = ax3.contourf(risk_map, 20, cmap = 'Reds')
ax3.set_aspect('equal')
ax3.set_xlabel('X-axis')
ax3.set_ylabel('Y-axis')
cb3 = fig2.colorbar(plot3, ax = ax3, orientation = 'horizontal')
cb3.ax.set_title('Risk')
plot4 = ax4.contourf(deviation_map, 100, cmap = 'viridis', alpha = .5)
plot4 = ax4.contour(deviation_map, 100, cmap = 'viridis')
ax4.set_aspect('equal')
ax4.set_xlabel('X-axis')
ax4.set_ylabel('Y-axis')
cb4 = fig2.colorbar(plot4, ax = ax4, orientation = 'horizontal')
cb4.ax.set_title('Deviation')


plt.show()
