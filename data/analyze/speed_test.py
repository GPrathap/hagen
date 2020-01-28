import numpy as np
from numpy import genfromtxt
import matplotlib.pyplot as plt
import seaborn as sns

import ssa

hagen_filter = genfromtxt('/home/geesara/catkin_ws/src/test/hagen_filter.csv', delimiter=',')[:,0][20:1500]*1000
pcl_filter = genfromtxt('/home/geesara/catkin_ws/src/test/pcl_filter.csv', delimiter=',')[:,0][20:1500]*1000
based_filter = genfromtxt('/home/geesara/catkin_ws/src/test/based_segmentation_filter.csv', delimiter=',')[:,0][20:1500]*1000


ssa_mode = ssa.SingularSpectrumAnalysis(hagen_filter, 60, verbose=False)
comp_1 = ssa_mode.execute(3)

ssa_mode = ssa.SingularSpectrumAnalysis(pcl_filter, 60, verbose=False)
comp_2 = ssa_mode.execute(3)

ssa_mode = ssa.SingularSpectrumAnalysis(based_filter, 60, verbose=False)
comp_3 = ssa_mode.execute(3)


plt.rcParams["font.size"] = "35"

fig, ax = plt.subplots()

plt.ylabel('runtime(ms)')
plt.xlabel('point cloud scan number')


ax.plot(pcl_filter, "orange",  label='PCL')
ax.plot(comp_2, "red", linewidth=2)
ax.plot(hagen_filter, "green",  label='Proposed')
ax.plot(comp_1, "red", linewidth=2)
ax.plot(based_filter, "black",  label='Zermas et al. 2017')
ax.plot(comp_3, "red", linewidth=1)


legend = ax.legend(loc='upper center', fontsize=23)
# # print(depth_filter[:,0])
# # print(hagen_filter.shape)
# # print(pcl_filter.shape)
plt.show()


