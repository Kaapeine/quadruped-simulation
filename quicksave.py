# coding: utf-8
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot([0,1], [0,1], [0,1])
plt.show()
get_ipython().run_line_magic('save', 'quicksave 1-11')
