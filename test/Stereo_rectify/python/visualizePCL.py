# -*- coding: utf-8 -*-
"""
Created on Sun Jul 26 20:55:50 2020

@author: vekar
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

file_data_path = './/..//PCL_test.pcl'
point_cloud= np.loadtxt(file_data_path, skiprows=1, max_rows=250000)
mean_Z=np.mean(point_cloud,axis=0)[2]
spatial_query=point_cloud[abs( point_cloud[:,2]-mean_Z)<1]
xyz=spatial_query[:,:3]
rgb=spatial_query[:,3:]
ax = plt.axes(projection='3d')
ax.scatter(point_cloud[:,0], point_cloud[:,1], point_cloud[:,2], s=0.05)
plt.show()
