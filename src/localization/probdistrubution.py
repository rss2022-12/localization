# -*- coding: utf-8 -*-
"""
Created on Mon Mar 14 16:32:41 2022

@author: Sharmi Shah
"""
import numpy as np
import math as math
import matplotlib.pyplot as plt
from matplotlib import cm

alpha_hit = 0.74
alpha_short = 0.07
alpha_max = 0.07
alpha_rand = 0.12
sigma_hit = 8.0


zmax=200
sensor_model_table=np.ones((201,201))

phit=np.ones((201,201))


for z in range(201):
    for d in range(201):
         #phit
        if 0 <= z <= zmax:
            phit_i=1/math.sqrt(2*sigma_hit)*math.exp((-(z-d)**2)/(2*sigma_hit**2))
        else:
            phit_i=0
        phit[d,z]=phit_i
        
summed=np.sum(phit,axis=1,keepdims=True)
phit_norm= phit/summed
   
for z in range(201):
    for d in range(201):
      
        
        #pshort
        if 0 <= z <= d and d!=0:
            pshort=2/d*(1-z/d)
        else:
            pshort=0
            
        #pmax
        if  z == zmax:
            pmax=1
        else:
            pmax=0
            
        #prand
        if 0 <= z <= zmax:
            prand=1/zmax
        else:
            prand=0
        sensor_model_table[d,z]=alpha_hit*phit_norm[d,z]+alpha_short*pshort+alpha_max*pmax+alpha_rand*prand
summed=np.sum(sensor_model_table,axis=1,keepdims=True)
sensor_model_table= sensor_model_table/summed
      
# print(sensor_model_table)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
x = y = np.arange(0,201, dtype=int)
X, Y = np.meshgrid(x, y)

ax.plot_surface(X, Y, sensor_model_table)

ax.set_xlabel('Z Label')
ax.set_ylabel('D Label')
ax.set_zlabel('prob Label')