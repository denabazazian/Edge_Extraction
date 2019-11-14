from pyntcloud import PyntCloud 
import numpy as np
from mpl_toolkits.mplot3d import Axes3D 
import matplotlib.pyplot as plt 
import sys
import pdb

pcd1 = PyntCloud.from_file("/ArtificialPointClouds/bunny.pcd")
#pcd1 = PyntCloud.from_file("/TetrahedronMultiple.pcd")
#pcd1 = PyntCloud.from_file("/ArtificialPointClouds/CubeFractal2.pcd")

# define hyperparameters
k_n = 50
thresh = 0.03

# find neighbors
kdtree_id = pcd1.add_structure("kdtree")
k_neighbors = pcd1.get_neighbors(k=k_n, kdtree=kdtree_id) 

# calculate eigenvalues
ev = pcd1.add_scalar_field("eigen_values", k_neighbors=k_neighbors)

x = pcd1.points['x'].values 
y = pcd1.points['y'].values 
z = pcd1.points['z'].values 

e1 = pcd1.points['e3('+str(k_n+1)+')'].values
e2 = pcd1.points['e2('+str(k_n+1)+')'].values
e3 = pcd1.points['e1('+str(k_n+1)+')'].values

sum_eg = np.add(np.add(e1,e2),e3)
sigma = np.divide(e1,sum_eg)

#pdb.set_trace()
#img = ax.scatter(x, y, z, c=sigma, cmap='jet')

sigma = sigma>thresh

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Visualize each one of the eigenvalues
#img = ax.scatter(x, y, z, c=e1, cmap='jet')
#img = ax.scatter(x, y, z, c=e2, cmap='jet')
#img = ax.scatter(x, y, z, c=e3, cmap='jet')

# visualize the edges
img = ax.scatter(x, y, z, c=sigma, cmap='jet')
#img = ax.scatter(x, y, z, c=sigma, cmap=plt.hot())

fig.colorbar(img) 
plt.show() 
