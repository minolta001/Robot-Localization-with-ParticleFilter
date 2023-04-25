import numpy as np
import matplotlib.pyplot as plt

data = np.genfromtxt('dist_matrix.csv', delimiter=',')

cmap = plt.cm.get_cmap('gray_r')

plt.imshow(data, cmap=cmap, vmin=-1, vmax=np.nanmax(data))

plt.show()