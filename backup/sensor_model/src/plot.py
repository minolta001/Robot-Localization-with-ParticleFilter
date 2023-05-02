import numpy as np
import matplotlib.pyplot as plt

data = np.genfromtxt('dist_matrix.csv', delimiter=',')

cmap = plt.cm.get_cmap('gray')

plt.imshow(data, cmap=cmap, vmin=np.nanmin(data), vmax=np.nanmax(data))

plt.show()