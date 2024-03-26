"""Model beam cross-section with stringers using B-splines"""

from scipy.interpolate import splprep, splev
import numpy as np
import matplotlib.pyplot as plt


x_outer = np.array([0, 5, 5, 0])
y_outer = np.array([0, 0, 5, 5])

x_inner = np.array([1, 1.5, 1.5, 2.0, 2.0, 2.5, 2.5, 3.0, 3.0, 3.5, 3.5, 4.0, 4.0, 4.0, 1])
y_inner = np.array([1, 1.0, 0.5, 0.5, 1.0, 1.0, 0.5, 0.5, 1.0, 1.0, 0.5, 0.5, 1.0, 4.0, 4.0])

tck, u = splprep([x_outer, y_outer], s=0)
new_points = splev(u, tck)

print(tck)
print(u)
print(new_points)

plt.scatter(x_outer, y_outer)
plt.plot(new_points[0], new_points[1])
plt.scatter(x_inner, y_inner)
plt.show()