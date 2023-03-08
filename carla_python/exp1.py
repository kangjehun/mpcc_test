# import numpy as np
# from scipy.optimize import curve_fit
# from mpl_toolkits.mplot3d import Axes3D
# import matplotlib.pyplot as plt

# # define the function to fit
# def f(xy, Cr0, Cm1, Cm2, Cr2):
#     x, y = xy
#     return -Cr0 + Cm1*y - Cm2*x*y - Cr2*x**2

# # load data from test file
# data = np.loadtxt('/home/usrg/catkin_ws/src/mpcc_ros/carla_python/fx_data_file.txt')
# x, y, z = data[:,0], data[:,1], data[:,2]

# # perform the fit using curve_fit function
# popt, pcov = curve_fit(f, (x,y), z)

# # print the fitted parameters A, B, C, D
# print('Cm1 =', popt[1])
# print('Cm2 =', popt[2])
# print('Cr0 =', popt[0])
# print('Cr2 =', popt[3])

# # create a grid of x and y values for the fitted curve
# x_grid, y_grid = np.meshgrid(np.linspace(min(x), max(x), 100), np.linspace(min(y), max(y), 100))
# z_fit = f((x_grid, y_grid), *popt)

# # create a 3D scatter plot of the data
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.scatter(x, y, z, marker='o')

# # create a wireframe plot of the fitted curve
# ax.plot_wireframe(x_grid, y_grid, z_fit, color='r')

# # set labels for the axes
# ax.set_xlabel('x')
# ax.set_ylabel('y')
# ax.set_zlabel('z')

# # show the plot
# plt.show()

import numpy as np
from scipy.optimize import least_squares
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

# x : vx, y = d, z = Fx

# define the function to fit
def f(xy, Cr0, Cm1, Cm2, Cr2):
    x, y = xy
    return -Cr0 + Cm1*y - Cm2*x*y - Cr2*x**2

# load data from test file
data = np.loadtxt('/home/usrg/catkin_ws/src/mpcc_ros/carla_python/fx_data_file.txt')

# set the number of samples you want to take
num_samples = 1000

# sample the data
indices = np.random.choice(len(x), num_samples, replace=False)

x, y, z = data[indices:,0], data[indices:,1], data[indices:,2]

# define the residuals function to minimize
def residuals(p):
    return f((x,y), *p) - z

# set initial guess for the parameters
p0 = [1.0, 1.0, 1.0, 1.0]

# perform the fit using least_squares function
res = least_squares(residuals, p0)

# extract the fitted parameters Cr0, Cm1, Cm2, Cr2
popt = res.x
print('Cm1 =', popt[1])
print('Cm2 =', popt[2])
print('Cr0 =', popt[0])
print('Cr2 =', popt[3])

# create a grid of x and y values for the fitted curve
x_grid, y_grid = np.meshgrid(np.linspace(min(x), max(x), 100), np.linspace(min(y), max(y), 100))
z_fit = f((x_grid, y_grid), *popt)

# create a 3D scatter plot of the data
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(x, y, z, marker='o')

# create a wireframe plot of the fitted curve
ax.plot_wireframe(x_grid, y_grid, z_fit, color='r')

# set labels for the axes
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')

# show the plot
plt.show()
