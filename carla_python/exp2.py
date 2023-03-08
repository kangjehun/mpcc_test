import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt


# Define the Pacejka tire model function
def pacejka(x, B, C, D):
    return D * np.sin(C * np.arctan(B * x))

# Load the data from the text file
data = np.loadtxt('/home/usrg/catkin_ws/src/mpcc_ros/carla_python/pacejka_data_file.txt')
Ffy, Fry, alpha_f, alpha_r = data[:,0], data[:,1], data[:,2], data[:,3]

# Define the initial guess for the parameters B, C, D
initial_guess = [1, 1, 1]
# Fit the data with the Pacejka tire model using the curve_fit function
popt_f, pcov_f = curve_fit(pacejka, alpha_f, Ffy, p0=initial_guess)
popt_r, pcov_r = curve_fit(pacejka, alpha_r, Fry, p0=initial_guess)
Bf, Cf, Df = popt_f
# print the fitted parameters A, B, C, D
print('Bf =', popt_f[0])
print('Cf =', popt_f[1])
print('Df =', popt_f[2])
Br, Cr, Dr = popt_r
# print the fitted parameters A, B, C, D
print('Br =', popt_r[0])
print('Cr =', popt_r[1])
print('Dr =', popt_r[2])

# Generate the predicted lateral forces using the fitted parameters
Ffy_pred = pacejka(alpha_f, Bf, Cf, Df)
Fry_pred = pacejka(alpha_r, Br, Cr, Dr)

# Plot the results
plt.figure()
plt.plot(alpha_f, Ffy, 'bo', label='Real Data (Front Tire)')
# plt.plot(alpha_f, Ffy_pred, 'r-', label='Fitted Curve (Front Tire)')
plt.plot(alpha_r, Fry, 'go', label='Real Data (Rear Tire)')
# plt.plot(alpha_r, Fry_pred, 'm-', label='Fitted Curve (Rear Tire)')
plt.xlabel('Slip Angle (degrees)')
plt.ylabel('Lateral Force (N)')
plt.legend()
plt.show()