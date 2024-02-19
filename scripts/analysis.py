#!/usr/bin/env python3

# 
# @file   analysis.py
# @brief  Plotting and analysis tool to determine IMU parameters.
# @author Russell Buchanan
# 

import argparse
import csv

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit

def line_func(x, m, b):
	return m * x + b

def get_intercept(x, y, m, b):

	logx = np.log(x)
	logy = np.log(y)
	coeffs, _ = curve_fit(line_func, logx, logy, bounds=([m, -np.inf], [m + 0.001, np.inf]))
	poly = np.poly1d(coeffs)
	yfit = lambda x: np.exp(poly(np.log(x)))

	return yfit(b), yfit


def generate_prediction(tau, q_quantization=0, q_white=0, q_bias_instability=0, q_walk=0, q_ramp=0):
	n = len(tau)

	A = np.empty((n, 5))
	A[:, 0] = 3 / tau**2
	A[:, 1] = 1 / tau
	A[:, 2] = 2 * np.log(2) / np.pi
	A[:, 3] = tau / 3
	A[:, 4] = tau**2 / 2

	params = np.array([q_quantization ** 2, q_white ** 2, q_bias_instability ** 2, q_walk ** 2, q_ramp ** 2])

	return np.sqrt(A.dot(params))


parser = argparse.ArgumentParser()

parser.add_argument('--data', metavar='STR', type=str,
					help='TUM data files to plot')

parser.add_argument('--config', metavar='STR', type=str,
					help='yaml config file')

parser.add_argument("--skip", type=int, default=1)

args = parser.parse_args()

line_count = 0

rostopic = "/sensors/imu"
update_rate = 400.0
if args.config:
	import yaml #pip install pyyaml
	with open(args.config, "r") as stream:
		config = yaml.safe_load(stream)
	rostopic = config["imu_topic"]
	update_rate = config["imu_rate"]

# Assumes tum format

period = np.array([])
acceleration = np.empty((0,3), float)
rotation_rate = np.empty((0,3), float)

with open(args.data) as input_file:
	csv_reader = csv.reader(input_file, delimiter=' ')

	first_row = True
	counter = 0

	for row in csv_reader:
		# if(first_row):
		# 	first_row = False
		# 	continue

		counter = counter + 1

		if (counter % args.skip != 0):
			continue

		t = float(row[0])
		period = np.append(period, [t], axis=0)
		acceleration = np.append(acceleration, np.array([float(row[1]), float(row[2]), float(row[3])]).reshape(1,3), axis=0)
		rotation_rate = np.append(rotation_rate, np.array([float(row[4]), float(row[5]), float(row[6])]).reshape(1,3), axis=0)


white_noise_break_point = np.where(period == 10)[0][0]
random_rate_break_point = np.where(period == 10)[0][0]


accel_wn_intercept_x, xfit_wn = get_intercept(period[0:white_noise_break_point], acceleration[0:white_noise_break_point,0], -0.5, 1.0)
accel_wn_intercept_y, yfit_wn = get_intercept(period[0:white_noise_break_point], acceleration[0:white_noise_break_point,1], -0.5, 1.0)
accel_wn_intercept_z, zfit_wn = get_intercept(period[0:white_noise_break_point], acceleration[0:white_noise_break_point,2], -0.5, 1.0)

accel_rr_intercept_x, xfit_rr = get_intercept(period, acceleration[:,0], 0.5, 3.0)
accel_rr_intercept_y, yfit_rr = get_intercept(period, acceleration[:,1], 0.5, 3.0)
accel_rr_intercept_z, zfit_rr = get_intercept(period, acceleration[:,2], 0.5, 3.0)


accel_min_x = np.amin(acceleration[:,0])
accel_min_y = np.amin(acceleration[:,1])
accel_min_z = np.amin(acceleration[:,2])

accel_min_x_index = np.argmin(acceleration[:,0])
accel_min_y_index = np.argmin(acceleration[:,1])
accel_min_z_index = np.argmin(acceleration[:,2])

log_file = open("output.log", "w+")
kalibr_file = open("imu.yaml", "w")

log_file.write("#ACCELEROMETER:\n")
log_file.write(f"X Velocity Random Walk: {accel_wn_intercept_x: .5f} m/s/sqrt(s) {accel_wn_intercept_x*60: .5f} m/s/sqrt(hr)\n")
log_file.write(f"Y Velocity Random Walk: {accel_wn_intercept_y: .5f} m/s/sqrt(s) {accel_wn_intercept_y*60: .5f} m/s/sqrt(hr)\n")
log_file.write(f"Z Velocity Random Walk: {accel_wn_intercept_z: .5f} m/s/sqrt(s) {accel_wn_intercept_z*60: .5f} m/s/sqrt(hr)\n")
log_file.write("\n")

log_file.write(f"X Bias Correlation Time: {accel_min_x_index: .5f} 1/s {accel_min_x_index*3600: .5f} 1/hr\n")
log_file.write(f"Y Bias Correlation Time: {accel_min_y_index: .5f} 1/s {accel_min_y_index*3600: .5f} 1/hr\n")
log_file.write(f"Z Bias Correlation Time: {accel_min_z_index: .5f} 1/s {accel_min_z_index*3600: .5f} 1/hr\n")
log_file.write("\n")

log_file.write(f"X Bias Instability: {accel_min_x: .5f} m/s^2 {accel_min_x*3600*3600: .5f} m/hr^2\n")
log_file.write(f"Y Bias Instability: {accel_min_y: .5f} m/s^2 {accel_min_y*3600*3600: .5f} m/hr^2\n")
log_file.write(f"Z Bias Instability: {accel_min_z: .5f} m/s^2 {accel_min_z*3600*3600: .5f} m/hr^2\n")
log_file.write("\n")

log_file.write(f"X Accel Random Walk: {accel_rr_intercept_x: .5f} m/s^2/sqrt(s)\n")
log_file.write(f"Y Accel Random Walk: {accel_rr_intercept_y: .5f} m/s^2/sqrt(s)\n")
log_file.write(f"Z Accel Random Walk: {accel_rr_intercept_z: .5f} m/s^2/sqrt(s)\n")
log_file.write("\n")

average_acc_white_noise = (accel_wn_intercept_x + accel_wn_intercept_y + accel_wn_intercept_z) / 3
average_acc_bias_instability = (accel_min_x + accel_min_y + accel_min_z) / 3
average_acc_random_walk = (accel_rr_intercept_x + accel_rr_intercept_y + accel_rr_intercept_z) / 3

# use worst value
worst_accel_white_noise = np.amax([accel_wn_intercept_x, accel_wn_intercept_y, accel_wn_intercept_z])
worst_accel_random_walk = np.amax([accel_rr_intercept_x, accel_rr_intercept_y, accel_rr_intercept_z])

kalibr_file.write("#Accelerometer\n")
kalibr_file.write("accelerometer_noise_density: " + repr(worst_accel_white_noise) + " \n")
kalibr_file.write("accelerometer_random_walk: " + repr(worst_accel_random_walk) + " \n")
kalibr_file.write("\n")


dpi = 90
figsize = (16, 9)
fig1 = plt.figure(num="Acceleration", dpi=dpi, figsize=figsize)

plt.loglog(period, acceleration[:,0], "r--" , label='X')
plt.loglog(period, acceleration[:,1], "g--" , label='Y')
plt.loglog(period, acceleration[:,2], "b--" , label='Z')

plt.loglog(period, xfit_wn(period), "m-")
plt.loglog(period, yfit_wn(period), "m-")
plt.loglog(period, zfit_wn(period), "m-", label="White noise fit line")

plt.loglog(period, xfit_rr(period), "y-",)
plt.loglog(period, yfit_rr(period), "y-",)
plt.loglog(period, zfit_rr(period), "y-", label="Random Rate fit line")

plt.loglog(1.0, accel_wn_intercept_x, "ro", markersize=20)
plt.loglog(1.0, accel_wn_intercept_y, "go", markersize=20)
plt.loglog(1.0, accel_wn_intercept_z, "bo", markersize=20)

plt.loglog(3.0, accel_rr_intercept_x, "r*", markersize=20)
plt.loglog(3.0, accel_rr_intercept_y, "g*", markersize=20)
plt.loglog(3.0, accel_rr_intercept_z, "b*", markersize=20)

plt.loglog(period[accel_min_x_index], accel_min_x, "r^", markersize=20)
plt.loglog(period[accel_min_y_index], accel_min_y, "g^", markersize=20)
plt.loglog(period[accel_min_z_index], accel_min_z, "b^", markersize=20)

fitted_model = generate_prediction(period, q_white=average_acc_white_noise,
									q_bias_instability=average_acc_bias_instability, q_walk=average_acc_random_walk)
plt.loglog(period, fitted_model, "-k", label='fitted model')

plt.title("Accelerometer", fontsize=30)
plt.ylabel("Allan Deviation m/s^2", fontsize=30)
plt.legend(fontsize=25)
plt.grid(True)
plt.xlabel("Period (s)", fontsize=30)
plt.tight_layout()

plt.draw()
plt.pause(1)
w = plt.waitforbuttonpress(timeout=5)
plt.close()

fig1.savefig('acceleration.png', dpi=600, bbox_inches = "tight")

gyro_wn_intercept_x, xfit_wn = get_intercept(period[0:white_noise_break_point], rotation_rate[0:white_noise_break_point,0], -0.5, 1.0)
gyro_wn_intercept_y, yfit_wn = get_intercept(period[0:white_noise_break_point], rotation_rate[0:white_noise_break_point,1], -0.5, 1.0)
gyro_wn_intercept_z, zfit_wn = get_intercept(period[0:white_noise_break_point], rotation_rate[0:white_noise_break_point,2], -0.5, 1.0)

gyro_rr_intercept_x, xfit_rr = get_intercept(period, rotation_rate[:,0], 0.5, 3.0)
gyro_rr_intercept_y, yfit_rr = get_intercept(period, rotation_rate[:,1], 0.5, 3.0)
gyro_rr_intercept_z, zfit_rr = get_intercept(period, rotation_rate[:,2], 0.5, 3.0)

gyro_min_x = np.amin(rotation_rate[:,0])
gyro_min_y = np.amin(rotation_rate[:,1])
gyro_min_z = np.amin(rotation_rate[:,2])

gyro_min_x_index = np.argmin(rotation_rate[:,0])
gyro_min_y_index = np.argmin(rotation_rate[:,1])
gyro_min_z_index = np.argmin(rotation_rate[:,2])

log_file.write("#GYROSCOPE:\n")
log_file.write(f"X Angle Random Walk: {gyro_wn_intercept_x: .5f} deg/sqrt(s) {gyro_wn_intercept_x * 60: .5f} deg/sqrt(hr)\n")
log_file.write(f"Y Angle Random Walk: {gyro_wn_intercept_y: .5f} deg/sqrt(s) {gyro_wn_intercept_y * 60: .5f} deg/sqrt(hr)\n")
log_file.write(f"Z Angle Random Walk: {gyro_wn_intercept_z: .5f} deg/sqrt(s) {gyro_wn_intercept_z * 60: .5f} deg/sqrt(hr)\n")
log_file.write("\n")

log_file.write(f"X Bias Correlation Time: {gyro_min_x_index: .5f} 1/s {gyro_min_x_index*3600: .5f} 1/hr\n")
log_file.write(f"Y Bias Correlation Time: {gyro_min_y_index: .5f} 1/s {gyro_min_y_index*3600: .5f} 1/hr\n")
log_file.write(f"Z Bias Correlation Time: {gyro_min_z_index: .5f} 1/s {gyro_min_z_index*3600: .5f} 1/hr\n")
log_file.write("\n")

log_file.write(f"X Bias Instability: {gyro_min_x: .5f} deg/s {gyro_min_x*60*60: .5f} deg/hr\n")
log_file.write(f"Y Bias Instability: {gyro_min_y: .5f} deg/s {gyro_min_y*60*60: .5f} deg/hr\n")
log_file.write(f"Z Bias Instability: {gyro_min_z: .5f} deg/s {gyro_min_z*60*60: .5f} deg/hr\n")
log_file.write("\n")

log_file.write(f"X Rate Random Walk: {gyro_rr_intercept_x: .5f} deg/s/sqrt(s)\n")
log_file.write(f"Y Rate Random Walk: {gyro_rr_intercept_y: .5f} deg/s/sqrt(s)\n")
log_file.write(f"Z Rate Random Walk: {gyro_rr_intercept_z: .5f} deg/s/sqrt(s)\n")

# Go back to the beginning of the file and output to terminal
log_file.seek(0)
print(log_file.read())
log_file.close()

average_gyro_white_noise = (gyro_wn_intercept_x + gyro_wn_intercept_y + gyro_wn_intercept_z) / 3
average_gyro_bias_instability = (gyro_min_x + gyro_min_y + gyro_min_z) / 3
average_gyro_random_walk = (gyro_rr_intercept_x + gyro_rr_intercept_y + gyro_rr_intercept_z) / 3

# use worst value
worst_gyro_white_noise = np.amax([gyro_wn_intercept_x, gyro_wn_intercept_y, gyro_wn_intercept_z])
worst_gyro_random_walk = np.amax([gyro_rr_intercept_x, gyro_rr_intercept_y, gyro_rr_intercept_z])

kalibr_file.write("#Gyroscope\n")
# Convert back to radians here
kalibr_file.write("gyroscope_noise_density: " + repr(worst_gyro_white_noise * np.pi / 180) + " \n")
kalibr_file.write("gyroscope_random_walk: " + repr(worst_gyro_random_walk * np.pi / 180) + " \n")
kalibr_file.write("\n")


if args.config:
	kalibr_file.write("rostopic: " + repr(rostopic) + " \n")
	kalibr_file.write("update_rate: " + repr(update_rate) + " \n")
else:
	kalibr_file.write("rostopic: " + repr(rostopic) + " #Make sure this is correct\n")
	kalibr_file.write("update_rate: " + repr(update_rate) + " #Make sure this is correct\n")
kalibr_file.write("\n")
kalibr_file.close()


fig2 = plt.figure(num="Gyro", dpi=dpi, figsize=figsize)

plt.loglog(period, rotation_rate[:,0], "r-" , label='X')
plt.loglog(period, rotation_rate[:,1], "g-" , label='Y')
plt.loglog(period, rotation_rate[:,2], "b-" , label='Z')

plt.loglog(period, xfit_wn(period), "m-")
plt.loglog(period, yfit_wn(period), "m-")
plt.loglog(period, zfit_wn(period), "m-", label="White noise fit line")

plt.loglog(period, xfit_rr(period), "y-")
plt.loglog(period, yfit_rr(period), "y-")
plt.loglog(period, zfit_rr(period), "y-", label="Random rate fit line")

plt.loglog(1.0, gyro_wn_intercept_x, "ro", markersize=20)
plt.loglog(1.0, gyro_wn_intercept_y, "go", markersize=20)
plt.loglog(1.0, gyro_wn_intercept_z, "bo", markersize=20)

plt.loglog(3.0, gyro_rr_intercept_x, "r*", markersize=20)
plt.loglog(3.0, gyro_rr_intercept_y, "g*", markersize=20)
plt.loglog(3.0, gyro_rr_intercept_z, "b*", markersize=20)

plt.loglog(period[gyro_min_x_index], gyro_min_x, "r^", markersize=20)
plt.loglog(period[gyro_min_y_index], gyro_min_y, "g^", markersize=20)
plt.loglog(period[gyro_min_z_index], gyro_min_z, "b^", markersize=20)

fitted_model = generate_prediction(period, q_white=average_gyro_white_noise,
									q_bias_instability=average_gyro_bias_instability, q_walk=average_gyro_random_walk)
plt.loglog(period, fitted_model, "-k", label='fitted model')

plt.title("Gyroscope", fontsize=30)
plt.ylabel("Allan Deviation deg/s", fontsize=30)
plt.legend(fontsize=25)
plt.grid(True)
plt.xlabel("Period (s)", fontsize=30)
plt.tight_layout()

plt.draw()
plt.pause(1)
w = plt.waitforbuttonpress(timeout=5)
plt.close()

fig2.savefig('gyro.png', dpi=600, bbox_inches = "tight")

print("Writing Kalibr imu.yaml file.")
print("Make sure to update rostopic and rate.")
