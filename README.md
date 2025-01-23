# Allan Variance ROS
## ROS package which loads a rosbag of IMU data and computes Allan Variance parameters
The purpose of this tool is to read a long sequence of IMU data and compute the Angle Random Walk (ARW), Bias Instability and Gyro Random Walk for the gyroscope as well as Velocity Random Walk (VRW), Bias Instability and Accel Random Walk for the accelerometer.

While there are many open source tools which do the same thing, this package has the following features:

- Fully ROS compatable. Simply record a `rosbag` and provide it as input. No conversion required.
- Written in C++ making use of rosbag::View means the `rosbag` is processed at maximum speed. No need to play back the bag file.
- Designed for [Kalibr](https://github.com/ethz-asl/kalibr). Will produce an `imu.yaml` file.

This tool is designed for Ubuntu 20.04. Attempting to use on another distro or version may require some code changes.

If you do not have Ubuntu 20.04, you can use the devcontainer to build and run the tool. See the [devcontainer README](.devcontainer/README.md) for more information.

## How to build

The allan_variance_ros folder should be in the src folder of the catkin workspace.
For example: ~/catkin_ws/src/allan_variance_ros

From the catkin workspace directory (e.g. ~/catkin_ws), run:
``catkin build allan_variance_ros``
``source devel/setup.bash``

## How to use

1. Ensure rosmaster is running by running ``roscore`` in a terminal.

2. Place your IMU on some damped surface and record your IMU data to a rosbag. You must record **at least** 3 hours of data. The longer the sequence, the more accurate the results.

3. **Recommended** Reorganize ROS messages by timestamp:

  ``rosrun allan_variance_ros cookbag.py --input original_rosbag --output cooked_rosbag``

4. Run the Allan Variance computation tool (example config files provided):

  ``rosrun allan_variance_ros allan_variance [path_to_folder_containing_bag] [path_to_config_file]``

5. This will compute the Allan Deviation for the IMU and generate a CSV. The next step is to visualize the plots and get parameters. For this run:

  ``rosrun allan_variance_ros analysis.py --data allan_variance.csv``

  If you have a config file to set the topic and update rate, you can pass it in as well:

  ``rosrun allan_variance_ros analysis.py --data allan_variance.csv --config config/realsense_d425i.yaml``

  Press `space` to go to next figure.



### Example Log

3 hour log of [Realsense D435i IMU](https://drive.google.com/file/d/1ovI2NvYR52Axt-KuRs5HjVk7-57ky72H/view?usp=sharing) with timestamps already re-arranged.

![Acceleration](/figs/realsense_acceleration.png)
![Gyroscope](/figs/realsense_gyro.png)

Example terminal output:

```
ACCELEROMETER:
X Velocity Random Walk:  0.00333 m/s/sqrt(s)  0.19983 m/s/sqrt(hr)
Y Velocity Random Walk:  0.01079 m/s/sqrt(s)  0.64719 m/s/sqrt(hr)
Z Velocity Random Walk:  0.00481 m/s/sqrt(s)  0.28846 m/s/sqrt(hr)
X Bias Instability:  0.00055 m/s^2  7173.28800 m/hr^2
Y Bias Instability:  0.00153 m/s^2  19869.01200 m/hr^2
Z Bias Instability:  0.00052 m/s^2  6701.58000 m/hr^2
X Accel Random Walk:  0.00008 m/s^2/sqrt(s)
Y Accel Random Walk:  0.00020 m/s^2/sqrt(s)
Z Accel Random Walk:  0.00007 m/s^2/sqrt(s)
GYROSCOPE:
X Angle Random Walk:  0.00787 deg/sqrt(s)  0.47215 deg/sqrt(hr)
Y Angle Random Walk:  0.00987 deg/sqrt(s)  0.59204 deg/sqrt(hr)
Z Angle Random Walk:  0.00839 deg/sqrt(s)  0.50331 deg/sqrt(hr)
X Bias Instability:  0.00049 deg/s  1.76568 deg/hr
Y Bias Instability:  0.00136 deg/s  4.88153 deg/hr
Z Bias Instability:  0.00088 deg/s  3.15431 deg/hr
X Rate Random Walk:  0.00007 deg/s/sqrt(s)
Y Rate Random Walk:  0.00028 deg/s/sqrt(s)
Z Rate Random Walk:  0.00011 deg/s/sqrt(s)

```

## Kalibr

[Kalibr](https://github.com/ethz-asl/kalibr) is a useful collection of tools for calibrating cameras and IMUs. For IMU calibration it needs the noise parameters of the IMU generated in a yaml file. `allan_variance_ros` automatically generates this file file as `imu.yaml`:

```
#Accelerometer
accelerometer_noise_density: 0.006308226052016165 
accelerometer_random_walk: 0.00011673723527962174 

#Gyroscope
gyroscope_noise_density: 0.00015198973532354657 
gyroscope_random_walk: 2.664506559330434e-06 

rostopic: '/sensors/imu' #Make sure this is correct
update_rate: 400.0 #Make sure this is correct

```
## Allan Variance ROS Evaluation

### IMU Noise Simulator

Thanks to [@kekeliu-whu](https://github.com/kekeliu-whu) who contributed an IMU noise simulator is based on the [Kalibr IMU noise model](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model). You can generate a rosbag of simulated IMU noise and run allan_variance_ros to verify the tool is working.
As shown in PR https://github.com/ori-drs/allan_variance_ros/pull/24 accuracy is quite good.


### To generate simulated noise

`rosrun allan_variance_ros imu_simulator [path_to_output_bag_file] [path_to_simulation_config_file]`

A simulation config file is provided in `allan_variance_ros/config/simulation/imu_simulator.yaml`

### To test Allan Variance ROS on simulated rosbag

  ``rosrun allan_variance_ros allan_variance [path_to_folder_containing_bag] [path_to_config_file]``

A config file is provided in `allan_variance_ros/config/sim.yaml`

### Additional Example bags

Some additional rosbags of real IMU data for different sensors is available [here](https://drive.google.com/drive/folders/1a3Es85JDKl7tSpVWEUZryOwtsXB8793o). Thanks to Patrick Geneva.

## Author

[Russell Buchanan](https://www.ripl-lab.com/)

If you use this package for academic work, please consider using the citation below:

```
@software{AllanVarianceRos,
  author       = {Russell Buchanan},
  title        = {Allan Variance ROS},
  month        = Nov,
  year         = 2021,
  publisher    = {Oxford Robotics Institute, DRS Lab},
  version      = {1.2},
  url          = {https://github.com/ori-drs/allan_variance_ros}}
}
```

## References

- [Indirect Kalman Filter for 3D Attitude Estimation, Trawny & Roumeliotis](http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf)
- [An introduction to inertial navigation, Oliver Woodman](https://www.cl.cam.ac.uk/techreports/UCAM-CL-TR-696.pdf) 
- [Characterization of Errors and Noises in MEMS Inertial Sensors Using Allan Variance Method, Leslie Barreda Pupo](https://upcommons.upc.edu/bitstream/handle/2117/103849/MScLeslieB.pdf?sequence=1&isAllowed=y)
- [Kalibr IMU Noise Documentation](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model)
