# theodwyn-ros
Collection of ROS2 Middleware Packages for the *Theodwyn* Robots, Eomer and Eowyn unmanned ground vehicles (UGVs) 

---
## Content

- [theodwyn-ros](#theodwyn-ros)
  - [Content](#Content)
  - [Dependencies](#Dependencies)
  - [Build](#Build)
  - [Configuration](#Configuration)
  - [Execution](#Execution)


---

## Dependencies
###
This repository contains a collection of ROS packages, evidently dependent on the [ROS2 Middleware libraries](https://www.ros.org/). The associated installation instructions can be found on their website. 

In addition, the ROS development tools will be necessary to build this package from the provided source files. The tools may be installed via `apt`, after completing the aforementioned installation
```bash
sudo apt install ros-dev-tools
```
### Eigen3
To manage various aspects of linear algerbra within this repository, the [Eigen library](https://eigen.tuxfamily.org) is utilized. The associated installation instructions can be found on their website.
### Boost C++
To manage various aspects of hardware communications and I/O, the [Boost libraries](https://www.boost.org/doc/user-guide/intro.html) are utilized. The libraries may be nominally installed via `apt` in the following
```bash
sudo apt-get install libboost-all-dev
```
### ros2-vicon-receiver
To communicate with the local vicon system, the ros2-vicon-receiver package is used to publish localization data to the ROS network
```bash
git submodule update --init --recursive
```
> [!NOTE]
> This is a fork of the original repository, making necessary changes for consistent operations. Located at [PeteLealiieeJ/ros2-vicon-receiver](https://github.com/PeteLealiieeJ/ros2-vicon-receiver)

## Build
The Pacakge and CMake files provided allow this repository to be built simply with `colcon` in the following.
### Install ROS packages' dependencies
```bash
rosdep install --from-paths src --rosdistro $ROS_DISTRO -y --ignore-src
```
### Build ROS packages
In the top-level directory of this repository
```bash
colcon build --packages-select theo_msgs theo_srvs theo_comm theo_core theo_teleop theo_autoop
```
```bash
colcon build --packages-select vicon_receiver --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```
These commands will spawn `build/`, `install/`, `log/` folders in the top-level directory of this repository. To use the built packages, **ensure the libraries have been sourced** in the shell session, whether that process is automated via the `~/.bashrc` or manually performed in every new shell session with the following command.
```bash
source install/setup.bash
```

---

## Configuration
Each of the provided packages has respective configuration files, which may be utilized to change the behavior of the respective nodes. The parameters of the nodes, categorized to each of the provided pacakages, is listed below.
> [!NOTE]
> It is important to note that, by default, launch files will deploy nodes in the /eowyn/ namespace (/vicon/eowyn/ for vicon receiver).

### theo_core

#### Parameters for `mixer_node`

| Parameter            | Type     | Default           | Description |
|---|---:|---|---|
| `r_wheel`     | float | `1`     | radius of wheel in meters |
| `lx`          | float | `1`     | x-horizontal distance between wheel in meters |
| `ly`          | float | `1`     | y-horizontal distance between wheel in meters |
| `omega_ref`   | float | `1`     | reference wheel speed for max throttle (calibrated) |
| `flipdir_0`   | bool  | `false` | flip direction of wheel 0 in mixer |
| `flipdir_1`   | bool  | `false` | flip direction of wheel 1 in mixer |
| `flipdir_2`   | bool  | `false` | flip direction of wheel 2 in mixer |
| `flipdir_3`   | bool  | `false` | flip direction of wheel 3 in mixer |


#### Parameters for `pantilt_node`

| Parameter            | Type     | Default           | Description |
|---|---:|---|---|
| `pca_channels`    | int       | `None` | Channels in PCA9685                              |
| `init_cmd`        | list[2]   | `None` | Initial command to servos at spin-up             |
| `actuation_range` | list[2]   | `None` | Servo range of actuation                         |
| `pca_min_max_pwr` | list[2]   | `None` | Min/Max pulse widths of the respective servos    |
| `safety_bounds`   | list[2]   | `None` | Safety bounds of servos                          |


#### Parameters for `sabertooth_node`

| Parameter            | Type     | Default           | Description |
|---|---:|---|---|
| `portname_0` | string  | `/dev/ttyTHS1` | portname for first saberooth module    |
| `portname_1` | string  | `/dev/ttyTHS2` | portname for second saberooth module    |
| `baudrate`   | int32   | `9600`           | set baudrate of both sabertooths        |


### theo_teleop

#### Parameters for `joymapping_node`

| Parameter            | Type     | Default           | Description |
|---|---:|---|---|
| `chassis_max_linspeed`    | float | `1`   | maximum linear speed in meters per second   |
| `chassis_max_angspeed`    | float | `1`   | maximum linear speed in radian per second   |
| `pantilt_maxspeed`        | float | `1`   | maximum servo speed in radian per second    |
| `settings_update_waitime` | float | `0.5` | waittime for updating settings              |


### theo_autoop

#### Parameters for `pid_controller_node`

| Parameter            | Type     | Default           | Description |
|---|---:|---|---|
| `p_linear_gain`                   | float | `0`     | proporational gain for translational controller   |
| `i_linear_gain`                   | float | `0`     | proporational gain for translational controller   |
| `d_linear_gain`                   | float | `0`     | proporational gain for translational controller    |
| `p_angular_gain`                  | float | `0`     | proporational gain for translational controller   |
| `i_angular_gain`                  | float | `0`     | proporational gain for translational controller   |
| `d_angular_gain`                  | float | `0`     | proporational gain for translational controller    |
| `proximity_met_range_meters`      | float | `0.05`  | proximity range error to confirm configuration in meters    |
| `proximity_met_direction_degrees` | float | `5`     | proximity direction error to confirm configuration in degrees  |

### theo_comm

#### Parameters for `trajectory_transmitter_node`

| Parameter            | Type     | Default           | Description |
|---|---:|---|---|
| `filename`                        | string | `""`    | File path to input CSV file   |
| `delay_time`                      | float  | `0`     | Delay time to start transmitting trajectory after request/configuratoin confirmation   |


---

## Execution
### Transmitting CSV Trajectory from External Machine to a *Theodwyn* Robot
#### (1a) Starting Autonomous and Tele-Operation Nodes Onboard *Theodwyn* Robot
The robot must, at some point prior to operations, spin up its ROS nodes onboard. The following in the top-level directory, regardless of when and how it is executed, will do so
```bash
source install/setup.bash                       # ONLY needs to be run once per shell session
```
```bash
ros2 launch theo_core autoop_drive.launch       # configurations may need to be changed in case of unexpected behavior
```

> [!WARNING]
> The user profile on the *Theodwyn* robot is required to be a member of the following user groups for onboard communications during robot operations: `dialout`, `gpio`, `i2c`, `tty`

#### (1b) Starting Trajectory Transmission from External Computer
The external machine/computer will spin up the transmission and vicon receiver nodes, external to the *Theodwyn* Robot. After sourcing the `install/setup.bash`, say, in the current directory, the user has a csv file named `csv_out.csv`. The following can be used to transmit the trajectory, assuming the *Theodwyn* Robot and external machine are on the same local network.
```bash
CSV_FILE=$(realpath "csv_out.csv")
```
```bash
ros2 launch theo_comm transmit_trajectory.launch.py transmit_file_path:=$CSV_FILE
```