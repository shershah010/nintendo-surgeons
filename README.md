# nintendo surgeons

## Overview
The use of soft robot grippers holds many applications for specialized manipulation tasks, including those where objects may be in a variety of shapes or require delicate handling. We aim to utilize vision and bend sensor data to build a platform allowing for testing of various models of a soft robot finger, including that proposed by [Daniela Rus](http://www.centropiaggio.unipi.it/sites/default/files/roso18_0123_fi.pdf). Additionally, we strive to build an effective controller for the soft finger that uses the enhanced sensory data we receive. With these in place, we will work to characterize the dynamics of a grasp using soft fingers and evaluate the performance of existing models to determine the most optimal. This ROS package is designed to test both the soft gripper and the controllers.

## Start Up
1. Open a terminal on the lab computers
2. Run `roscore`
3. Go to the `~/ros_workspaces/lab4` and run `source devel/setup.bash`
4. Then run `rosrun lab4_pkg soft_gripper_interface.py`
5. Run `rosrun lab4_pkg record_data.py`

## Folder Descriptions

### Ardunio/motor
Holds a single file that takes the outputs from the sensor and streams it so that ROS can collect it.

### data
Holds the data that is recorded by all the sensors used. This includes the `time`, `pwm` values, `pressure`, and `deflection`. Additionally, this folder holds an ipython file that preforms data analysis.

### launch
This folder contains a single launch file that runs the entire package with the following parameters and their default values: `video_device: /dev/video0`, `image_width: 640`, `image_height: 480`, `pixel_format: yuyu`, `camera_frame_id: usb_cam`, `io_method: mmap`.

### matlab
This folder holds all matlab code, including generated files.
- `LagrangianDynamics` - given kinetic energy, potential energy, position, velocity, and q_act?, this function returns D? C? G? B?.
- `cost_function` - given stiffness, damping, input gain, input inertia, an array of timesteps, an array of pwm inputs, and array of angle measurements, an initial angle, an initial velocity, an initial torque, and an initial jerk, calculate the predicted angular measurements and return the error.
- `data analysis` - given a csv file, calculate the optimal stiffness, damping, input gain, and input inertia for a particular soft gripper model using linear regression.
- `dhp` - stores twists as  Denavit–Hartenberg parameters.
- `find_q` - given an array of timesteps, an array of torque inputs, stiffness, damping, and initial angle, and an initial velcity, return the predicted angular at the next timestep.
- `find_tau` - given an array of timesteps, an array of pwm inputs, input gain, input inertia, initial torque, and initial jerk, return the predicted torque at the next timestep.
- `finger_dynamics` - symbolically develop the dynamics of the system and store them as functions in the `gen` folder. The system is modeled as a revolute-prismatic-prismatic-revolute (RPPR) robot.
- `finger_transforms` - given an initial position, and length, return the DH parameters for each of the joints.
- `importdata` - given a file, a start row, and an end row, return the data in the file as a matrix.
- `tau-dynamics` - given an current tau, current jerk, inertia and gain, return the current jerk and jounce/snap.

### msg
This folder holds two ROS messages that returns the information from the Ardunio to the main computer.

### scripts
This folder holds ROS scripts to handle the IO between the main computer, Ardunio, and the soft gripper.
