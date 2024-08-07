# larcc_drivers

This repository contains the drivers and configurations needed to work with the LAR Collaborative Cell. It was built to work with:
* UR10e manipulator (Universal Robot 10 e-series)
* Ubuntu 20.04.3 LTS or other Linux distros using Docker
* ROS Noetic

## Table of Contents

1. [UR10e Controller Configuration](#ur10e-controller-configuration)
2. [Installation](#installation)
3. [Controlling the UR10e through MoveIt](#controlling-the-ur10e-through-moveit)
4. [Real-time UR10e following a tracked object](#real-time-ur10e-following-a-tracked-object)
5. [Gripper Remote Control](#gripper-remote-control)
6. [Real-time UR10e following & picking a tracked object](#real-time-ur10e-following-&-picking-a-tracked-object)


## UR10e Controller Configuration

This configuration only needs to be done once when setting up the robot.

<details>
<summary>Click for details...</summary>

For working on a real robot you need to install the [externalcontrol-1.0.5.urcap](https://github.com/lardemua/larcc_drivers/blob/master/resources/externalcontrol-1.0.5.urcap) which can be found inside the resources folder of this repository.

Using a USB pen drive, follow:
1. Format the flash drive
2. Download and save the externalcontrol-1.0.5.urcap on the USB pen drive
3. Insert the USB drive on UR10e controller (the controller has two USB ports)

![controller-ports](docs/controller_ports.png)

4. Turn on the Teach Pendant
 
![tp1](docs/es_01_welcome.png)

5. Click on *Menu* (top right corner) + *System* + *URCaps* + Select *External Control* and press "+"

![tp2](docs/es_05_urcaps_installed.png)

6. Configure the remote host's IP to ```192.168.56.1```

![tp3](docs/es_07_installation_excontrol.png)

7. Click on *Menu* (top right corner) + *System* + *Network*
8. Configure:
   1. Network method : Static Address
   2. IP address: ```192.168.56.2```
   3. Subnet mask: ```255.255.255.0```
   4. Default gateway: ```192.168.56.2```

9. Click on *Apply*

![tp4](docs/tp1.jpg)

10. Disable EtherNet/IP fieldbus:

Installation > Fieldbus > EtherNet/IP > Disable

![tp5](docs/tp_ethernet_fieldbus.png)

</details>

## Installation

### On Ubuntu 20

<details>
<summary>Click for details...</summary>
First, it is required to have MoveIt and other packages installed in your system:

```
sudo apt install ros-noetic-moveit ros-noetic-industrial-robot-status-interface ros-noetic-scaled-controllers ros-noetic-pass-through-controllers ros-noetic-ur-client-library ros-noetic-ur-msgs ros-noetic-velocity-controllers ros-noetic-force-torque-sensor-controller socat
```

(**Note:** At this moment, if you do not have a catkin workspace, you should now create one, by following the steps described [here](http://wiki.ros.org/catkin/Tutorials/create_a_workspace))

After all these installations, on your catkin workspace you need to clone this repository:

```
cd catkin_ws/src
git clone https://github.com/lardemua/larcc_drivers.git
```

So that packages are able to import from each other, run the following:

```
echo "export PYTHONPATH=\$PYTHONPATH:~/catkin_ws/src/larcc_drivers" >> ~/.bashrc
source ~/.bashrc
```

troubleshooting: in case protobuf version >=3.20:

```
pip install protobuf==3.19.*
```

Now compile your catkin workspace:

```
cd ~/catkin_ws
catkin_make
```

Finally, to establish the communication between the robot and the computer, it is required to **connect an Ethernet cable from the UR10e controller to the computer**.
After you connect the cable, you need to configure the IPv4 like this:

![tp6](docs/ip.jpeg)

#### Set up camera dependencies

1. For the astra_camera package, install the following dependencies:

   ```
   sudo apt-get install -y libgflags-dev ros-noetic-image-geometry ros-noetic-camera-info-manager \
      ros-noetic-image-transport ros-noetic-image-publisher libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev
   ```

   and install libuvc:

   ```
      git clone https://github.com/libuvc/libuvc.git
      cd libuvc
      mkdir build && cd build
      cmake .. && make -j4
      sudo make install
      sudo ldconfig
   ```

   Then clone the [ros_astra_camera repository](https://github.com/orbbec/ros_astra_camera):
   
   ```
   cd ~/catkin_ws/src
   git clone https://github.com/orbbec/ros_astra_camera.git
   ```
   
   Compile your catkin workspace:

   ```
   cd ~/catkin_ws
   catkin_make
   ```

   Install udev rules:

   ```
   roscd astra_camera
   sudo cp 56-orbbec-usb.rules /etc/udev/rules.d
   sudo service udev reload
   sudo service udev restart
   sudo udevadm control --reload && sudo  udevadm trigger
   ```

2. For the usb_cam package, install the following dependencies:

   ```
   sudo apt install libv4l-dev v4l-utils
   ```
   
   Then clone the [usb_cam repository](https://github.com/lardemua/usb_cam):
   
   ```
   cd ~/catkin_ws/src
   git clone https://github.com/lardemua/usb_cam.git
   ```
   
   Finally, compile your catkin workspace.

   ```
   cd ~/catkin_ws
   catkin_make
   ```
</details>

### Using Docker (recommended)

After installing [Docker](https://docs.docker.com/engine/install/ubuntu/) and [Distrobox](https://github.com/89luca89/distrobox?tab=readme-ov-file#installation), build one of the docker images with ROS Noetic available at [lar_docker](https://github.com/lardemua/lar_docker). The Dockerfile available in this repository uses the one with Cuda 11.8 by default.

Next build the larcc image from the Dockerfile in this repository:

```
docker build -t lardemua/larcc-distrobox .
```

Create and enter the created distrobox:

```
SHELL=/bin/bash distrobox create --image lardemua/larcc-distrobox --name larcc-distrobox --additional-flags "--gpus all" --home $HOME/larcc-distrobox
distrobox enter larcc-distrobox
```

Set up ROS for your user inside the distrobox:

```
/user-setup.sh
```

Clone the [usb_cam repository](https://github.com/lardemua/usb_cam) and the [ros_astra_camera repository](https://github.com/orbbec/ros_astra_camera):

```
cd ~/catkin_ws/src
git clone https://github.com/lardemua/usb_cam.git
git clone https://github.com/orbbec/ros_astra_camera.git
```

Compile your catkin workspace.

```
cd ~/catkin_ws
catkin_make
```

Install udev rules by running the following outside the container:

```
cd $HOME/larcc-distrobox/catkin_ws/src/ros_astra_camera
sudo cp 56-orbbec-usb.rules /etc/udev/rules.d
sudo service udev reload
sudo service udev restart
sudo udevadm control --reload && sudo  udevadm trigger
```

## Controlling the UR10e through MoveIt

Follow the next 4 steps to remotely control the real UR10e robot:

1. Connect the robot to your computer using the Ethernet cable.

2. Launch the robot drivers with:

   ```
   roslaunch larcc_bringup ur10e_bringup.launch
   ```

3. Run the external control program on the teach pendant:

   Click on *Program* + *URCaps* + *External Control* + Press "play"

   ![tp7](docs/tp2.jpg)

   At this point, you should get the message "_Robot connected to reverse interface. Ready to receive control commands._" printed out on your terminal window, just like this:

   ![tp8](docs/you_can_start_planning.png)

4. Then you have 3 options to control the robot:
   
   - control the robot using a Python script for joint positions:

      ```
      roslaunch larcc_examples arm_gripper_movement_joints.launch
      ```

   - control the robot using a Python script for 3D positions with quaternions:

      ```
      roslaunch larcc_examples arm_gripper_movement_quaternions.launch
      ```
   
   - control the robot using the MoveIt Rviz interface:

      ```
      roslaunch ur10e_moveit_config moveit_rviz.launch config:=true
      ```

      With this interface, you can control the real robot by simply moving the manipulator marker on RViz, and then asking the robot to move to that goal (using the Motion Planning Panel). MoveIt will plan the trajetory.

      [//]: # (![tp9]&#40;docs/UR10e_moving_moveit.gif&#41;)
      ![tp9](docs/ur10e_external_control.gif)

## Real-time UR10e following a tracked object
By using ViSP RGB-D object tracking (see [this repository](https://github.com/afonsocastro/generic-rgbd)) and MoveIt to remotely control the UR10e,
it was developed this demonstration of a real-time followed object that is being tracked with one RGB-D camera (Intel RealSense D435):

![rgbd_tracking_ur10e_following](docs/RGBD_tracking_n_control_demo.gif)

## Gripper Remote Control
In order to control the ROBOTIQ 2F-140 gripper from an external computer, it is required to:
1. Turn on the robot (UR10e)
2. Connect the TCP/IP communication between the robot and the computer
3. **Remove/Uninstall any ROBOTIQ URCap**: remove 'Robotiq_Grippers' on UR10e Teach Pendant (Click on Menu (top right corner) + System + URCaps)
4. **Install the [rs485-1.0.urcap](https://github.com/afonsocastro/larcc_interface/blob/master/resources/rs485-1.0.urcap)**. See the first 5 points of [on-ur10e-controller](#on-ur10e-controller).

Finally, for testing:

7. ```cd larcc_interface/gripper/src```

8. ```python3 test_robotiq.py ```

[//]: # (![gripper_open_close]&#40;docs/Gripper_Open_Close.gif&#41;)

[//]: # (![gripper_open_close])
<p align="center">
<img src="docs/Gripper_Open_Close.gif" width="25%" height="25%"/>
</p>

The Python module for controlling Robotiq 2F-140 from an external PC can be found [here](https://github.com/afonsocastro/larcc_interface/tree/master/gripper/src).

On _test_robotiq.py_, you may have to change HOST ip address to UR10e.

Notes:
1. UR10e can either be on _Remote Control_ or in _Local Control_ mode (does not make any difference).
2. It is not required to launch any robot driver. The TCP/IP connection is enough, since the RS485 communication will be directly established between the gripper and the external PC
3. It is necessary rs485 URcap. That is daemon for communicate between UR10e's rs485 network and external pc via tcp (port 54321). RobotiqHand module use this mechanism. So you must activate rs485 URCap on UR10e. And, If you activate Robotiq_Gripper URCap on UR10e, that URCap always use inner rs485 network without exclusive. This means external rs485 communication make conflict communication contents. So if you want to control 2f-140 via rs485 from external pc, you must deactivate Robotiq_Gripper URCap on UR10e
 
## Real-time UR10e following & picking a tracked object

![arm_and_gripper_control](docs/Arm_and_Gripper_Control.gif)
