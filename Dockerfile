# Use distrobox image with ROS Noetic and CUDA 11.8 as base
FROM lardemua/ros-cuda-distrobox:noetic-11.8

RUN apt-get update && apt-get upgrade -y

# Install larcc_drivers dependencies
RUN apt-get install -y ros-noetic-moveit ros-noetic-industrial-robot-status-interface ros-noetic-scaled-controllers \
    ros-noetic-pass-through-controllers ros-noetic-ur-client-library ros-noetic-ur-msgs ros-noetic-velocity-controllers \
    ros-noetic-force-torque-sensor-controller socat

RUN apt-get install -y libv4l-dev v4l-utils

# Install astra_camera dependencies
RUN apt-get install -y libgflags-dev  ros-noetic-image-geometry ros-noetic-camera-info-manager \
    ros-noetic-image-transport ros-noetic-image-publisher libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev

RUN git clone https://github.com/libuvc/libuvc.git
RUN mkdir /libuvc/build
WORKDIR /libuvc/build
RUN cmake .. && make -j4
RUN sudo make install
RUN sudo ldconfig
WORKDIR /
RUN rm -rf /libuvc/

# Install Python dependencies
RUN pip install tensorflow[and-cuda] mediapipe scikit-learn matplotlib opencv-python \
    numpy==1.23.5 protobuf==3.20.3
## Potential problems:
## ERROR: torch 2.3.0 has requirement typing-extensions>=4.8.0, but you'll have typing-extensions 4.5.0 which is incompatible.
