# Use ROS Noetic base image (Ubuntu 20.04)
FROM ros:noetic-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install essential packages
RUN apt-get update && apt-get install -y \
    git \
    cmake \
    udev \
    build-essential \
    libopencv-dev \
    libeigen3-dev \
    ros-noetic-cv-bridge \
    ros-noetic-dynamic-reconfigure \
    ros-noetic-sensor-msgs \
    ros-noetic-roscpp \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
	libusb-1.0-0-dev \ 
	libhidapi-libusb0 \ 
	libhidapi-dev \
	python3-catkin-tools
	
# Install Python dependencies
RUN apt-get update && apt-get install -y \ 
	python3-pip 
	
RUN pip3 install numpy opencv-python

# Create catkin workspace
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws

# Install zed-open-capture
RUN git clone https://github.com/stereolabs/zed-open-capture.git src/zed-open-capture
WORKDIR /catkin_ws/src/zed-open-capture/udev
RUN bash install_udev_rule.sh
WORKDIR /catkin_ws/src/zed-open-capture
RUN mkdir build && cd build && cmake .. && make -j$(nproc)
RUN cd build && sudo make install && sudo ldconfig
WORKDIR /catkin_ws

#install ros wrapper
RUN git clone https://github.com/timengelbracht/zed_open_capture_ros.git src/zed_open_capture_ros


WORKDIR /catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    cd /catkin_ws && catkin config --extend /opt/ros/noetic && catkin build"


# Source ROS environment
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]
