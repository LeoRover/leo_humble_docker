FROM ros:humble-ros-base-jammy

# Add fictionlab apt archive
RUN curl -sSL https://archive.fictionlab.pl/fictionlab.gpg -o /usr/share/keyrings/fictionlab-archive-keyring.gpg
RUN echo \
      "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/fictionlab-archive-keyring.gpg] https://archive.fictionlab.pl \
      $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
      tee /etc/apt/sources.list.d/fictionlab.list > /dev/null

# Install libcamera dependencies
RUN apt-get update \
      && apt-get install -y libpisp-dev python3-pip python3-jinja2 python3-ply ninja-build \
      && pip3 install meson

# Build and install libcamera
RUN git clone -b v0.5.0+rpt20250429 https://github.com/raspberrypi/libcamera.git \
      && cd libcamera \
      && meson setup build --buildtype=release \
      -Dpipelines=rpi/vc4,rpi/pisp \
      -Dipas=rpi/vc4,rpi/pisp \
      -Dv4l2=true \
      -Dgstreamer=disabled \
      -Dtest=false \
      -Dlc-compliance=disabled \
      -Dcam=disabled \
      -Dqcam=disabled \
      -Ddocumentation=disabled \
      -Dpycamera=enabled \
      && ninja -C build install

# Clone and build leo packages
RUN mkdir -p /root/ros_ws/src
WORKDIR /root/ros_ws

RUN git clone -b humble-backports https://github.com/LeoRover/leo_robot-ros2.git src/leo_robot-ros2
RUN git clone -b 0.3.0 https://github.com/LeoRover/leo_camera_ros.git src/leo_camera_ros

RUN apt-get update && rosdep update && \
      rosdep install --from-paths src --ignore-src -y --skip-keys "libcamera-dev libcamera"

RUN . /opt/ros/humble/setup.sh && \
      colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

# Install additional ROS packages
RUN apt-get install -y \
      ros-humble-micro-ros-agent \
      ros-humble-compressed-image-transport

COPY ./entrypoint.sh /root/entrypoint.sh
RUN chmod +x /root/entrypoint.sh

# Commands to build and run the container
#sudo docker build -t leo_humble_1_9 .

#sudo docker run \
#  --privileged \
#  --net=host \
#  -v /dev:/dev \
#  -v /usr/lib/ros/ros-nodes:/usr/lib/ros/ros-nodes \
#  -v /usr/lib/ros/uros-agent:/usr/lib/ros/uros-agent \
#  -v /etc/ros:/etc/ros \
#  -v /home/pi/.ros:/root/.ros \
#  -it --rm leo_humble_1_9 \
#  uros-agent

ENTRYPOINT [ "/root/entrypoint.sh" ]
