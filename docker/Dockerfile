FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-xacro \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-joint-state-publisher \
        ros-jazzy-rviz2 \
        ros-jazzy-nav2-bringup \
        ros-jazzy-slam-toolbox \
        ros-jazzy-robot-localization \
        ros-jazzy-ros-gz \
        ros-jazzy-teleop-twist-keyboard \
        ros-jazzy-tf2-tools \
        ros-jazzy-nav2-simple-commander \
        python3-numpy \
        python3-scipy \
        python3-pytest \
        python3-transforms3d \
        mesa-utils \
        x11-utils \
        xauth \
        nano \
        less \
    && rm -rf /var/lib/apt/lists/*

# Qt apps (rviz2, gz gui) must talk X11 through XWayland inside the container.
ENV QT_QPA_PLATFORM=xcb
# Software rendering by default; override with LIBGL_ALWAYS_SOFTWARE=0 when
# /dev/dri is passed through for the AMD iGPU.
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV ROS_DOMAIN_ID=42

WORKDIR /work
CMD ["bash"]
