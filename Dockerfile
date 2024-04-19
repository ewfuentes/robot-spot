FROM nvidia/cuda:12.2.0-devel-ubuntu20.04

 
# Minimal setup
RUN apt-get update \
 && apt-get install -y locales lsb-release tmux
ENV TZ=America/New_York
ARG USERNAME=robot
ARG USER_UID=1000
ARG USER_GID=1000
RUN DEBIAN_FRONTEND=noninteractive TZ=America/New_York dpkg-reconfigure locales
 
# Install ROS Noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN apt-get update \
 && DEBIAN_FRONTEND=noninteractive TZ=America/New_York apt-get install -y --no-install-recommends \
 ros-noetic-desktop-full python3-rosdep git curl python3-rosinstall python3-rosinstall-generator \
 python3-vcstool build-essential python3-catkin-tools python-is-python3

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
USER $USERNAME


RUN sudo rosdep init \
 && rosdep fix-permissions \
 && rosdep update


RUN DEBIAN_FRONTEND=noninteractive sudo apt install -y python3-pip ros-noetic-ackermann-msgs \
  wget libtool m4 automake iproute2 inetutils-ping vim less

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /host/catkin_ws/devel/setup.bash" >> ~/.bashrc
RUN echo "source /host/.secrets" >> ~/.bashrc
COPY .tmux.conf /home/$USERNAME/.tmux.conf
WORKDIR /host
