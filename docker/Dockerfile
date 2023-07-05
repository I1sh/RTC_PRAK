ARG from

FROM ${from}

MAINTAINER Ivan Shevtsov "ishevtsov0108@gmail.com"

# Timezone Configuration
ENV TZ=Europe/Moscow
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt-get update && apt-get install -y apt-utils \
                                         lsb-release \
                                         mesa-utils \
                                         gnupg2 \
                                         net-tools \
                                         build-essential \
                                         wget \
                                         unzip \
                                         curl \
                                         git \
                                         mc \
                                         locales



RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8

# Install ROS 
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt update -y && DEBIAN_FRONTEND=noninteractive apt-get install -y ros-noetic-desktop-full && \
    echo "source /opt/ros/noetic/setup.bash"  >> ~/.bashrc && \
    echo "source /catkin_ws/devel/setup.bash"  >> ~/.bashrc

# Install catkin tools and additional packages
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y python3-rosdep \
                                                      python3-rosinstall \
                                                      python3-rosinstall-generator \ 
                                                      python3-wstool \ 
                                                      build-essential  && \
    rosdep init && rosdep update


# Install extra libraries
RUN apt install libeigen3-dev

# User WS dependings
COPY /catkin_ws /catkin_ws
RUN rosdep install --from-paths /catkin_ws --ignore-src --rosdistro=noetic -y 

RUN rm -R /catkin_ws
