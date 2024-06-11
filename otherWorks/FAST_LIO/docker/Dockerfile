ARG BASE_IMAGE=osrf/ros:noetic-desktop-full
FROM ${BASE_IMAGE}

# 替换为清华源
RUN sed -i 's/archive.ubuntu.com/mirrors.tuna.tsinghua.edu.cn/g' /etc/apt/sources.list

# 时区：上海；Debian包管理器的前端改为非交互模式
ENV TZ=Asia/Shanghai DEBIAN_FRONTEND=noninteractive

# 安装一些基本软件包
RUN apt-get update \
    && apt-get install -y tzdata vim git zsh build-essential sudo apt-utils \
    curl gdb gdbserver && rm -rf /var/lib/apt/lists/*

# 配置时区
RUN ln -fs /usr/share/zoneinfo/${TZ} /etc/localtime \
    && echo ${TZ} > /etc/timezone \
    && dpkg-reconfigure --frontend noninteractive tzdata

# 安装如eigen,opencv之类的库
RUN apt-get update -y && apt-get install -y libeigen3-dev libpcap-dev python3 \
        python3-dev python3-pip cmake libopencv-dev python3-opencv python3-venv python3-wheel \
        libgoogle-glog-dev libgflags-dev libatlas-base-dev libsuitesparse-dev \
        mesa-utils libgl1-mesa-glx libgl1-mesa-dri libomp-dev \
    && rm -rf /var/lib/apt/lists/*

# 将 /usr/bin/python 链接到 /usr/bin/python3
RUN #ln -s /usr/bin/python3 /usr/bin/python

# 使用阿里云的pypi镜像
RUN pip3 config set global.index-url http://mirrors.aliyun.com/pypi/simple
RUN pip3 config set install.trusted-host mirrors.aliyun.com
RUN pip3 install --upgrade pip

# 安装一些ros库
RUN apt-get update -y && sudo apt-get install -y \
                            ros-$ROS_DISTRO-jsk-common-msgs \
                            ros-$ROS_DISTRO-jsk-recognition-msgs \
                            ros-$ROS_DISTRO-jsk-rviz-plugins \
                            ros-$ROS_DISTRO-jsk-rqt-plugins \
                            ros-$ROS_DISTRO-jsk-visualization \
                            ros-$ROS_DISTRO-swri-profiler \
                            ros-$ROS_DISTRO-swri-profiler-tools \
                            ros-$ROS_DISTRO-rosfmt \
                            ros-$ROS_DISTRO-cv-bridge \
                            ros-$ROS_DISTRO-tf \
                            ros-$ROS_DISTRO-rosfmt \
                            ros-$ROS_DISTRO-message-filters \
                            ros-$ROS_DISTRO-image-transport \
                            ros-$ROS_DISTRO-rviz \
                            libtbb-dev proj-bin libparmetis-dev && \
                            rm -rf /var/lib/apt/lists/*

COPY docker/install/install_livox_sdk.sh /opt/
RUN  /opt/install_livox_sdk.sh

# COPY docker/install/install_ceres.sh /opt/
# RUN /opt/install_ceres.sh

# COPY docker/install/install_json.sh /opt/
# RUN /opt/install_json.sh

# COPY docker/install/install_gtsam.sh /opt/
# RUN /opt/install_gtsam.sh

# COPY docker/install/install_ohmyzsh.sh /opt/
# RUN /opt/install_ohmyzsh.sh

# 配置hx环境
ARG USERNAME
ARG PASSWORD
RUN python3 -m venv /venv
COPY docker/install/install_hxbasic.sh /opt/auto4d-ground/
RUN PATH=/venv/bin:$PATH /opt/auto4d-ground/install_hxbasic.sh

# Add new sudo user
ARG USER=calib
ARG PASSWD=calib
ARG UID=1000
ARG GID=1000

ENV _USER=$USER
ENV _PASSWD=$PASSWD
ENV _UID=$UID
ENV _GID=$GID

RUN useradd --create-home -m $_USER                                && \
    echo "$_USER:$_PASSWD" | chpasswd                              && \
    usermod --shell /bin/bash $_USER                               && \
    usermod -aG sudo $_USER                                        && \
    echo "$_USER ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers.d/$_USER && \
    chmod 0440 /etc/sudoers.d/$_USER                               && \
    usermod  --uid $_UID $_USER                                    && \
    groupmod --gid $_GID $_USER

RUN mkdir -p /workspace/src/ && chown -R ${_USER}:${_USER} /workspace

# COPY requirements.txt /opt/
# RUN /venv/bin/pip3 install -r /opt/requirements.txt

USER ${_USER}

ENV PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/workspace
ENV ROS_DISTRO=noetic
ENV ROS_VERSION=1
ENV PKG_CONFIG_PATH=/opt/ros/noetic/lib/pkgconfig
ENV ROS_PYTHON_VERSION=3
ENV ROS_PACKAGE_PATH=/opt/ros/noetic/share
ENV ROSLISP_PACKAGE_DIRECTORIES=
ENV ROS_ETC_DIR=/opt/ros/noetic/etc/ros
ENV CMAKE_PREFIX_PATH=/opt/ros/noetic
ENV ROS_MASTER_URI=http://localhost:11311
ENV ROS_ROOT=/opt/ros/noetic/share/ros
ENV PATH=/usr/local/cuda/bin:/opt/ros/noetic/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64:/opt/ros/noetic/lib:/workspace

COPY docker/install/install_livox_ros.sh /opt/
RUN /opt/install_livox_ros.sh
