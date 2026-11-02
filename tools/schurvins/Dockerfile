ARG BASE_IMAGE=ros:melodic-ros-base-bionic@sha256:3444627c80c3d5d4179058e5e9254233df2e0a58fa83fc986da96deea1c720a5
FROM ${BASE_IMAGE}
ENV DEBIAN_FRONTEND=noninteractive
RUN sed -i 's|http://security.ubuntu.com/ubuntu|http://archive.ubuntu.com/ubuntu|g' /etc/apt/sources.list \
    && apt-get -o Acquire::Retries=3 update \
    && apt-get -o Acquire::Retries=3 install -y --no-install-recommends \
    build-essential cmake git ca-certificates patch python-catkin-tools \
    python-numpy python-opencv python-yaml libeigen3-dev libopencv-dev \
    libglew-dev libyaml-cpp-dev libblas-dev liblapack-dev libsuitesparse-dev \
    libceres-dev libgoogle-glog-dev libgflags-dev libboost-all-dev \
    ros-melodic-cv-bridge ros-melodic-image-transport ros-melodic-pcl-ros \
    ros-melodic-tf ros-melodic-cmake-modules ros-melodic-rostest \
    time && dpkg-query -W > /opt/schurvins-packages.txt \
    && rm -rf /var/lib/apt/lists/*
ARG BUILD_UID=1000
ARG BUILD_GID=1000
RUN groupadd --gid ${BUILD_GID} rm27 && useradd --uid ${BUILD_UID} --gid ${BUILD_GID} --create-home rm27
COPY packages.lock /opt/schurvins-packages.lock
# Fail closed if the repositories can no longer reproduce the recorded closure.
RUN diff -u /opt/schurvins-packages.lock /opt/schurvins-packages.txt
WORKDIR /ws
