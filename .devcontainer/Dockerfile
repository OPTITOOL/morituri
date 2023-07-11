FROM ubuntu:22.04

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ENV EDITOR=nano

RUN apt-get update && \
    DEBIAN_FRONTEND="noninteractive" apt-get install --yes \
    software-properties-common && \
    # gcc 13 ppa   
    add-apt-repository ppa:ubuntu-toolchain-r/test && \  
    apt-get update && apt-get upgrade --yes && \
    DEBIAN_FRONTEND="noninteractive" apt-get install --yes \
    bash \
    build-essential \
    curl \
    htop \
    locales \
    man \
    python3 \
    python3-pip \
    sudo \
    unzip \
    wget \
    git \
    cmake \
    ninja-build \
    apt-utils \
    apt-transport-https \
    tar \
    zip \
    pkg-config \
    gcc-12 \
    g++-12 \
    gcovr \
    make \
    cppcheck \
    flawfinder \
    gdb \
    locales \
    clang-format && \

    # cleanup
    apt-get remove --purge -y gnupg && \
    apt-get autoremove --purge -y && \
    apt-get clean

# Set the locale
RUN sed -i '/en_US.UTF-8/s/^# //g' /etc/locale.gen && \
    locale-gen
ENV LANG en_US.UTF-8  
ENV LANGUAGE en_US:en  
ENV LC_ALL en_US.UTF-8 

COPY installvcpkg.sh installvcpkg.sh
COPY vcpkg.json vcpkg.json

# create dependencies
#RUN ./installvcpkg.sh

