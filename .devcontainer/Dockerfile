FROM ubuntu:22.04

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG GCC_VERSION=13.1.0

ENV EDITOR=nano

RUN apt-get update && apt-get upgrade --yes && \
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
    gcovr \
    make \
    libbz2-dev \ 
    cppcheck \
    flawfinder \
    gdb \
    locales \
    clang-format \
    libgmp-dev \
    libmpfr-dev \
    libmpc-dev \
    libisl-dev && \

    #build gcc
    wget -q https://ftp.gnu.org/gnu/gcc/gcc-${GCC_VERSION}/gcc-${GCC_VERSION}.tar.xz -O - | tar -xJ && \
    \
    cd gcc-${GCC_VERSION} && \
    \
    ./configure \
    --enable-languages=c,c++ \
    --disable-multilib \
    --with-system-zlib && \
    make -j`nproc` && \
    make install && \
    cd .. && \
    rm -r gcc gcc-${GCC_VERSION} && \


    # cleanup
    apt-get remove --purge -y gnupg libgmp-dev libmpfr-dev libmpc-dev libisl-dev && \
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
RUN ./installvcpkg.sh

