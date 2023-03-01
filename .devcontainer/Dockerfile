FROM ubuntu:22.04

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ENV EDITOR=nano

RUN apt-get update && apt-get upgrade --yes

RUN apt-get update && \
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
    software-properties-common \
    tar \
    zip \
    pkg-config \
    gcc-12 \
    g++-12 \
    gcovr \
    cppcheck \
    autoconf \
    automake \
    autopoint \
    bash \
    bison \
    bzip2 \
    flex \
    g++ \
    g++-multilib \
    gettext \
    git \
    gperf \
    intltool \
    libbz2-dev \
    libc6-dev-i386 \
    libgdk-pixbuf2.0-dev \
    libltdl-dev \
    libssl-dev \
    libtool-bin \
    libxml-parser-perl \
    lzip \
    make \
    openssl \
    p7zip-full \
    patch \
    perl \
    python3 \
    python3-mako \
    ruby \
    sed \
    unzip \
    wget \
    xz-utils \
    python-is-python3 \
    nsis \
    flawfinder \
    gdb \
    locales \
    clang-format

# Set the locale
RUN sed -i '/en_US.UTF-8/s/^# //g' /etc/locale.gen && \
    locale-gen
ENV LANG en_US.UTF-8  
ENV LANGUAGE en_US:en  
ENV LC_ALL en_US.UTF-8 

RUN apt-get clean 

COPY installvcpkg.sh installvcpkg.sh

# create dependencies
RUN ./installvcpkg.sh