#!/bin/bash

vcpkgPath=~/vcpkg

cd ~

####### vcpkg
rm -rf $vcpkgPath
git clone -b fix-shapelib --depth 1 https://github.com/OPTITOOL/vcpkg.git
cd $vcpkgPath

#init vcpkg
./bootstrap-vcpkg.sh

#build linux dependenciess
./vcpkg install --clean-after-build gdal[geos] boost-log boost-program-options boost-locale boost-timer boost-iostreams shapelib libosmium