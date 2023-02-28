#!/bin/bash

vcpkgPath=~/vcpkg


cd $vcpkgPath

#init vcpkg
./bootstrap-vcpkg.sh

#build linux dependenciess
./vcpkg install --recurse --clean-after-build gdal[core] geos boost-log boost-program-options boost-locale shapelib libosmium