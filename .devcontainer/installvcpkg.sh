#!/bin/bash

vcpkgPath=/vcpkg

cd /

####### vcpkg
rm -rf $vcpkgPath
git clone -b fix-shapelib https://github.com/OPTITOOL/vcpkg.git
cd $vcpkgPath

#init vcpkg
./bootstrap-vcpkg.sh

#build linux dependenciess
./vcpkg install --clean-after-build 