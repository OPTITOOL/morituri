#!/bin/bash

vcpkgPath=/vcpkg

cd /

####### vcpkg
rm -rf $vcpkgPath
git clone https://github.com/microsoft/vcpkg.git
cd $vcpkgPath

#init vcpkg
./bootstrap-vcpkg.sh

#build linux dependenciess
./vcpkg install --clean-after-build 