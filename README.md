# MICO 

All MICO-plugins in one repository using git submodules.

## Get it from last release!

The easiest way to start using mico is to download it from the last release webpage. Click here and select the proper file for your OS.

## Building for sources

If you want, you can compile it from the sources. To do so, we recomend you to use VCPKG. But first, you need to prepare your system.

1. If you are using ubuntu, install some dependencies
```
sudo apt-get install libssl-dev
sudo apt-get install -y curl zip unzip tar
sudo apt-get install -y autoconf libtool bison python3 python3-distutils gperf nasm
sudo apt-get install -y libx11-dev libxft-dev libxext-dev libgles2-mesa-dev libxrandr-dev libxi-dev libxcursor-dev libxdamage-dev libxinerama-dev 
sudo apt-get install -y libx11-xcb-dev libglu1-mesa-dev libgl1-mesa-dev
```
2. Make sure you have at least CMake 3.21
3. Clone VCPKG wherever you want. And, if you are using Windows, create a system variable called VCPKG_ROOT pointing to the repository.
```
git clone https://github.com/microsoft/vcpkg
```
4. Time to configure the compilation. If you are in Windows, and you use Visual Studio 2022. Just open the repo. If you are in ubuntu, go to the repository folder, open a terminal and type:
```
cmake . --preset=Ubuntu -DCMAKE_TOOLCHAIN_FILE=/home/bardo91/programming/vcpkg/scripts/buildsystems/vcpkg.cmake
```
This last step might take some time.... In my case it took 3 hours to compile PCL and Qt5. Be patient.

And that's it, you are ready to compile. In MSVC you can select the project you want to compile. In ubuntu, go to the out/build/Ubuntu folder and type make to compile all!

BTW: If your computer get stuck compiling the dependencies with VPKG (Which happens typically compiling PCL or QT5), I recomend you to limit the number of threads for VCPKG. To do so, befor calling `cmake . --preset....`, type `export VCPKG_MAX_CONCURRENCY=4` in your screen (o whatever number of threads you want). It happens because compiling and linking those libraries consumes a lot of RAM, and if you have many threads compiling/linking at the same time the PC might get run out of memory, and the OS get stuck.

## Alternative to VCPKG in Ubuntu to compile MICO

You can just call cmake, without specifying the TOOLCHAIN file. To make it work, you need to install all the dependencies manually. In Ubuntu it is not very hard. 

```
sudo apt-get install qt5-default libeigen3-dev libboost-all-dev libpcl-dev libopencv-dev
```
* Pangolin: https://github.com/stevenlovegrove/Pangolin
* Pybind11: https://github.com/pybind/pybind11
* Python3 with numpy
