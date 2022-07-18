# MICO - Modules-based Interface for Computer prOgramming

![](https://github.com/mico-corp/mico/blob/master/doc/example_play_ground.png)

## Get it from last release!

The easiest way to start using mico is to download it from the last release webpage. Click here and select the proper file for your OS.


## Building from sources in Ubuntu

To make it work, you need to install all the dependencies manually. In Ubuntu it is not very hard. 

```
sudo apt install -y git cmake qt5-default qtmultimedia5-dev libeigen3-dev libboost-all-dev libopencv-dev libpcl-dev freeglut3-dev libgraphviz-dev libgl1-mesa-dev mesa-common-dev libusb-1.0-0-dev libdlib-dev # Common dependencies
sudo apt install -y python3 python3-numpy                                                # For python and numpy
sudo apt-get install -y fftw3 fftw3-dev													# For audio module
sudo add-repository -y ppa:inivation-ppa/inivation
sudo apt update
sudo apt install -y dv-runtime-dev
```
Then, clone the repository, build the package and install it

```
git clone https://github.com/mico-corp/mico
cd mico
mkdir build
cd build
cmake .. -DBUILD_XXX=ON # Enable the packages you want...
make -j4
make package
sudo dpkg -i mico-1.1.0-Linux.deb
```

Now you should be able to run the app by typing in the terminal `flow_kids`

## Building from sources in Windows using VCPKG

If you want, you can compile it from the sources. To do so, we recomend you to use VCPKG. But first, you need to prepare your system.

1. Make sure you have at least CMake 3.21
2. Clone VCPKG wherever you want. And, if you are using Windows, create a system variable called VCPKG_ROOT pointing to the repository.
```
git clone https://github.com/microsoft/vcpkg
```
3. Install Visual Studio Community or Code
4. Open the project and configure it with the given CMakePresets.json file


## Building from sources in Ubuntu using VCPKG (Not working yet)

This method does not work properly yet. VCPKG in Linux compiles libraries as static, which shouldn't be a problem, but one of the libraries it uses 
does not compiles with fPIC, causing problems when generating mplugins (which are dynamic libraries and requires relocation)... So... Do not use this method yet.

1. If you are using ubuntu, install some dependencies
```
sudo apt-get install libssl-dev
sudo apt-get install -y curl zip unzip tar
sudo apt-get install -y autoconf libtool bison python3 python3-distutils gperf nasm
sudo apt-get install -y libx11-dev libxft-dev libxext-dev libgles2-mesa-dev libxrandr-dev libxi-dev libxcursor-dev libxdamage-dev libxinerama-dev 
sudo apt-get install -y libx11-xcb-dev libglu1-mesa-dev libgl1-mesa-dev
sudo apt-get install -y at curl unzip tar libxt-dev gperf libxaw7-dev cifs-utils \
  build-essential g++ gfortran zip libx11-dev libxkbcommon-x11-dev libxi-dev \
  libgl1-mesa-dev libglu1-mesa-dev mesa-common-dev libxinerama-dev libxxf86vm-dev \
  libxcursor-dev yasm libnuma1 libnuma-dev python-six python3-six python-yaml \
  flex libbison-dev autoconf libudev-dev libncurses5-dev libtool libxrandr-dev \
  xutils-dev dh-autoreconf autoconf-archive libgles2-mesa-dev ruby-full \
  pkg-config meson libxext-dev libxfixes-dev libxrender-dev \
  libxcb1-dev libx11-xcb-dev libxcb-glx0-dev libxcb-util0-dev libxkbcommon-dev libxcb-keysyms1-dev \
  libxcb-image0-dev libxcb-shm0-dev libxcb-icccm4-dev libxcb-sync0-dev \
  libxcb-xfixes0-dev libxcb-shape0-dev libxcb-randr0-dev \
  libxcb-render-util0-dev libxcb-xinerama0-dev libxcb-xkb-dev libxcb-xinput-dev libkrb5-dev \
  libxcb-res0-dev python3-setuptools python3-mako python3-pip python3-venv nodejs libwayland-dev \
  guile-2.2-dev libxdamage-dev liblttng-ust0 libkrb5-3 zlib1g libicu66
pip3 install meson==0.61.1
```
2. Make sure you have at least CMake 3.21
3. Clone VCPKG wherever you want. And, if you are using Windows, create a system variable called VCPKG_ROOT pointing to the repository.
```
git clone https://github.com/microsoft/vcpkg
```
4. Time to configure the compilation. If you are in Windows, and you use Visual Studio 2022. Just open the repo. If you are in ubuntu, go to the repository folder, open a terminal and type:
```
cmake . --preset=Ubuntu -DCMAKE_TOOLCHAIN_FILE=/home/bardo91/programming/vcpkg/scripts/buildsystems/vcpkg.cmake --overlay-ports="../mico/vcpkg/ports"
```
This last step might take some time.... In my case it took 3 hours to compile PCL and Qt5. Be patient.

And that's it, you are ready to compile. In MSVC you can select the project you want to compile. In ubuntu, go to the out/build/Ubuntu folder and type make to compile all!

BTW: If your computer get stuck compiling the dependencies with VPKG (Which happens typically compiling PCL or QT5), I recomend you to limit the number of threads for VCPKG. To do so, befor calling `cmake . --preset....`, type `export VCPKG_MAX_CONCURRENCY=4` in your screen (o whatever number of threads you want). It happens because compiling and linking those libraries consumes a lot of RAM, and if you have many threads compiling/linking at the same time the PC might get run out of memory, and the OS get stuck.



## FAQ
