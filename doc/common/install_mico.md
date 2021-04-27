How to install MICO {#install_mico}
=====================

# Installation Ubuntu


# Installation Windows

## Package installer

The easiest way to install MICO is to use the automatic package installer. It can be download directly from [github releases](https://github.com/mico-corp/mico/releases).

Download the zip file and follow the instructions. 

## Build from sources

Building it from sources takes more time, but most of the process has been automated through a pair of cmake calls. 

### 1. Installing Visual Studio
We recommend you to install Visual Studio Community version. It can be downloaded from the official Microsoft webpage (https://visualstudio.microsoft.com/es/vs/community/). There is not current support for MinGw or CygWin, but if you want to use them share the process with us and we will integrate that option, we appreciate any contribution.

### 2. Installing Qt5
Qt5 needs to be installed manually. You can get it from the official QT webpage: https://www.qt.io/download. By now, MICO only has compatibility with Qt5 (at least versión 5.12.0).

### 3. Installing MICO
The rest of dependencies are automatically downloaded and compiled with a cmake script. At first, clone MICO repository in any folder on your computer. Get into the folder and run cmake. To run the installation script set ON the variable "MICO_INSTALL_ALL_DEPENDENCIES"

```
git clone https://github.com/mico-corp/mico/
mkdir build
cd build
cmake .. -DMICO_INSTALL_ALL_DEPENDENCIES=ON
```

At this point, go to the kitchen an make a te or pop-corn. The process takes an average of two hours to download and compile all the dependencies in Windows. Once the process has finished we are ready to install everything

### Installing flow_kids and MICO-plugins.
To install MICO visual interface and standard plugins just run the following lines of code in the build folder

```
cmake --build . --config Release -j
cmake --build . --config Release -j --target flow_install
cmake --build . --config Release -j --target INSTALL
```

