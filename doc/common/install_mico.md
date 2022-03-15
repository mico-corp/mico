How to install MICO {#install_mico}
=====================

# Installation Ubuntu


# Installation Windows

## Package installer

The easiest way to install MICO is to use the automatic package installer. It can be download directly from [github releases](https://github.com/mico-corp/mico/releases).

Download the zip file and follow the instructions. 

## Build from sources

Building it from sources takes more time, but most of the process has been automated using cmake and VCPKG. 

### 1. Installing Visual Studio
We recommend you to install Visual Studio Community version. It can be downloaded from the official Microsoft webpage (https://visualstudio.microsoft.com/es/vs/community/). There is not current support for MinGw or CygWin, but if you want to use them share the process with us and we will integrate that option, we appreciate any contribution.

### 2. Preparing VCPKG
VCPKG is a package/library manager that makes programmers' life easier! It works in Window, Linux and Mac and contains a bunch of ports for almost all the common libraries used by developers daily. 
In order to use it, you just need to clone it and create a new variable in the environment pointing to its root folder.

```
git clone https://github.com/microsoft/vcpkg
cd vcpkg
./bootstrap-vcpkg.bat
```

Do not forget to create the environment variable:
\image html common/sample_environment_variable.jpg width=640px


### 3. Compile MICO
If you are in Windows and plan to use Visual Studio, you just need to clone the repository and open the folder in it. VS will do the rest (assuming you are running VS2022)
```
git clone https://github.com/mico-corp/mico/
```

At this point, go to the kitchen an make a te or pop-corn. The process takes an average of two hours to download and compile all the dependencies in Windows. Once the process has finished we are ready to install everything



