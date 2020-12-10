# MICO 

All MICO-plugins in one repository using git submodules.

## Dependencies

```
sudo apt-get install qt5-default libeigen3-dev libboost-all-dev libpcl-dev
```

## Build 

```
git clone --recurse-submodules -j6 https://github.com/mico-corp/mico
cd mico; 
git submodule update --remote --merge;
mkdir build ; cd build;
cmake .. && make -j{nproc}
```
