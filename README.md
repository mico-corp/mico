# MICO 

All MICO-plugins in one repository using git submodules.

## Build 

```
git clone --recurse-submodules -j6 https://github.com/mico-corp/mico
cd mico; 
git submodule update --remote --merge;
mkdir build ; cd build;
cmake .. && make -j{nproc}
```