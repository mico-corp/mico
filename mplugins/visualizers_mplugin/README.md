# visualizers_mplugin
Visualizers MICO plugin

[![Releases](https://img.shields.io/github/release/mico-corp/visualizers_mplugin.svg)](https://github.com/mico-corp/visualizers_mplugin/releases)  [![Issues](https://img.shields.io/github/issues/mico-corp/visualizers_mplugin.svg)](https://github.com/mico-corp/visualizers_mplugin/issues)

[![Build Status](https://travis-ci.com/mico-corp/visualizers_mplugin.svg?branch=master)](https://travis-ci.com/mico-corp/visualizers_mplugin)

## Dependencies

### Pangolin

Installation from source code: 

```
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build ; cd build
cmake .. -DCMAKE_BUILD_TYPE=Release ; make -j
sudo make install
```

### FLOW

Installation from source code: 

```
git clone https://github.com/mico-corp/flow.git
cd dvsal
mkdir build ; cd build
cmake .. -DCMAKE_BUILD_TYPE=Release ; make -j
sudo make install
```

### CORE_MPLUGIN

Installation from source code: 

```
git clone https://github.com/mico-corp/core_mplugin.git
cd dvsal
mkdir build ; cd build
cmake .. -DCMAKE_BUILD_TYPE=Release ; make -j
sudo make install
```