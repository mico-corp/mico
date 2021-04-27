# fastcom_mplugin
Fastcom publishers and subscribers MICO plugin

[![Releases](https://img.shields.io/github/release/mico-corp/fastcom_mplugin.svg)](https://github.com/mico-corp/fastcom_mplugin/releases)  [![Issues](https://img.shields.io/github/issues/mico-corp/fastcom_mplugin.svg)](https://github.com/mico-corp/fastcom_mplugin/issues)

[![Build Status](https://travis-ci.com/mico-corp/fastcom_mplugin.svg?branch=master)](https://travis-ci.com/mico-corp/fastcom_mplugin)

## Dependencies

### FASTCOM

Installation from source code: 

```
git clone https://github.com/Bardo91/fastcom.git
cd fastcom
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