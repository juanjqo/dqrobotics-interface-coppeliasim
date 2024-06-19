#!/bin/sh
mkdir build
cd build/
cmake ..
make -j10
sudo make install
