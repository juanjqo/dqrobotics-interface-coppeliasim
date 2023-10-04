# cpp-interface-coppelia



### Install zmq 

```shell
git clone https://github.com/zeromq/libzmq
mkdir cmake-build && cd cmake-build
cmake .. && make -j 4
sudo make test && sudo make install && sudo ldconfig
```


### Install cppzmq

```shell
git clone https://github.com/zeromq/cppzmq.git
cd cppzmq
mkdir build
cd build
cmake ..
sudo make -j4 install
```
