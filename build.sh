#!/bin/bash

set -euo pipefail

case "${1:-}" in
  ("prerequisites")
    #install package using brew
    brew install automake zmq bazel 
    brew install boost
    
    # install meshcat
    pip install meshcat

   # build json
    cd third_party/
    rm -rf json/
    git clone https://github.com/nlohmann/json.git
    cd json/
    git checkout v3.9.1
    mkdir build/
    cd build/
    cmake ..
    make
    cd ../../
  
   # build crossguid
    rm -rf crossguid/
    git clone https://github.com/graeme-hill/crossguid.git
    cd crossguid/
    git checkout v0.2.2
    mkdir build/
    cd build/
    cmake ..
    make
    cd ../../
  
    # build rbdl
    cd rbdl/
    rm -rf build/
    mkdir build/
    cd build/
    cmake ..
    make
    cp -r ../include/rbdl/* include/rbdl/
    cp -r ../addons/urdfreader/*.h addons/urdfreader/
    cd ../../

    # cppzmq
    rm -rf cppzmq/
    git clone https://github.com/zeromq/cppzmq.git
    cd cppzmq/
    git checkout v4.7.1
    cd ../

    # ompl
    rm -rf ompl/
    git clone https://github.com/ompl/ompl.git
    cd ompl/
    git checkout 1.5.2
    mkdir -p build/Release
    cd build/Release
    cmake ../..
    cd ../

    # msgpack -c
    rm -rf msgpack-c/
    git clone https://github.com/msgpack/msgpack-c.git
    cd msgpack-c/
    git checkout cpp-3.3.0
    cd ../
    ;;
  ("obstaclefreeregion")
    bazel build //apps/obstaclefreeregion --cxxopt='-std=c++17'
    ;;
  ("visualizer")
    bazel build //apps/visualizer
    ;;
  ("clear")
    bazel clean --expunge
    ;;
  (*)
    echo "Usage: $0 <package>" 1>&2
    echo "where <package> is one of the following:" 1>&2
    echo "  prerequisites" 1>&2
    echo "  obstaclefreeregion" 1>&2
    echo "  visualizer" 1>&2
    echo "  clear" 1>&2
    exit 1
    ;;
esac
