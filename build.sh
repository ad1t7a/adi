#!/bin/bash

set -euo pipefail

case "${1:-}" in
  ("prerequisites")
    #install package using brew
    #brew install automake zmq bazel boost glfw3 openssl@1.1

    # install meshcat
    #pip install meshcat

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

    # physics engine
    rm -rf bullet3/
    git clone https://github.com/bulletphysics/bullet3.git
    cd bullet3/
    ./build_cmake_pybullet_double.sh
    cd ../

    # ur5
    cd urdriver/
    mkdir -p build/
    cd build/
    cmake ../
    make -j8
    cd ../../

    #realsense camera
    rm -rf rm -rf librealsense/
    git clone https://github.com/IntelRealSense/librealsense
    cd librealsense/
    mkdir build
    cd build/
    cmake ../ -DBUILD_PYTHON_BINDINGS=bool:true -DOPENSSL_ROOT_DIR='/usr/local/Cellar/openssl@1.1/1.1.1k/'
    make -j8
    SCRIPTPATH=$(dirname "$SCRIPT")
    echo 'export PYTHONPATH=$PYTHONPATH:$SCRIPTPATH/wrappers/python' >> ~/.zshrc
  
    #realsense camera
    rm -rf rm -rf octomap/
    git clone https://github.com/OctoMap/octomap.git
    cd octomap/
    mkdir build/
    cd build/
    cmake ../
    make

    ;;
  ("obstaclefreeregion")
    bazel build //apps/obstaclefreeregion --cxxopt='-std=c++17'
    ;;
  ("armcontrol")
    bazel build //apps/armcontrol --cxxopt='-std=c++17'
    ;;
  ("visualizer")
    bazel build //apps/visualizer --cxxopt='-std=c++17'
    ;;
  ("clear")
    bazel clean --expunge
    ;;
  (*)
    echo "Usage: $0 <package>" 1>&2
    echo "where <package> is one of the following:" 1>&2
    echo "  prerequisites" 1>&2
    echo "  obstaclefreeregion" 1>&2
    echo "  armcontrol" 1>&2
    echo "  visualizer" 1>&2
    echo "  clear" 1>&2
    exit 1
    ;;
esac
