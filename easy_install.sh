#!/bin/bash
set -e
REPO_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/" && pwd )"
TMP_DIR="/tmp"

install_g2o() {
    echo "Prepare to install g2o"
    sudo apt-get install qt5-qmake qt5-default libqglviewer-dev-qt5 libsuitesparse-dev libcxsparse3 libcholmod3
    if ( ls /usr/local/include | grep g2o);then
        echo "g2o is already installed......"
    else
        wget https://github.com/RainerKuemmerle/g2o/archive/refs/tags/20201223_git.zip
        unzip 20201223_git.zip
        cd g2o-20201223_git
        mkdir build && cd build
        cmake ..
        make -j$(nproc) 
        sudo make install
        echo "g2o installed successfully"   
        cd $REPO_DIR
    fi
    sudo ldconfig 
}

install_osqp_eigen() {
    sudo apt-get install libeigen3-dev
    sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
    echo "eigen3 is installed successfully!"
    echo "Prepare to install osqp"
    if ( ls /usr/local/include | grep osqp);then
        echo "osqp is already installed......"
    else
        git clone --recursive https://github.com/oxfordcontrol/osqp
        cd osqp
        mkdir build && cd build
        cmake -G "Unix Makefiles" ..
        cmake --build .
        sudo cmake --build . --target install
        echo "osqp installed successfully"
        cd $REPO_DIR
    fi
    sudo ldconfig 
    echo "Prepare to install osqp-eigen"
    if ( ls /usr/local/include | grep OsqpEigen);then
        echo "OsqpEigen is already installed......"
    else
        git clone https://github.com/robotology/osqp-eigen.git
        cd osqp-eigen
        mkdir build && cd build
        cmake ../
        make -j$(nproc) 
        sudo make install
        echo "osqp-eigen installed successfully"
        cd $REPO_DIR
    fi
    sudo ldconfig 
}
install_benchmark() {
    echo "Prepare to install google benchmark"
    if ( ls /usr/local/include | grep benchmark);then
        echo "benchmark is already installed......"
    else
    
        git clone https://github.com/google/benchmark.git
        cd benchmark
        git clone https://github.com/google/googletest.git
        mkdir build && cd build
        cmake .. -DCMAKE_BUILD_TYPE=RELEASE
        make -j$(nproc) 
        sudo make install
        echo "benchmark installed successfully"
        cd $REPO_DIR
    fi
    sudo ldconfig 
}
install_glog() {
    sudo apt-get install libgoogle-glog-dev
    echo "glog is installed successfully!"
}
install_gflags() {
    sudo apt-get install libgflags-dev
    echo "gflags is installed successfully!"
}
install_protobuf() {
    sudo apt-get install autoconf automake libtool curl make g++ unzip libffi-dev
    echo "Prepare to install google protobuf"
    if (ls /usr/local/include | grep google);then
        echo "google protobuf is already installed......"
    else
        git clone https://github.com/google/protobuf.git
        cd protobuf
        ./autogen.sh
        ./configure
        make -j$(nproc) 
        sudo make install
        echo "google protobuf installed successfully"
        cd $REPO_DIR
    fi
    sudo ldconfig 
}
install_matplotlibcpp() {
    sudo apt-get install python-matplotlib python-numpy python2.7-dev
}
main() {
    echo ${REPO_DIR}
    # install_benchmark
    # install_glog
    # install_gflags
    
    # install_osqp_eigen
    # install_g2o
}


main