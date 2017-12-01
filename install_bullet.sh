#!/usr/bin/env bash
git clone https://github.com/bulletphysics/bullet3.git
cd bullet3
mkdir build_
cd build_
cmake .. -DBUILD_SHARED_LIBS=ON -DBUILD_EXTRAS=ON -DINSTALL_EXTRA_LIBS=ON -DUSE_DOUBLE_PRECISION=ON
make -j
sudo make install
