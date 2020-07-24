    sudo apt-get install libgtest-dev protobuf-compiler libflann-dev
    
    
    
### Dependencies :

- aslam
- brisk
- agast
- GLOB
- onpencl + contrib modules
- eigen
- minkindr
- gflags
- gtest
- yalm_cpp
- PCL

#### Install PCL :

get last version on github (released not git get) : https://github.com/PointCloudLibrary/pcl

    mkdir build && cd build/
    cmake -DCMAKE_BUILD_TYPE=Release ..
    make -j16
    sudo make -j16 install