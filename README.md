Skeleton Tracking SDK by Cubemos Sample Program
===============================================

This repository is sample program of Skeleton Tracking SDK by Cubemos.  
This sample program works on cross-platform (Windows, Linux).  

Environment
-----------
* Visual Studio 2017/2019 / GCC 7.4 / Clang 6.0 (or later) 
* Skeleton Tracking SDK by Cubemos v2.3.0 (or later)
* OpenCV 3.4.2 (or later)
* CMake 3.15.4 (latest release is preferred)

### 2D Skeleton Tracking
If you want to use Skeleton Tracking in 2D, You don't need any additional sensor or libraries.  

* Web Camera
* Image File (e.g. JPEG, PNG, TIFF)

### 3D Skeleton Tracking
If you want to use Skeleton Tracking in 3D, You need these sensors and libraries.  

* RealSense and RealSense SDK v2.x
* Azure Kinect and Azure Kinect Sensor SDK v1.4.0 (or later)

Fix CMake Files of Skeleton Tracking SDK by Cubemos
---------------------------------------------------
In current version, CMake files for Skeleton Tracking SDK by Cubemos has a few mistakes.  
Please fix CMake files according with [this document](https://gist.github.com/UnaNancyOwen/6e07e2d6d459b2cadfa7a17d867771ad#file-fix-md).  

License
-------
Copyright &copy; 2020 Tsukasa SUGIURA  
Distributed under the [MIT License](http://www.opensource.org/licenses/mit-license.php "MIT License | Open Source Initiative").

Contact
-------
* Tsukasa Sugiura  
    * <t.sugiura0204@gmail.com>  
    * <http://unanancyowen.com>  

Reference
---------
* Skeleton Tracking SDK for Intel RealSense Depth Cameras | Intel RealSense  
  <https://www.intelrealsense.com/skeleton-tracking/>

* Skeleton Tracking SDK | cubemos  
  <https://www.cubemos.com/skeleton-tracking-sdk/>
