cmake_minimum_required( VERSION 3.6 )

# Language
enable_language( CXX )

# Compiler Settings
set( CMAKE_CXX_STANDARD 17 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )
set( CMAKE_CXX_EXTENSIONS OFF )

# Project
project( azurekinect LANGUAGES CXX )
add_executable( azurekinect util.hpp util.cpp kinect.hpp kinect.cpp main.cpp )

# (Option) Start-Up Project for Visual Studio
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "azurekinect" )

# Find Package
find_package( CUBEMOS_SKELETON_TRACKING REQUIRED )
find_package( k4a REQUIRED )
find_package( OpenCV REQUIRED )

if( CUBEMOS_SKELETON_TRACKING_FOUND AND k4a_FOUND AND OpenCV_FOUND )
  target_link_libraries( azurekinect cubemos_skeleton_tracking )
  target_link_libraries( azurekinect k4a::k4a )
  target_link_libraries( azurekinect ${OpenCV_LIBS} )
endif()
