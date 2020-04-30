cmake_minimum_required( VERSION 3.6 )

# Language
enable_language( CXX )

# Compiler Settings
set( CMAKE_CXX_STANDARD 17 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )
set( CMAKE_CXX_EXTENSIONS OFF )

# Project
project( camera LANGUAGES CXX )
add_executable( camera util.hpp util.cpp main.cpp )

# (Option) Start-Up Project for Visual Studio
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "camera" )

# Find Package
find_package( CUBEMOS_CORE REQUIRED )
find_package( CUBEMOS_SKELETON_TRACKING REQUIRED )
find_package( OpenCV REQUIRED )

if( CUBEMOS_CORE_FOUND AND CUBEMOS_SKELETON_TRACKING_FOUND AND OpenCV_FOUND )
  target_link_libraries( camera ${CUBEMOS_CORE_LIBS} )
  target_link_libraries( camera ${CUBEMOS_SKELETON_TRACKING_LIBS} )
  target_link_libraries( camera ${OpenCV_LIBS} )
endif()