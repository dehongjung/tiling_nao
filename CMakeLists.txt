cmake_minimum_required(VERSION 2.8)
project(project)
find_package(qibuild)


qi_create_bin(project project.cpp) 
qi_use_lib(project OPENCV2_CORE OPENCV2_HIGHGUI OPENCV2_FEATURES2D OPENCV2_IMGPROC OPENCV2_CALIB3D ALCOMMON)
