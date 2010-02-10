cmake_minimum_required(VERSION 2.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rosbuild_init ()

rosbuild_add_library(nmpt src/BlockTimer.cpp 
    src/ConvolutionalLogisticPolicy.cpp 
    #src/CVPRTestSpeed.cpp 
    src/CVPRTrainModels.cpp 
    src/FastSaliency.cpp 
    #src/FastSUN.cpp 
    #src/FastSUNImage.cpp 
    #src/FoveatedFaceTracker.cpp 
    #src/FoveatedFaceTrackerImage.cpp 
    src/ImageDataSet.cpp 
    #src/ImagePatchPyramid.cpp 
    #src/MIPOMDP.cpp 
    src/MultinomialObservationModel.cpp 
    src/ObjectDetector.cpp 
    src/OpenCVBoxFilter.cpp 
    src/OpenCVHaarDetector.cpp 
    src/OpenLoopPolicies.cpp 
    #src/SimpleFaceTracker.cpp 
    #src/SimpleSaliencyExample.cpp 
    #src/TrainNarrowFOVModel.cpp
)


