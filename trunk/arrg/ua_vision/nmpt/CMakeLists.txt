cmake_minimum_required(VERSION 2.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)

rosbuild_init ()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rosbuild_add_library(nmpt
    src/nmpt/BlockTimer.cpp 
    src/nmpt/ConvolutionalLogisticPolicy.cpp 
    src/nmpt/CVPRTrainModels.cpp 
    src/nmpt/FastSaliency.cpp 
    src/nmpt/ImageDataSet.cpp 
    src/nmpt/ImagePatchPyramid.cpp 
    src/nmpt/MIPOMDP.cpp
    src/nmpt/MultinomialObservationModel.cpp 
    src/nmpt/ObjectDetector.cpp 
    src/nmpt/OpenCVBoxFilter.cpp 
    src/nmpt/OpenCVHaarDetector.cpp 
    src/nmpt/OpenLoopPolicies.cpp 
)

rosbuild_add_executable(CVPRTestSpeed src/nmpt/CVPRTestSpeed.cpp)
target_link_libraries(CVPRTestSpeed nmpt)

rosbuild_add_executable(FastSUN src/nmpt/FastSUN.cpp)
target_link_libraries(FastSUN nmpt)

rosbuild_add_executable(FastSUNImage src/nmpt/FastSUNImage.cpp)
target_link_libraries(FastSUNImage nmpt)

rosbuild_add_executable(FoveatedFaceTracker src/nmpt/FoveatedFaceTracker.cpp)
target_link_libraries(FoveatedFaceTracker nmpt)

rosbuild_add_executable(FoveatedFaceTrackerImage src/nmpt/FoveatedFaceTrackerImage.cpp)
target_link_libraries(FoveatedFaceTrackerImage nmpt)

rosbuild_add_executable(SimpleFaceTracker src/nmpt/SimpleFaceTracker.cpp)
target_link_libraries(SimpleFaceTracker nmpt)

rosbuild_add_executable(SimpleSaliencyExample src/nmpt/SimpleSaliencyExample.cpp)
target_link_libraries(SimpleSaliencyExample nmpt)

rosbuild_add_executable(TrainNarrowFOVModel src/nmpt/TrainNarrowFOVModel.cpp)
target_link_libraries(TrainNarrowFOVModel nmpt)

