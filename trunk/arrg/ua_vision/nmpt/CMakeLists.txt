cmake_minimum_required(VERSION 2.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)

rosbuild_init ()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(nmpt_CFLAGS_OTHER "-fpermissive")

rosbuild_add_library(nmpt
    src/nmpt/BlockTimer.cpp 
    src/nmpt/BoxFeature.cpp 
    src/nmpt/BoxFeature2.cpp 
    src/nmpt/ConvolutionalLogisticPolicy.cpp 
    src/nmpt/DetectionEvaluator.cpp 
    src/nmpt/DetectionEvaluator2.cpp 
    src/nmpt/EstimateInternalMotion.cpp 
    src/nmpt/FastPatchList.cpp 
    src/nmpt/FastPatchList2.cpp 
    src/nmpt/FastSalience.cpp 
    src/nmpt/Feature.cpp 
    src/nmpt/Feature2.cpp 
    src/nmpt/FeatureRegressor.cpp 
    src/nmpt/FeatureRegressor2.cpp 
    src/nmpt/GentleBoostCascadedClassifier.cpp 
    src/nmpt/GentleBoostClassifier.cpp 
    src/nmpt/GentleBoostClassifier2.cpp 
    src/nmpt/HaarFeature.cpp 
    src/nmpt/HaarFeature2.cpp 
    src/nmpt/ImageDataSet.cpp 
    src/nmpt/ImageDataSet2.cpp 
    src/nmpt/ImageKalmanFilter.cpp 
    src/nmpt/ImagePatch.cpp 
    src/nmpt/ImagePatch2.cpp 
    src/nmpt/ImagePatchPyramid.cpp 
    src/nmpt/InternalMotionModel.cpp 
    src/nmpt/KF.cpp 
    src/nmpt/KFPointTracker.cpp 
    src/nmpt/LQR.cpp 
    src/nmpt/LQRPointTracker.cpp 
    src/nmpt/MatrixKalmanFilter.cpp 
    src/nmpt/MIPOMDP.cpp 
    src/nmpt/MultinomialObservationModel.cpp 
    src/nmpt/MultiObjectTrackingState.cpp 
    src/nmpt/NMPTUtils.cpp 
    src/nmpt/ObjectDetector.cpp 
    src/nmpt/OpenCV2BoxFilter.cpp 
    src/nmpt/OpenCVHaarDetector.cpp 
    src/nmpt/OpenLoopPolicies.cpp 
    src/nmpt/PatchDataset.cpp 
    src/nmpt/PatchDataset2.cpp 
    src/nmpt/PatchList.cpp 
    src/nmpt/PatchList2.cpp 
    src/nmpt/StructTypes.cpp 
)

rosbuild_add_executable(ChooseSaccadeTargets src/nmpt/ChooseSaccadeTargets.cpp)
target_link_libraries(ChooseSaccadeTargets nmpt)

rosbuild_add_executable(CVPRTestSpeed src/nmpt/CVPRTestSpeed.cpp)
target_link_libraries(CVPRTestSpeed nmpt)

rosbuild_add_executable(CVPRTrainModels src/nmpt/CVPRTrainModels.cpp)
target_link_libraries(CVPRTrainModels nmpt)

rosbuild_add_executable(EstimateInternalMotion src/nmpt/EstimateInternalMotion.cpp)
target_link_libraries(EstimateInternalMotion nmpt)

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

rosbuild_add_executable(SimpleSalienceExample src/nmpt/SimpleSalienceExample.cpp)
target_link_libraries(SimpleSalienceExample nmpt)

rosbuild_add_executable(TrainNarrowFOVModel src/nmpt/TrainNarrowFOVModel.cpp)
target_link_libraries(TrainNarrowFOVModel nmpt)

