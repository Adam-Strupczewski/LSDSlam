TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

CONFIG += c++11

INCLUDEPATH += \
    /usr/include/eigen3/ \
    /usr/include/suitesparse \
    /usr/include/boost \
    /usr/local/include/opencv \
    /opt/ros/indigo/include/ \
    ../src \
    ../cfg/cpp \
    ../thirdparty/Sophus

LIBS += \
    -lpthread \
    -lboost_system \
    -lboost_thread \
    -lboost_filesystem \
    -lboost_iostreams \
    -L/usr/local/lib \
    -lopencv_core \
    -lopencv_imgproc \
    -lopencv_highgui \
    -lopencv_ml \
    -lopencv_video \
    -lopencv_features2d \
    -lopencv_calib3d \
    -lopencv_objdetect \
    -lopencv_contrib \
    -lopencv_legacy \
    -lopencv_flann \
    -L/opt/ros/indigo/lib \
    -lg2o_core \
    -lg2o_stuff \
    -lg2o_solver_csparse \
    -lg2o_csparse_extension \
    -lg2o_types_sim3 \
    -lg2o_types_sba \
    -lcsparse \
    -lcxsparse

SOURCES += \
    ../src/DataStructures/Frame.cpp \
    ../src/DataStructures/FrameMemory.cpp \
    ../src/DataStructures/FramePoseStruct.cpp \
    ../src/DepthEstimation/DepthMap.cpp \
    ../src/DepthEstimation/DepthMapPixelHypothesis.cpp \
    ../src/GlobalMapping/FabMap.cpp \
    ../src/GlobalMapping/g2oTypeSim3Sophus.cpp \
    ../src/GlobalMapping/KeyFrameGraph.cpp \
    ../src/GlobalMapping/TrackableKeyFrameSearch.cpp \
    ../src/IOWrapper/OpenCV/ImageDisplay_OpenCV.cpp \
    ../src/IOWrapper/ROS/ROSImageStreamThread.cpp \
    ../src/IOWrapper/ROS/ROSOutput3DWrapper.cpp \
    ../src/IOWrapper/Timestamp.cpp \
    ../src/Tracking/least_squares.cpp \
    ../src/Tracking/Relocalizer.cpp \
    ../src/Tracking/SE3Tracker.cpp \
    ../src/Tracking/Sim3Tracker.cpp \
    ../src/Tracking/TrackingReference.cpp \
    ../src/util/globalFuncs.cpp \
    ../src/util/settings.cpp \
    ../src/util/SophusUtil.cpp \
    ../src/util/Undistorter.cpp \
    ../src/LiveSLAMWrapper.cpp \
    ../src/main_live_odometry.cpp \
    ../src/main_on_images.cpp \
    ../src/SlamSystem.cpp

include(deployment.pri)
qtcAddDeployment()

HEADERS += \
    ../src/DataStructures/Frame.h \
    ../src/DataStructures/FrameMemory.h \
    ../src/DataStructures/FramePoseStruct.h \
    ../src/DepthEstimation/DepthMap.h \
    ../src/DepthEstimation/DepthMapPixelHypothesis.h \
    ../src/GlobalMapping/FabMap.h \
    ../src/GlobalMapping/g2oTypeSim3Sophus.h \
    ../src/GlobalMapping/KeyFrameGraph.h \
    ../src/GlobalMapping/TrackableKeyFrameSearch.h \
    ../src/IOWrapper/ROS/ROSImageStreamThread.h \
    ../src/IOWrapper/ROS/ROSOutput3DWrapper.h \
    ../src/IOWrapper/ROS/rosReconfigure.h \
    ../src/IOWrapper/ImageDisplay.h \
    ../src/IOWrapper/InputImageStream.h \
    ../src/IOWrapper/NotifyBuffer.h \
    ../src/IOWrapper/Output3DWrapper.h \
    ../src/IOWrapper/Timestamp.h \
    ../src/IOWrapper/TimestampedObject.h \
    ../src/Tracking/least_squares.h \
    ../src/Tracking/Relocalizer.h \
    ../src/Tracking/SE3Tracker.h \
    ../src/Tracking/Sim3Tracker.h \
    ../src/Tracking/TrackingReference.h \
    ../src/util/EigenCoreInclude.h \
    ../src/util/globalFuncs.h \
    ../src/util/IndexThreadReduce.h \
    ../src/util/settings.h \
    ../src/util/SophusUtil.h \
    ../src/util/Undistorter.h \
    ../src/LiveSLAMWrapper.h \
    ../src/SlamSystem.h \
    ../src/lsd_slam_viewer/keyframeGraphMsg.h \
    ../src/lsd_slam_viewer/keyframeMsg.h

