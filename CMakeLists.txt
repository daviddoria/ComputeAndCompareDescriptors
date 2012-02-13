cmake_minimum_required(VERSION 2.6)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

PROJECT(ComputeAndCompareDescriptors)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET") # Without this can get this error: /include/eigen3/Eigen/src/Core/DenseStorage.h:69: Eigen::internal::plain_array<T, Size, MatrixOrArrayOptions, 16>::plain_array() [with T = float, int Size = 4, int MatrixOrArrayOptions = 0]: Assertion `(reinterpret_cast<size_t>(array) & 0xf) == 0 && "this assertion is explained here: " "http://eigen.tuxfamily.org/dox-devel/TopicUnalignedArrayAssert.html" " **** READ THIS WEB PAGE !!! ****"' failed.

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

FIND_PACKAGE(PCL 1.5 REQUIRED)
INCLUDE(${VTK_USE_FILE})
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

QT4_WRAP_UI(UISrcs ComputeAndCompareDescriptorsWidget.ui)
QT4_WRAP_CPP(MOCSrcs ComputeAndCompareDescriptorsWidget.h)

ADD_EXECUTABLE(ComputeAndCompareDescriptors
ComputeAndCompareDescriptors.cpp
ComputeAndCompareDescriptorsWidget.cpp


ComputeClusteredViewpointFeatureHistograms.cpp
ComputeViewpointFeatureHistograms.cpp
ComputeMaskedViewpointFeatureHistograms.cpp
ComputePointFeatureHistograms.cpp
ComputeFastPointFeatureHistograms.cpp
ComputePFHRGB.cpp

ComputeAndCompareDescriptorsWidget.cpp

Helpers.cpp
PointSelectionStyle3D.cpp
${UISrcs} ${MOCSrcs} ${ResourceSrcs})

TARGET_LINK_LIBRARIES(ComputeAndCompareDescriptors QVTK ${VTK_LIBRARIES}
${ITK_LIBRARIES}
${PCL_LIBRARIES}
)
