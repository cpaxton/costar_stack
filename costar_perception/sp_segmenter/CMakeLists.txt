cmake_minimum_required(VERSION 2.8.3)
project(sp_segmenter C CXX)

SET(CMAKE_BUILD_TYPE "Release")

OPTION(BUILD_ROS_BINDING "Build ROS Binding" ON)
OPTION(BUILD_USE_OBJRECRANSAC "Use ObjRecRANSAC" ON)
OPTION(BUILD_PYTHON_BINDING "Build Python Binding" OFF)
OPTION(BUILD_ENABLE_TRACKING "Build Enable Tracking Keypoints (EXPERIMENTAL)" OFF)

OPTION(BUILD_SCENE_PARSING_SUPPORT
  "Build with Scene Parsing Hypothesis Message (EXPERIMENTAL)" ON)

# Enable C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support, or our tests failed to detect it correctly. Please use a different C++ compiler or report this problem to the developers.")
endif()

if (UNIX)
  message(STATUS "Setting GCC flags")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fexceptions -g -Wall")
else()
  message(STATUS "Setting MSVC flags")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc")
endif()
message(STATUS "** CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")

# CORE LIBRARIES NEEDED FOR SP_SEGMENTER
find_package(PCL REQUIRED) 
find_package(OpenCV 2 REQUIRED) 
find_package(OpenMP )
find_package(Boost)
find_package(VTK)

IF(NOT VTK_FOUND)
  message(FATAL_ERROR "VTK not found. VTK is not installed or cmake can not find it. Install VTK first and then try again.
  If VTK is installed but cmake can not find it, set the VTK_DIR entry to the folder which contains the file VTKConfig.cmake")
ENDIF(NOT VTK_FOUND)

IF(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  add_definitions(-DUSE_OPENMP)
  message (STATUS "Found OpenMP")
ENDIF(OPENMP_FOUND)

# COSTAR specific application. Leave this off all time if you are not working at costar
OPTION(TURN_THIS_OFF "TURN_THIS_OFF" ON)
if (TURN_THIS_OFF)
  message(STATUS "Warning, only turn this on when you are a part of CoSTAR team. You will need to have costar_objrec_msgs from costar_stack package in github to compile this successfully")
  ADD_DEFINITIONS( -DCOSTAR)
endif()

# Build ROS BINDING
IF (BUILD_ROS_BINDING)
  find_package(catkin REQUIRED COMPONENTS sensor_msgs std_msgs message_generation pcl_conversions geometry_msgs tf tf_conversions costar_objrec_msgs QUIET)

  ADD_DEFINITIONS( -DBUILD_ROS_BINDING)

  add_service_files(
    FILES SPSegmenterServer.srv SegmentInGripper.srv
  )

  generate_messages( DEPENDENCIES std_msgs )

  if (APPLE)
    catkin_package(
      CATKIN_DEPENDS sensor_msgs geometry_msgs message_runtime
      # DEPENDS PCL OpenCV ObjRecRANSAC
    )
  else()
    catkin_package(
      CATKIN_DEPENDS sensor_msgs geometry_msgs message_runtime
      # DEPENDS PCL OpenCV OpenMP
    )
  endif()

  if(BUILD_SCENE_PARSING_SUPPORT)
    ADD_DEFINITIONS( -DSCENE_PARSING)
  endif()

ENDIF()


IF (BUILD_ROS_BINDING)
  include_directories(include utility ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS} 
  )
  link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS} 
    ${catkin_LIBRARY_DIRS} 
  ) 
ELSE()
  include_directories(include utility)
  include_directories(${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
ENDIF()

add_definitions(${PCL_DEFINITIONS} ${OpenCV_DEFINITIONS}) 

add_library(Utility
  include/sp_segmenter/utility/typedef.h include/sp_segmenter/utility/utility.h utility/utility.cpp
  include/sp_segmenter/utility/mcqd.h utility/mcqd.cpp include/sp_segmenter/seg.h src/seg.cpp)
add_library(linear utility/liblinear/linear.h utility/liblinear/tron.h 
            utility/liblinear/linear.cpp utility/liblinear/predict.c utility/liblinear/tron.cpp 
            utility/liblinear/blas/blas.h utility/liblinear/blas/blasp.h utility/liblinear/blas/daxpy.c 
            utility/liblinear/blas/ddot.c utility/liblinear/blas/dnrm2.c utility/liblinear/blas/dscal.c)
add_library(PoolLib include/sp_segmenter/features.h src/features.cpp src/HierFea.cpp src/Int_Imager.cpp src/Pooler_L0.cpp src/sp.cpp)
add_library(DataParser include/sp_segmenter/UWDataParser.h include/sp_segmenter/BBDataParser.h include/sp_segmenter/JHUDataParser.h src/UWDataParser.cpp src/BBDataParser.cpp src/JHUDataParser.cpp) 

target_link_libraries(Utility linear ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${VTK_LIBS} )
target_link_libraries(PoolLib Utility linear ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${VTK_LIBS} )
target_link_libraries(DataParser Utility ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${VTK_LIBS} )

add_library(SpCompact src/sp_compact.cpp)
target_link_libraries(SpCompact PoolLib Utility linear ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${VTK_LIBS} )

add_library(SemanticSegmentation src/semantic_segmentation.cpp src/table_segmenter.cpp src/spatial_pose.cpp src/common.cpp)


# Enable OBJRECRANSAC
IF (BUILD_USE_OBJRECRANSAC)
  find_library(ObjRecRANSAC_LIBRARY ObjRecRANSAC)
  message(STATUS "ObjRecRANSAC enabled")
  ADD_DEFINITIONS( -DUSE_OBJRECRANSAC)
  add_library(GreedyObjRecRANSAC include/sp_segmenter/greedyObjRansac.h src/greedyObjRansac.cpp) 
  include_directories(${ObjRecRANSAC_INCLUDE_DIRS})
  target_link_libraries(GreedyObjRecRANSAC Utility ${ObjRecRANSAC_LIBRARY})

  if(NOT EXISTS OBJREC_USE_CUDA)
    FIND_PACKAGE(CUDA)
    if(CUDA_FOUND)
      set(OBJREC_USE_CUDA TRUE)
    endif()
  endif()

  IF(OBJREC_USE_CUDA)
    set(OBJREC_CUDA_DEVICE_ID 1 CACHE STRING "Which GPU device to use when running")
    message(STATUS "CUDA Found and enabled with OBJREC_USE_CUDA=TRUE, Using CUDA GPU Device # " ${OBJREC_CUDA_DEVICE_ID})
    add_definitions(-DUSE_CUDA)
    add_definitions(-DCUDA_DEVICE_ID=${OBJREC_CUDA_DEVICE_ID})

    set(CUDA_TOOLKIT_ROOT_DIR $ENV{CUDA_TOOLKIT_ROOT_DIR})
    FIND_PACKAGE(CUDA REQUIRED)

    add_definitions(-DOBJ_REC_RANSAC_PROFILE)

    #add_definitions(-DOBJ_REC_RANSAC_VERBOSE)
    #add_definitions(-DOBJ_REC_RANSAC_VERBOSE_1)
    #add_definitions(-DOBJ_REC_RANSAC_VERBOSE_2)
  ENDIF()

  target_link_libraries(SemanticSegmentation Utility DataParser PoolLib GreedyObjRecRANSAC
                ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${VTK_LIBS} ${Boost_LIBRARIES})
ELSE()
  message(STATUS "ObjRecRANSAC disabled")
  target_link_libraries(SemanticSegmentation Utility DataParser PoolLib
                ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${VTK_LIBS} ${Boost_LIBRARIES})
ENDIF()



# Sample code for training and doing segmentation
add_executable(sp_compact src/main_sp_compact.cpp)
add_executable(sample_semantic_segmenter src/main_sample_semantic_segmentation.cpp)
target_link_libraries(sample_semantic_segmenter SemanticSegmentation)

IF (BUILD_ROS_BINDING)

  message(STATUS "ROS Binding enabled")
  add_library(RosSemanticSegmentation
    include/sp_segmenter/ros_semantic_segmentation.h
    src/ros_semantic_segmentation.cpp)

  IF (BUILD_ENABLE_TRACKING)
    ADD_DEFINITIONS( -DUSE_TRACKING)
    add_library(Tracking src/tracker.cpp src/klttracker.cpp src/PnPUtil.cpp)
    target_link_libraries(Tracking ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
    target_link_libraries(RosSemanticSegmentation ${catkin_LIBRARIES} Tracking SemanticSegmentation Utility DataParser PoolLib
                ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${VTK_LIBS})
  ELSE()
    target_link_libraries(RosSemanticSegmentation ${catkin_LIBRARIES} SemanticSegmentation Utility DataParser PoolLib
                ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${VTK_LIBS})
  ENDIF()

  add_executable(SPSegmenterServer src/sp_segmenter_server.cpp)
  target_link_libraries(SPSegmenterServer RosSemanticSegmentation) 

  target_link_libraries(sp_compact SpCompact ${catkin_LIBRARIES})
ELSE()
  message(STATUS "ROS Binding disabled")
  target_link_libraries(sp_compact SpCompact)
ENDIF()

IF (BUILD_PYTHON_BINDING)
  message(STATUS " Python Binding enabled. Library 'SpCompactPy' will be built which can imported to python")
  message(STATUS " See python_binding/sample_training.py for an example of how to use it.")
  find_package(PythonInterp)
  find_package(PythonLibs)
  if(APPLE AND ${PYTHON_VERSION_MAJOR} EQUAL 3)
      find_package(Boost COMPONENTS python3)
  else()
      find_package(Boost COMPONENTS python)
  endif()

  INCLUDE_DIRECTORIES(python_binding ${PYTHON_INCLUDE_DIRS})
  LINK_LIBRARIES(${PYTHON_LIBRARIES})

  add_library(SpCompactPy SHARED python_binding/py_sp_compact.cpp)
  add_library(SemanticSegmentationPy SHARED python_binding/py_semantic_segmentation.cpp)

  set_target_properties( SpCompactPy SemanticSegmentationPy
    PROPERTIES
    PREFIX ""
  )

  IF (BUILD_ROS_BINDING)
    # Put the library into catkin's pythonpath
    set_target_properties ( SpCompactPy SemanticSegmentationPy
      PROPERTIES
      LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
    )
  ENDIF()

  IF (APPLE)
    set_target_properties( SpCompactPy SemanticSegmentationPy 
      PROPERTIES SUFFIX ".so" )
  ENDIF()

  target_link_libraries(SpCompactPy SpCompact ${Boost_LIBRARIES})
  target_link_libraries(SemanticSegmentationPy SemanticSegmentation ${Boost_LIBRARIES})

ELSE()
  message(STATUS "Python Binding disabled.")
ENDIF()

# add_executable(TestSymmetricOrientationRealignment src/TestSymmetricOrientationRealignment.cpp)
# target_link_libraries(TestSymmetricOrientationRealignment semanticSegmentation Utility DataParser PoolLib ${Boost_LIBRARIES} )
