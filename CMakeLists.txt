cmake_minimum_required(VERSION 2.8.3)
SET(CMAKE_BUILD_TYPE "Release")
find_package(catkin COMPONENTS sensor_msgs std_msgs message_generation pcl_conversions geometry_msgs tf tf_conversions cv_bridge costar_objrec_msgs QUIET)# greedy_objrec_ransac)
project(sp_segmenter C CXX)



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


find_package(PCL REQUIRED) 
find_package(OpenCV REQUIRED) 
find_package(OpenMP )
find_package(Boost)
find_package(VTK)

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

if (UNIX)
  message(status "Setting GCC flags")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fexceptions -g -Wall")
else()
  message(status "Setting MSVC flags")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc")
endif()
message(status "** CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")

if(catkin_FOUND)
    add_service_files(
      FILES
      SPSegmenterServer.srv
      segmentInGripper.srv
    )

    generate_messages(
      DEPENDENCIES
      std_msgs
    )

  if (APPLE)
    catkin_package(
      INCLUDE_DIRS include utility
      LIBRARIES 
      CATKIN_DEPENDS sensor_msgs geometry_msgs message_runtime #greedy_objrec_ransac
      DEPENDS PCL OpenCV 
    )
  else()
    catkin_package(
      INCLUDE_DIRS include utility
      LIBRARIES 
      CATKIN_DEPENDS sensor_msgs geometry_msgs message_runtime #greedy_objrec_ransac
      DEPENDS PCL OpenCV OpenMP
    )
  endif()
endif()

include_directories(include utility)

find_library(ObjRecRANSAC_LIBRARY ObjRecRANSAC)

include_directories(${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS} ${catkin_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS} ${OpenCV_DEFINITIONS}) 

#library
add_library(Tracking src/tracker.cpp src/klttracker.cpp src/PnPUtil.cpp)
target_link_libraries(Tracking ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_library(Utility include/sp_segmenter/utility/typedef.h include/sp_segmenter/utility/utility.h utility/utility.cpp include/sp_segmenter/utility/mcqd.h utility/mcqd.cpp include/sp_segmenter/seg.h src/seg.cpp include/sp_segmenter/greedyObjRansac.h src/greedyObjRansac.cpp) 
target_link_libraries(Utility linear ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES}   ${ObjRecRANSAC_LIBRARY} ${VTK_LIBS} )

add_library(linear utility/liblinear/linear.h utility/liblinear/tron.h 
            utility/liblinear/linear.cpp utility/liblinear/predict.c utility/liblinear/tron.cpp 
            utility/liblinear/blas/blas.h utility/liblinear/blas/blasp.h utility/liblinear/blas/daxpy.c 
            utility/liblinear/blas/ddot.c utility/liblinear/blas/dnrm2.c utility/liblinear/blas/dscal.c)

add_library(PoolLib include/sp_segmenter/features.h src/features.cpp src/HierFea.cpp src/Int_Imager.cpp src/Pooler_L0.cpp src/sp.cpp)
target_link_libraries(PoolLib Utility linear ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES}   ${ObjRecRANSAC_LIBRARY} ${VTK_LIBS} )

add_library(DataParser include/sp_segmenter/UWDataParser.h include/sp_segmenter/BBDataParser.h include/sp_segmenter/JHUDataParser.h src/UWDataParser.cpp src/BBDataParser.cpp src/JHUDataParser.cpp) 
target_link_libraries(DataParser Utility ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES}   ${ObjRecRANSAC_LIBRARY} ${VTK_LIBS} )


add_library(semanticSegmentation include/sp_segmenter/semanticSegmentation.h src/semanticSegmentation.cpp) 
target_link_libraries(semanticSegmentation Utility DataParser PoolLib Tracking #${Boost_LIBRARIES} 
                  ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${ObjRecRANSAC_LIBRARY}  ${VTK_LIBS})

#add_executable(spTraining src/main_sp_training.cpp) 
#target_link_libraries(spTraining Utility DataParser PoolLib ${Boost_LIBRARIES} 
#              ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(spSVM src/main_sp_svm.cpp) 
#target_link_libraries(spSVM Utility DataParser PoolLib #${Boost_LIBRARIES} 
#              ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${ObjRecRANSAC_LIBRARY}  ${VTK_LIBS})

add_executable(spCompact src/main_sp_compact.cpp)
target_link_libraries(spCompact Utility DataParser PoolLib ${Boost_LIBRARIES}
               ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(SimpleSegmenter src/main_simple_seg.cpp) 
target_link_libraries(SimpleSegmenter Utility DataParser PoolLib #${Boost_LIBRARIES} 
              ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${ObjRecRANSAC_LIBRARY} ${VTK_LIBS})

#if(CATKIN_FOUND)
    add_executable(SPSegmenterNode src/main_ros.cpp) 
    target_link_libraries(SPSegmenterNode Utility DataParser PoolLib Tracking #${Boost_LIBRARIES} 
                  ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${ObjRecRANSAC_LIBRARY}  ${VTK_LIBS})
#endif(CATKIN_FOUND)

add_executable(foreRun include/sp_segmenter/greedyObjRansac.h src/main_foreground.cpp)
target_link_libraries(foreRun Utility PoolLib ${OpenCV_LIBS} ${PCL_LIBRARIES}  ${ObjRecRANSAC_LIBRARY} ${VTK_LIBS})

add_executable(SPSegmenterServer src/sp_segmenter_server.cpp)
target_link_libraries(SPSegmenterServer semanticSegmentation Utility DataParser PoolLib #${Boost_LIBRARIES} 
                  ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${ObjRecRANSAC_LIBRARY}  ${VTK_LIBS})
#add_executable(sift_test src/main_sift_test.cpp) 
#target_link_libraries(sift_test Utility DataParser PoolLib ${Boost_LIBRARIES} 
#              ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(gen_gt_seg src/main_gt_seg.cpp) 
#target_link_libraries(gen_gt_seg Utility DataParser PoolLib ${Boost_LIBRARIES} 
#              ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(triple_test src/main_triple.cpp) 
#target_link_libraries(triple_test Utility DataParser PoolLib ${Boost_LIBRARIES} 
#              ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(semantics src/main_semantics.cpp) 
#target_link_libraries(semantics Utility DataParser PoolLib ${Boost_LIBRARIES} 
#              ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(DictLearning src/main_dict.cpp include/sp_segmenter/WKmeans.h src/WKmeans.cpp ) 
#target_link_libraries(DictLearning DataParser Utility PoolLib ${Boost_LIBRARIES} 
#              ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(uw_test src/main_uw_test.cpp) 
#target_link_libraries(uw_test Utility DataParser PoolLib ${Boost_LIBRARIES} 
#              ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(hard_mining src/main_hardneg.cpp) 
#target_link_libraries(hard_mining Utility DataParser PoolLib ${Boost_LIBRARIES} 
#              ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})


