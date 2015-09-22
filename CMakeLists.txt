cmake_minimum_required(VERSION 2.8.3)
SET(CMAKE_BUILD_TYPE "Release")

add_definitions(-g -std=c++0x -pthread)
find_package(catkin REQUIRED COMPONENTS sensor_msgs pcl_conversions resource_retriever pluginlib nodelet)

project(sp_segmenter)

find_package(PCL REQUIRED) 
find_package(OpenCV REQUIRED) 
find_package(OpenMP REQUIRED)
find_package(Boost COMPONENTS Chrono)

IF(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
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

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS sensor_msgs 
  DEPENDS PCL OpenCV OpenMP 
)

include_directories(${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS} ${OpenCV_DEFINITIONS}) 

#library
add_library(Utility utility/typedef.h utility/utility.h utility/utility.cpp utility/mcqd.h utility/mcqd.cpp) 
target_link_libraries(Utility liblinear ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_library(liblinear utility/liblinear/linear.h utility/liblinear/tron.h 
            utility/liblinear/linear.cpp utility/liblinear/predict.cpp utility/liblinear/tron.cpp 
            utility/liblinear/blas.h utility/liblinear/blasp.h utility/liblinear/daxpy.cpp 
            utility/liblinear/ddot.cpp utility/liblinear/dnrm2.cpp utility/liblinear/dscal.cpp)

add_library(PoolLib include/features.h src/features.cpp src/HierFea.cpp src/Int_Imager.cpp src/Pooler_L0.cpp src/sp.cpp)
target_link_libraries(PoolLib Utility liblinear ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_library(DataParser include/UWDataParser.h include/BBDataParser.h include/JHUDataParser.h src/UWDataParser.cpp src/BBDataParser.cpp src/JHUDataParser.cpp) 
target_link_libraries(DataParser Utility ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(spTraining src/main_sp_training.cpp) 
#target_link_libraries(spTraining Utility DataParser PoolLib ${Boost_LIBRARIES} 
#              ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(spSVM src/main_sp_svm.cpp) 
target_link_libraries(spSVM Utility DataParser PoolLib ${Boost_LIBRARIES} 
              ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(SimpleSegmenter src/main_simple_seg.cpp) 
target_link_libraries(SimpleSegmenter Utility DataParser PoolLib ${Boost_LIBRARIES} 
              ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

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

#add_executable(DictLearning src/main_dict.cpp include/WKmeans.h src/WKmeans.cpp ) 
#target_link_libraries(DictLearning DataParser Utility PoolLib ${Boost_LIBRARIES} 
#              ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(uw_test src/main_uw_test.cpp) 
#target_link_libraries(uw_test Utility DataParser PoolLib ${Boost_LIBRARIES} 
#              ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(hard_mining src/main_hardneg.cpp) 
#target_link_libraries(hard_mining Utility DataParser PoolLib ${Boost_LIBRARIES} 
#              ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})


