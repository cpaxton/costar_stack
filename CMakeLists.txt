cmake_minimum_required(VERSION 2.8.3)
project(sequential_scene_parsing)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(VTK REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED)
find_package(Bullet REQUIRED)
find_package(GLUT REQUIRED)

FIND_PACKAGE(OpenGL)
IF (OPENGL_FOUND)
	MESSAGE("OPENGL FOUND")
	MESSAGE(${OPENGL_LIBRARIES})
ELSE (OPENGL_FOUND)
	MESSAGE("OPENGL NOT FOUND")
	SET(OPENGL_gl_LIBRARY opengl32)
	SET(OPENGL_glu_LIBRARY glu32)
ENDIF (OPENGL_FOUND)

find_library(ObjRecRANSAC_LIBRARY ObjRecRANSAC)

find_package(catkin REQUIRED COMPONENTS
  VTK
  PCL
  tf
  pcl_conversions
  sensor_msgs
  costar_objrec_msgs
  objrec_hypothesis_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sequential_scene_parsing
#  CATKIN_DEPENDS message_generation message_runtime pcl_conversions pcl_ros sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
add_definitions(-DBT_USE_DOUBLE_PRECISION=ON)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  tool
  ${catkin_INCLUDE_DIRS}
  ${BULLET_INCLUDE_DIRS}
  ${GLUT_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
)

## Declare a C++ library
LINK_DIRECTORIES(${BULLET_LIBRARY_DIRS} )

set(OpenGLSupport
	include/debugdrawer/GLDebugFont.cpp
	include/debugdrawer/GLDebugFont.h
	include/debugdrawer/GL_ShapeDrawer.cpp
	include/debugdrawer/GL_ShapeDrawer.h
	include/debugdrawer/GLDebugDrawer.cpp
	include/debugdrawer/GLDebugDrawer.h
	include/debugdrawer/DemoApplication.cpp
	include/debugdrawer/DemoApplication.h
	include/debugdrawer/GlutDemoApplication.cpp
	include/debugdrawer/GlutDemoApplication.h
	include/debugdrawer/GlutStuff.cpp
	include/debugdrawer/GlutStuff.h
	include/debugdrawer/Win32DemoApplication.cpp
	include/debugdrawer/Win32DemoApplication.h
)
add_library(OpenGLSupport ${OpenGLSupport})

set(ObjConvexDecomposition tool/obj_convex_decomposition.cpp)
add_library(objConvexDecomposition ${ObjConvexDecomposition})

set(ObjectDataProperty src/object_data_property.cpp  src/scene_physics_penalty.cpp)
add_library(objectDataProperty ${ObjectDataProperty})

add_library(ObjRecRANSACTool include/ObjRecRANSACTool/ObjRecRANSACTool.cpp) 

set(PhysicsEngine src/scene_physics_engine.cpp src/scene_physics_support.cpp src/scene_data_forces.cpp)
add_library(physicsEngine ${PhysicsEngine})

set(SequentialSceneHypothesis src/sequential_scene_hypothesis)
add_library(SequentialSceneHypothesis ${SequentialSceneHypothesis})

set(SequentialSceneParsing src/sequential_scene_parsing.cpp)
add_library(sequentialSceneParsing ${SequentialSceneParsing})

set(RosSequentialSceneParsing src/ros_sequential_scene_parsing.cpp)
add_library(rosSequentialSceneParsing ${RosSequentialSceneParsing})

## Declare a C++ executable
add_executable(obj_main_decomposition tool/main.cpp)
add_executable(sequential_scene_ros src/main.cpp)
add_executable(load_and_pub_pcd src/publish_pcd_to_ros.cpp)
# add_executable(bare_scene_ros src/bare_main.cpp)

## Add cmake target dependencies of the executable
## same as for the library above

## Tool for creating bullet collision object from obj files
target_link_libraries(
 obj_main_decomposition objConvexDecomposition
)

## Main code for scene parsing

target_link_libraries(
 OpenGLSupport ${BULLET_LIBRARIES}  ${GLUT_glut_LIBRARY} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
)

target_link_libraries(
 objConvexDecomposition ${BULLET_LIBRARIES} ConvexDecomposition HACD
)

target_link_libraries(
 objectDataProperty ${BULLET_LIBRARIES}  BulletWorldImporter BulletFileLoader
)

target_link_libraries(
 ObjRecRANSACTool ${PCL_LIBRARIES} ${VTK_LIBS} ${BULLET_LIBRARIES} ${ObjRecRANSAC_LIBRARY}
)

target_link_libraries(
 physicsEngine objectDataProperty OpenGLSupport ${BULLET_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES}
)

target_link_libraries(
 SequentialSceneHypothesis physicsEngine ${Boost_LIBRARIES} ${BULLET_LIBRARIES} ${PCL_LIBRARIES}
)

target_link_libraries(
 sequentialSceneParsing SequentialSceneHypothesis objectDataProperty physicsEngine ObjRecRANSACTool
)

target_link_libraries(
 rosSequentialSceneParsing objectDataProperty sequentialSceneParsing ${catkin_LIBRARIES}
)

target_link_libraries(
 sequential_scene_ros physicsEngine rosSequentialSceneParsing
)

#target_link_libraries(
# bare_scene_ros rosSequentialSceneParsing
#)

add_executable(draw_bullet_hello_world tool/main_bullet_hello_world_draw.cpp tool/bullet_hello_world_draw.cpp )

target_link_libraries(
 draw_bullet_hello_world ${BULLET_LIBRARIES} BulletWorldImporter BulletFileLoader OpenGLSupport 
)

target_link_libraries(load_and_pub_pcd  ${catkin_LIBRARIES} ${PCL_LIBRARIES})
