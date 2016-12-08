cmake_minimum_required(VERSION 2.8.3)
project(asr_visualization_server)

# C++11 is needed
add_definitions(-Wall -std=gnu++0x -ggdb)
set(CMAKE_BUILD_TYPE Debug)
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()



find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	message_generation
        rapidxml
        roslib
)


add_service_files(
   DIRECTORY srv
   FILES DrawAllModelsDome.srv DrawAllModelsMild.srv ClearAllModels.srv DrawModelDome.srv DrawModelMild.srv
         ClearModelDome.srv ClearModelMild.srv ShowAvailableModels.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
	INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp std_msgs rapidxml roslib
        DEPENDS
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}

)



FILE(GLOB_RECURSE LIBFILES "include/*.h")

add_executable(visualization src/environment_visualizer.cpp
                                      src/marker_helper.cpp
                                      ${LIBFILES})

add_dependencies(visualization
        visualization_server_generate_messages_cpp
)

TARGET_LINK_LIBRARIES(visualization
      ${catkin_LIBRARIES}
)




