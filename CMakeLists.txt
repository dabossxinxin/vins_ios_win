cmake_minimum_required(VERSION 3.18)
set(CMAKE_BUILD_TYPE "Release")
project(vins_ios_win)
cmake_policy(SET CMP0111 OLD)

if(WIN32)
	add_definitions(-DNOMINMAX)
	set(CMAKE_CXX_FLAGS_DEBUG "/bigobj /EHsc")
	set(CMAKE_CXX_FLAGS_RELEASE "-O2 /bigobj /EHsc")
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(rabbitmq-c REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)

find_path(JSON_INCLUDE_DIR NAMES json HINTS ../jsoncpp/include)
find_file(JSON_RELEASE_LIB json_vc141_libmt.lib HINTS ../jsoncpp/lib)
find_file(JSON_DEBUG_LIB json_vc141_libmtd.lib HINTS ../jsoncpp/lib)

include_directories(${JSON_INCLUDE_DIR})
include_directories(${CERES_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

include_directories(${CMAKE_SOURCE_DIR}/ios_player)
include_directories(${CMAKE_SOURCE_DIR}/win_player)
include_directories(${CMAKE_SOURCE_DIR}/thirdparty)
include_directories(${CMAKE_SOURCE_DIR}/estimator)

add_subdirectory(thirdparty)

if(TARGET DBoW)
	add_subdirectory(estimator)
endif()

if(WIN32)
	add_subdirectory(win_player)
endif()

if(WIN32 AND TARGET win_player)
	add_subdirectory(example)
endif()

if(IOS AND TARGET DBoW AND TARGET vins_estimator)
	add_subdirectory(ios_player)
endif()