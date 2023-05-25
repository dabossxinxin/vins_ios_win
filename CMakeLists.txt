cmake_minimum_required(VERSION 3.18)
set(CMAKE_BUILD_TYPE "Release")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
project(vins)

if(WIN32)
	add_definitions(-DNOMINMAX)
	set(CMAKE_CXX_FLAGS_DEBUG "/bigobj /EHsc")
	set(CMAKE_CXX_FLAGS_RELEASE "-O2 /bigobj /EHsc")
endif()

find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
#find_package(Pangolin REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)

include_directories(${CERES_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

include_directories(${CMAKE_SOURCE_DIR}/thirdparty)
include_directories(${CMAKE_SOURCE_DIR}/estimator)

########################################################
#													   #
#			          编译DBoW库			           #
#													   #	
########################################################
file(GLOB VocaHdrs		"./thirdparty/*.hpp")
file(GLOB DBoWHdrs		"./thirdparty/DBoW/*.h")
file(GLOB DUtilsHdrs	"./thirdparty/DUtils/*.h")
file(GLOB DVsionHdrs	"./thirdparty/DVision/*.h")

file(GLOB VocaSrcs		"./thirdparty/*.cpp")
file(GLOB DBoWSrcs		"./thirdparty/DBoW/*.cpp")
file(GLOB DUtilsSrcs	"./thirdparty/DUtils/*.cpp")
file(GLOB DVsionSrcs	"./thirdparty/DVision/*.cpp")

source_group("voc hdr"			FILES ${VocaHdrs})
source_group("voc src"			FILES ${VocaSrcs})
source_group("DBoW hdr"			FILES ${DBoWHdrs})
source_group("DBoW src"			FILES ${DBoWSrcs})
source_group("DUtils hdr" 		FILES ${DUtilsHdrs})
source_group("DUtils src" 		FILES ${DUtilsSrcs})
source_group("DVsion hdr" 		FILES ${DVsionHdrs})
source_group("DVsion src" 		FILES ${DVsionSrcs})

add_library(DBoW 
	${VocaHdrs}
	${VocaSrcs}
	${DBoWHdrs}
	${DBoWSrcs}
	${DUtilsHdrs}
	${DUtilsSrcs}
	${DVsionHdrs}
	${DVsionSrcs}
)

target_link_libraries(DBoW 
	${OpenCV_LIBRARIES}
) 

########################################################
#													   #
#			          编译player库			           #
#													   #	
########################################################
file(GLOB WinPlayerSrcs	"./win_player/*.h")
file(GLOB WinPlayerHdrs	"./win_player/*.cpp")

source_group("Source" FILES ${VioSrcs})
source_group("Header" FILES ${VioHdrs})

#add_library(win_player
#	${WinPlayerSrcs} 
#	${WinPlayerHdrs}
#)

#target_link_libraries(win_player 
#	${Pangolin_LIBRARIES} 
#	${OpenCV_LIBRARIES}
#)

########################################################
#													   #
#			编译visual inertial odometry库			   #
#													   #	
########################################################
file(GLOB VioHdrs				"./estimator/*.h")
file(GLOB VioFactorHdrs			"./estimator/factor/*.h")
file(GLOB VioUtilityHdrs		"./estimator/utility/*.h")
file(GLOB VioInitialHdrs		"./estimator/initial/*.h")
file(GLOB VioMyBackEndHdrs		"./estimator/back_end/*.h")
file(GLOB VioLoopClosureHdrs	"./estimator/loop_closure/*.h")
file(GLOB VioFeatureTrackHdrs	"./estimator/feature_tracker/*.h")

file(GLOB VioSrcs				"./estimator/*.cpp")
file(GLOB VioFactorSrcs			"./estimator/factor/*.cpp")
file(GLOB VioUtilitySrcs		"./estimator/utility/*.cpp")
file(GLOB VioInitialSrcs		"./estimator/initial/*.cpp")
file(GLOB VioMyBackEndSrcs		"./estimator/back_end/*.cpp")
file(GLOB VioLoopClosureSrcs	"./estimator/loop_closure/*.cpp")
file(GLOB VioFeatureTrackSrcs	"./estimator/feature_tracker/*.cpp")

source_group("main src"				FILES ${VioSrcs})
source_group("main hdr"				FILES ${VioHdrs})
source_group("factor src"			FILES ${VioFactorSrcs})
source_group("factor hdr"			FILES ${VioFactorHdrs})
source_group("utility src" 			FILES ${VioUtilitySrcs})
source_group("utility hdr" 			FILES ${VioUtilityHdrs})
source_group("initial src" 			FILES ${VioInitialSrcs})
source_group("initial hdr" 			FILES ${VioInitialHdrs})
source_group("back_end src"			FILES ${VioMyBackEndSrcs})
source_group("back_end hdr"			FILES ${VioMyBackEndHdrs})
source_group("loop_closure src" 	FILES ${VioLoopClosureSrcs})
source_group("loop_closure hdr" 	FILES ${VioLoopClosureHdrs})
source_group("feature_tracker src" 	FILES ${VioFeatureTrackSrcs})
source_group("feature_tracker hdr" 	FILES ${VioFeatureTrackHdrs})

add_library(vins_estimator 
	${VioHdrs} 
	${VioSrcs}
	${VioFactorSrcs}
	${VioFactorHdrs}
	${VioUtilitySrcs}
	${VioUtilityHdrs}
	${VioInitialSrcs}
	${VioInitialHdrs}
	${VioMyBackEndSrcs}
	${VioMyBackEndHdrs}
	${VioLoopClosureSrcs}
	${VioLoopClosureHdrs}
	${VioFeatureTrackSrcs}
	${VioFeatureTrackHdrs}
)

target_link_libraries(vins_estimator 
	${CERES_LIBRARIES} 
	${OpenCV_LIBRARIES}
	DBoW
)

if(TARGET DBoW AND TARGET vins_estimator)
  add_subdirectory(ios_player)
endif()

add_executable(example ./example/estimator_node.cpp)
target_link_libraries(example vins_estimator win_player)