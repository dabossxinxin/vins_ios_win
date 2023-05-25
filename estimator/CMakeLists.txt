
########################################################
#													   #
#			编译visual inertial odometry库			   #
#													   #	
########################################################
file(GLOB VioHdrs				"./*.h")
file(GLOB VioFactorHdrs			"./factor/*.h")
file(GLOB VioUtilityHdrs		"./utility/*.h")
file(GLOB VioInitialHdrs		"./initial/*.h")
file(GLOB VioMyBackEndHdrs		"./back_end/*.h")
file(GLOB VioLoopClosureHdrs	"./loop_closure/*.h")
file(GLOB VioFeatureTrackHdrs	"./feature_tracker/*.h")

file(GLOB VioSrcs				"./*.cpp")
file(GLOB VioFactorSrcs			"./factor/*.cpp")
file(GLOB VioUtilitySrcs		"./utility/*.cpp")
file(GLOB VioInitialSrcs		"./initial/*.cpp")
file(GLOB VioMyBackEndSrcs		"./back_end/*.cpp")
file(GLOB VioLoopClosureSrcs	"./loop_closure/*.cpp")
file(GLOB VioFeatureTrackSrcs	"./feature_tracker/*.cpp")

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