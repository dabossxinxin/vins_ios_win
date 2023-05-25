
########################################################
#													   #
#			          编译player库			           #
#													   #	
########################################################
file(GLOB WinPlayerHdrs	"./*.h")
file(GLOB WinPlayerSrcs	"./*.cpp")

source_group("Source" FILES ${WinPlayerSrcs})
source_group("Header" FILES ${WinPlayerHdrs})

add_library(win_player
	${WinPlayerSrcs} 
	${WinPlayerHdrs}
)

target_link_libraries(win_player 
	${Pangolin_LIBRARIES}
	${OpenCV_LIBRARIES}
)