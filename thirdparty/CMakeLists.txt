
########################################################
#													   #
#			          编译DBoW库			           #
#													   #	
########################################################
file(GLOB VocaHdrs		"./*.hpp")
file(GLOB DBoWHdrs		"./DBoW/*.h")
file(GLOB DUtilsHdrs	"./DUtils/*.h")
file(GLOB DVsionHdrs	"./DVision/*.h")

file(GLOB VocaSrcs		"./*.cpp")
file(GLOB DBoWSrcs		"./DBoW/*.cpp")
file(GLOB DUtilsSrcs	"./DUtils/*.cpp")
file(GLOB DVsionSrcs	"./DVision/*.cpp")

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

target_link_libraries(DBoW ${OpenCV_LIBRARIES}) 