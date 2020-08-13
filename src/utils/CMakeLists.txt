###########################################################################
####################### XML processing ###################################
###########################################################################

add_library( xml_process
        EXIF_Parsing/tinyxml2.cpp)

add_executable(MetadataExtractor EXIF_Parsing/MetadataExtractor.cpp EXIF_Parsing/TinyEXIF.cpp)
target_link_libraries(MetadataExtractor
        xml_process)


###########################################################################
####################### Gridmap core system ###############################
###########################################################################

add_library(gridmap_core
        GridMap/gridmap_core
        GridMap/gridmap_core/GridMap.cpp
        GridMap/gridmap_core/GridMapMath.cpp
        GridMap/gridmap_core/SubmapGeometry.cpp
        GridMap/gridmap_core/BufferRegion.cpp
        GridMap/gridmap_core/Polygon.cpp
        GridMap/gridmap_core/CubicInterpolation.cpp
        GridMap/gridmap_core/iterators/GridMapIterator.cpp
        GridMap/gridmap_core/iterators/SubmapIterator.cpp
        GridMap/gridmap_core/iterators/CircleIterator.cpp
        GridMap/gridmap_core/iterators/EllipseIterator.cpp
        GridMap/gridmap_core/iterators/SpiralIterator.cpp
        GridMap/gridmap_core/iterators/PolygonIterator.cpp
        GridMap/gridmap_core/iterators/LineIterator.cpp
        GridMap/gridmap_core/iterators/SlidingWindowIterator.cpp
        )

###########################################################################
####################### gridmap CV ########################################
###########################################################################

add_library(gridmap_cv
        GridMap/gridmap_cv/GridMapCvProcessing.cpp
        )

target_link_libraries(gridmap_cv
        gridmap_core
        ${LINK_LIBS_EXTERNAL})