add_library( xml_process
        tinyxml2.cpp)

add_executable(MetadataExtractor MetadataExtractor.cpp TinyEXIF.cpp)
target_link_libraries(MetadataExtractor
        xml_process)