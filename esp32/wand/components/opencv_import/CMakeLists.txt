idf_component_register(SRC_DIRS "." "upstream/features2d/src" "upstream/core/src" "upstream/core/src/parallel" "upstream/imgproc/src"
                    INCLUDE_DIRS "include"
                    PRIV_INCLUDE_DIRS "upstream/core/include" "upstream/features2d/include" "upstream/generated/include" "upstream/imgproc/include"
                )
add_definitions(-D__OPENCV_BUILD)
add_definitions(-D__ESP32__)