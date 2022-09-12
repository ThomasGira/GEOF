cmake_minimum_required(VERSION 2.8)
project( GEOFF )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_subdirectory(simulator)
add_subdirectory(utils)

add_executable( VizTest ${PROJECT_SOURCE_DIR}/simulator/visualizer_test.cc)
target_link_libraries( VizTest 
                            PUBLIC visualizer
                            ${OpenCV_LIBS} )
target_include_directories(VizTest 
                            PUBLIC "${PROJECT_SOURCE_DIR}/simulator"
                           ${EXTRA_INCLUDES}
                           )


add_executable( VecTest ${PROJECT_SOURCE_DIR}/utils/vectors_test.cc)
target_link_libraries( VecTest 
                        PUBLIC vectors)
target_include_directories(VecTest 
                        PUBLIC "${PROJECT_SOURCE_DIR}/utils"
                           )


add_executable( CollisionTest ${PROJECT_SOURCE_DIR}/simulator/collision_test.cc)
target_link_libraries( CollisionTest 
                        PUBLIC vectors 
                        PUBLIC visualizer 
                        PUBLIC simulator
                        ${OpenCV_LIBS})
target_include_directories(VecTest 
                        PUBLIC "${PROJECT_SOURCE_DIR}/utils"
                           )