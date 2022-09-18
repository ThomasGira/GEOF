cmake_minimum_required(VERSION 2.8)
project( GEOFF )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
find_package(Curses REQUIRED)
include_directories(${CURSES_INCLUDE_DIR})

add_subdirectory(simulator)
add_subdirectory(utils)
add_subdirectory(navigation)

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


add_executable( CarTest ${PROJECT_SOURCE_DIR}/simulator/car_test.cc)
target_link_libraries( CarTest 
                        PUBLIC vectors 
                        PUBLIC visualizer 
                        PUBLIC simulator
                        ${OpenCV_LIBS})
target_include_directories(CarTest 
                        PUBLIC "${PROJECT_SOURCE_DIR}/utils"
                           )

add_executable( KeyTest ${PROJECT_SOURCE_DIR}/simulator/key_test.cc)
target_link_libraries( KeyTest 
                        PUBLIC vectors 
                        PUBLIC visualizer 
                        PUBLIC simulator
                        PUBLIC keyboard
                        ${CURSES_LIBRARIES}
                        ${OpenCV_LIBS}
                           )
target_include_directories(CarTest 
                        PUBLIC "${PROJECT_SOURCE_DIR}/utils"
                        PUBLIC "${PROJECT_SOURCE_DIR}/navigation"
                           )
