cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(kdtree_search)
project(pointPick)
project(areaPick)
find_package(PCL 1.2 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ~/workspace/iCrane_08_11/iCrane/thirdparty/quanergy/include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (kdtree_search kdtree_search.cpp)
add_executable (pointPick pointpicking.cpp)
add_executable (areaPick areaPicking.cpp)


target_link_libraries (kdtree_search ${PCL_LIBRARIES}
                      ${OpenCV_LIBS})
target_link_libraries (pointPick ${PCL_LIBRARIES}
                      ${OpenCV_LIBS})
target_link_libraries (areaPick ${PCL_LIBRARIES}
                      ${OpenCV_LIBS})
