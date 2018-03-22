cmake_minimum_required(VERSION 2.8)
project(CollisionAlgorithm)

file(GLOB_RECURSE HEADER_FILES
    "src/*.h"    
    "src/*.inl"
)

file(GLOB_RECURSE SOURCE_FILES
    "src/*.cpp"
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pedantic")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -rdynamic")

set(CMAKE_CXX_STANDARD 11)
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../graFE/src")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../MechanicalAlgorithm/src")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pedantic")
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "..")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "..")

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES} ${HEADER_FILES} )

set(COLLISIONALGORITHM_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${COLLISIONALGORITHM_INCLUDE_DIRS}>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

target_link_libraries(${PROJECT_NAME} GL graFE MechanicalAlgorithm)

## Install rules for the library and headers; CMake package configurations files
create_package(${PROJECT_NAME})

install(DIRECTORY sofa
        DESTINATION include/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.h" PATTERN "*hpp" PATTERN "*.inl"
)
