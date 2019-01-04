cmake_minimum_required(VERSION 3.1)
cmake_policy(SET CMP0048 OLD)
project(CollisionAlgorithm)

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

file(GLOB_RECURSE HEADER_FILES
    "*.h"
    "*.inl"
)

file(GLOB_RECURSE SOURCE_FILES
    "*.cpp"
)

file(GLOB_RECURSE SCENES_FILES
    "scenes/*.scn"
    "scenes/*.xml"
)

set (CMAKE_CXX_STANDARD 11)

find_package( OpenGL REQUIRED )
find_package( Qt5 COMPONENTS OpenGL REQUIRED)
find_package( Eigen3 REQUIRED )

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories("${CMAKE_CURRENT_SOURCE_DIR}")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/src")

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES} ${HEADER_FILES} ${SCENES_FILES} ${README_FILES} )
set(CollisionAlgorithm_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/src")

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CollisionAlgorithm_INCLUDE_DIRS}>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DPLUGIN_DATA_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\"")
target_link_libraries(${PROJECT_NAME} SofaCore SofaSimulationCommon Qt5::OpenGL)

## Install rules for the library and headers; CMake package configurations files
sofa_create_package(${PROJECT_NAME} 0.1 ${PROJECT_NAME} ${PROJECT_NAME})

install(DIRECTORY sofa
        DESTINATION include/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.h" PATTERN "*hpp" PATTERN "*.inl"
)

