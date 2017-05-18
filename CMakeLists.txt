cmake_minimum_required(VERSION 2.8.12)
project(CollisionAlgorithmPlugin)

find_package(SofaFramework REQUIRED)
find_package(SofaBase REQUIRED)
find_package(SofaSimulation REQUIRED)
find_package(SofaGeneral REQUIRED)

#find_package(TRI6FEM REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

set(HEADER_FILES      
     src/initCollisionAlgorithmPlugin.h
     src/AABBDecorator.h
     src/AABBDecorator.inl
     src/CollisionAlgorithm.h
)

set(SOURCE_FILES
     src/initCollisionAlgorithmPlugin.cpp
     src/AABBDecorator.cpp
     src/FindClosestPoint.cpp
     src/BindPointCloud.cpp
)

file(GLOB_RECURSE SCENES_FILES
    "scenes/*.scn"
    "scenes/*.xml"
)

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES} ${HEADER_FILES} ${SCENES_FILES} ${README_FILES} )
#set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")

set(COLLISIONALGORITHMPLUGIN_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${COLLISIONALGORITHMPLUGIN_INCLUDE_DIRS}>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DPLUGIN_DATA_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\"")
#target_link_libraries(${PROJECT_NAME} SofaCore SofaComponentBase SofaSimulationTree SofaUserInteraction SofaConstraint TRI6FEM)
target_link_libraries(${PROJECT_NAME} SofaCore SofaComponentBase SofaSimulationTree SofaUserInteraction SofaConstraint ConstraintGeometryPlugin)

## Install rules for the library and headers; CMake package configurations files
sofa_create_package(${PROJECT_NAME} 0.1 ${PROJECT_NAME} ${PROJECT_NAME})

install(DIRECTORY sofa
        DESTINATION include/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.h" PATTERN "*hpp" PATTERN "*.inl"
)
