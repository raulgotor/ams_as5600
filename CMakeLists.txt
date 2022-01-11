cmake_minimum_required(VERSION 3.0)
set(CMAKE_C_STANDARD 99)

project(ams_as5600 VERSION 1.0 LANGUAGES C CXX)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)

SET(TARGET_GROUP test)

set(PRODUCTION_DIR ${CMAKE_SOURCE_DIR}/src)

#SET(GCC_COVERAGE_COMPILE_FLAGS "-g -O0 ")
#SET(GCC_COVERAGE_LINK_FLAGS    "-coverage")

SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}" )
SET( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}" )



message("Building ${TARGET_GROUP}")
if(TARGET_GROUP STREQUAL production)

    message("Building Production")

elseif(TARGET_GROUP STREQUAL test)

    set(TESTS_DIR ${CMAKE_SOURCE_DIR}/tests)
    add_subdirectory(${TESTS_DIR})

else()
    message(FATAL_ERROR "I don't know the TARGET_GROUP you gave me!")
endif()

add_subdirectory(${PRODUCTION_DIR})
