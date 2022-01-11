project(tests_main)

set(SRC_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR})
set(MOCKS_DIR ${TESTS_DIR}/mocks)

message("Tests source dirs: " ${SRC_DIRECTORIES})
message("Mocks dirs:        " ${MOCKS_DIR})

if(DEFINED ENV{CPPUTEST_HOME})
    message(STATUS "Using CppUTest home: $ENV{CPPUTEST_HOME}")
    set(CPPUTEST_INCLUDE_DIRS $ENV{CPPUTEST_HOME}/include)
    set(CPPUTEST_LIBRARIES $ENV{CPPUTEST_HOME}/lib)
    set(CPPUTEST_LDFLAGS CppUTest CppUTestExt)
else()
    find_package(PkgConfig REQUIRED)
    #find_package(CPPUTEST REQUIRED cpputest>=3.8)
    pkg_search_module(CPPUTEST REQUIRED cpputest>=3.8)
    message(STATUS "Found CppUTest version ${CPPUTEST_VERSION}")
endif()

add_subdirectory(${MOCKS_DIR})

file(GLOB SOURCES
        "${SRC_DIRECTORIES}/*.h"
        "${SRC_DIRECTORIES}/*.cpp"
        "${SRC_DIRECTORIES}/*.c"
        "${MOCKS_DIR}"
        "${PRODUCTION_DIR}"
        )

include_directories(
        ${CPPUTEST_INCLUDE_DIRS}
        ${MOCKS_DIR}
        ${PRODUCTION_DIR}
)

link_directories(${CPPUTEST_LIBRARIES})

add_executable(as5600_tests as5600_tests.cpp ${SOURCES})

target_link_libraries(as5600_tests
                      src
                      mocks
                      ${CPPUTEST_LDFLAGS})


