file(GLOB SOURCES ${MOCKS_DIR}/*.cpp)

include_directories(
        ${MOCKS_DIR}
        ${PRODUCTION_DIR}
)

add_library(mocks ${SOURCES})
target_include_directories(mocks PUBLIC ${CMAKE_CURRENT_LIST_DIR})

