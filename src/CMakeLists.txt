file(GLOB SOURCES ${CMAKE_CURRENT_LIST_DIR}/*.c)

message("Production sources: ${SOURCES}")

add_library(src ${SOURCES})
target_include_directories(src PUBLIC ${CMAKE_CURRENT_LIST_DIR})