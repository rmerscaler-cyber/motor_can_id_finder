# Board support package, same layer boundary as lyh_cmake_duo_down.
add_library(h7_bsp STATIC)
target_sources(h7_bsp PRIVATE bsp_can.c cmd_debug.c)
target_include_directories(h7_bsp PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(h7_bsp PUBLIC h7_platform)
