# Application layer.
add_library(h7_app STATIC)
target_sources(h7_app PRIVATE robot_application.c robot_task.c dm_id_application.c)
target_include_directories(h7_app PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(h7_app PRIVATE h7_model)
