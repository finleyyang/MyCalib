file(GLOB TEST_CameraCalib RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.[ch]pp")

add_executable(${PROJECT_NAME}_TEST_CameraCalib
        ${TEST_CameraCalib})

target_link_libraries(${PROJECT_NAME}_TEST_CameraCalib
        ${MyCalib_LIBNAME})