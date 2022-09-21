Cmake_minimum_required (VERSION 3.5)


project(mercure_driver)

message(STATUS "Configuring Sub_project:mercure_drive")

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}

)

#detect_factory

add_library(mercure_driver mercure_driver.cpp)

target_link_libraries(mercure_driver
    ${OpenCV_LIBRARIES}
	gxiapi
)


