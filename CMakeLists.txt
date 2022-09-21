CMAKE_MINIMUM_REQUIRED(VERSION 3.5)
PROJECT(InfantryAutoaim)


SET(CMAKE_CXX_STANDARD 14)
SET(CMAKE_BUILD_TYPE Debug)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DPATH=\"\\\"${PROJECT_SOURCE_DIR}\\\"\"")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D${CMAKE_SYSTEM_NAME}")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

FIND_PACKAGE(OpenCV 4 REQUIRED)
FIND_PACKAGE(Threads)
FIND_PACKAGE(Eigen3 REQUIRED)

INCLUDE_DIRECTORIES(
	"include/"
	"include/mercure"
	"include/Armor"
	"include/Const"
	"include/Debug"
	"include/Extend"
	"include/ImageProgress"
	"include/Serial"
	"include/Solver"
	"include/Rune"
	${OPENCV_INCLUDE_DIRECTORIES}
	${Eigen_INCLUDE_DIRECTORIES}
)
add_subdirectory(
	${CMAKE_CURRENT_SOURCE_DIR}/src/mercure
)
FILE(GLOB_RECURSE sourcefiles "src/*.cpp")

ADD_EXECUTABLE(Autoaim  ${sourcefiles})
TARGET_LINK_LIBRARIES(Autoaim  mercure_driver  ${OpenCV_LIBS})
