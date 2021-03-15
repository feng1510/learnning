project(Fre2Carte)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11 -O3)

find_package(OpenCV)
find_package(Eigen3 REQUIRED)

if(OPENCV_FOUND)
	message(${OpenCV_DIR})
endif(OPENCV_FOUND)
set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

file(GLOB sources 
	${PROJECT_SOURCE_DIR}/src/*.cpp
)
file(GLOB headers 
	${PROJECT_SOURCE_DIR}/include/*.h 
)

include_directories(include)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

include_directories(/usr/local/include, ${EIGEN3_INCLUDE_DIRS})
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

add_executable(Fre2Carte src/main.cpp ${sources} ${headers})

target_link_libraries(Fre2Carte z ssl uv uWS ${OpenCV_LIBS})