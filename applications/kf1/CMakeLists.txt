project( KF1 )

cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

option(BUILD_KF1 "Build the Kalman Filter application." ON)

if( BUILD_KF1 )
add_executable( kf1 main.cpp )
#   target_link_libraries(kf2
#     ${PROTOBUF_LIBRARY}
#      )
endif( BUILD_KF1 )
