project( KF2 )

cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

if( BUILD_KF2 )
option(BUILD_KF2 "Build the Extended Kalman Filter application." ON)
add_executable( kf2 main.cpp )
#   target_link_libraries(kf2
#     ${PROTOBUF_LIBRARY}
#      )
endif( BUILD_KF2 )
