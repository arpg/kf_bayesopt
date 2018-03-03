project( KF1 )

cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
SET(CMAKE_CXX_FLAGS -pthread)
find_package( OpenCV 2.4 REQUIRED )
option(BUILD_KF1 "Build the Kalman Filter application." ON)

#add_executable(testProcessModel testProcessModel.cpp) 

#
#add_executable(testSimulator testSimulator.cpp Simulator.cpp) 

add_executable(testTrial testTrial.cpp Trial.cpp Simulator.cpp KalmanFilter.cpp) 

#add_executable(testExperiment testExperiment.cpp  Experiment.cpp Trial.cpp
#Simulator.cpp KalmanFilter.cpp) 

#add_executable(bayes bayes.cpp Trial.cpp Simulator.cpp KalmanFilter.cpp) 
#target_link_libraries(bayes ${OpenCV_LIBS} )

#if( BUILD_KF1 )
#add_executable( kf1
#Experiment.cpp
##ProcessModel.cpp
#Trial.cpp
#main.cpp )
#   target_link_libraries(kf2
#     ${PROTOBUF_LIBRARY}
#      )
#endif( BUILD_KF1 )
