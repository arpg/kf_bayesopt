find_package( Eigen3 REQUIRED )
find_package( Sophus REQUIRED )
find_package( glog REQUIRED )

list(APPEND KF_INC ${EIGEN3_INCLUDE_DIR}
  ${Sophus_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
  )

include_directories( ${KF_INC} )

add_subdirectory( kf1 )
add_subdirectory( kf2 )
