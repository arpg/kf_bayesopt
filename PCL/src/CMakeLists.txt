
#list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
#add_executable(pcd_write_test pcd_write.cpp)
#target_link_libraries(pcd_write_test ${PCL_LIBRARIES})
add_executable(ndt_time ndt.cpp)
target_link_libraries(ndt_time ${PCL_LIBRARIES})

add_executable(icp_time icp.cpp)
target_link_libraries(icp_time ${PCL_LIBRARIES})

add_executable(write_pcd write_pcd.cpp)
target_link_libraries(write_pcd ${PCL_LIBRARIES})

