SET(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "${CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS} -lsicklms-1.0 -lpthread")

rosbuild_add_executable(print_scans print_scans.cpp)

rosbuild_add_executable(time_scans time_scans.cpp)

rosbuild_add_executable(log_scans log_scans.cpp)

