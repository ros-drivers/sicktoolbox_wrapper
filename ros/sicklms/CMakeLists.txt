SET(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "${CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS} -lsicklms-1.0 -lpthread")

rosbuild_add_executable(sicklms sicklms.cpp)
