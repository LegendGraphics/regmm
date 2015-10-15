cmake_minimum_required(VERSION 2.8)

project(figtree)

include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/include)

set(incs		include/figtree.h
				include/figtree_internal.h
				include/KCenterClustering.h)
				
set(srcs		src/KCenterClustering.cpp
				src/figtree.cpp)

add_library (figtree SHARED ${srcs} ${incs})

set_target_properties(figtree PROPERTIES DEBUG_POSTFIX _debug)
set_target_properties(figtree PROPERTIES RELEASE_POSTFIX _release)

set(CMAKE_INSTALL_PREFIX 	${PROJECT_SOURCE_DIR})
install(TARGETS figtree DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)


