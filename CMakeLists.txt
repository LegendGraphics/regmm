project(regmm)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set( EIGEN3_INCLUDE_DIR $ENV{EIGEN3_INCLUDE_DIR} )

find_package(3rdParty)
include_directories(${FIGTREE_INCLUDE_DIR})   

include_directories(${EIGEN3_INCLUDE_DIR})
    
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(regmm_io 				regmm/io)
set(regmm_transform 		regmm/transform)
set(regmm_transform_cpd		regmm/transform/cpd)
set(regmm_transform_arap	regmm/transform/arap)
set(regmm_3rdparty			regmm/3rdparty)
set(regmm_3rdparty_figtree	regmm/3rdparty/figtree)
set(regmm_3rdparty_tinyobj	regmm/3rdparty/tinyobj)

		 
set(regmm_io_files	${regmm_io}/points.hpp
					${regmm_io}/mesh.hpp
					${regmm_io}/basic_types.hpp
                    ${regmm_io}/cast.hpp)

set(regmm_transform_files	${regmm_transform}/registrator.hpp
							${regmm_transform}/regmm_engine.hpp)
set(regmm_transform_cpd_files	${regmm_transform_cpd}/cpd_base.hpp
								${regmm_transform_cpd}/cpd_rigid.hpp
								${regmm_transform_cpd}/cpd_nonrigid.hpp
								${regmm_transform_cpd}/parameters.hpp
								${regmm_transform_cpd}/fgt_wrapper.hpp
								${regmm_transform_cpd}/low_rank.hpp)
				
set(regmm_transform_arap_files	${regmm_transform_arap}/constraint.hpp
								${regmm_transform_arap}/arap.hpp
								${regmm_transform_arap}/data_fitting.hpp
								${regmm_transform_arap}/arap_solver.hpp)
				
set(regmm_3rdparty_tinyobj_files	${regmm_3rdparty_tinyobj}/tiny_obj_loader.h
									${regmm_3rdparty_tinyobj}/tiny_obj_loader.cc
									${regmm_3rdparty_tinyobj}/obj_writer.h
									${regmm_3rdparty_tinyobj}/obj_writer.cc)

set(regmm_3rdparty_figtree_files	${regmm_3rdparty_figtree}/include/figtree.h)


#Organize the files
set(Sources		"Source Files")

set(srcfiles_io				${Sources}\\io)
set(srcfiles_transform		${Sources}\\transform)
set(srcfiles_transform_cpd	${Sources}\\transform\\cpd)
set(srcfiles_transform_arap	${Sources}\\transform\\arap)
set(srcfiles_3rdparty_tinyobj	${Sources}\\3rdparty\\tinyobj)
set(srcfiles_3rdparty_figtree	${Sources}\\3rdparty\\figtree)

source_group(${srcfiles_io} FILES ${regmm_io_files})
source_group(${srcfiles_transform} FILES ${regmm_transform_files})
source_group(${srcfiles_transform_cpd} FILES ${regmm_transform_cpd_files})
source_group(${srcfiles_transform_arap} FILES ${regmm_transform_arap_files})
source_group(${srcfiles_3rdparty_tinyobj} FILES ${regmm_3rdparty_tinyobj_files})
source_group(${srcfiles_3rdparty_figtree} FILES ${regmm_3rdparty_figtree_files})


set(LINK_LIBRARY ${LINK_LIBRARY} ${FIGTREE_LIBRARY})
set(srcs		${regmm_io_files} ${regmm_transform_files} ${regmm_transform_cpd_files} ${regmm_transform_arap_files} ${regmm_3rdparty_tinyobj_files} ${regmm_3rdparty_figtree_files})
set(srcs 		${srcs} regmm/main.cpp)

add_executable(regmm ${srcs})
target_link_libraries(regmm ${LINK_LIBRARY})


