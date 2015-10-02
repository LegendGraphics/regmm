project(regmm)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set( EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}" )
if( NOT EIGEN3_INCLUDE_DIR )
    message( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation." )
endif()

include_directories(${EIGEN3_INCLUDE_DIR})
    
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# set(src_base include/base)
# set(src_core include/core)
# set(src_fast include/fast)
# set(src_disp include/disp)
		 
# set(srcs_base	${src_base}/matrix.hpp
				# ${src_base}/parameters.hpp
				# ${src_base}/data.hpp)

# set(srcs_core	${src_core}/registrator.hpp
				# ${src_core}/cpd_base.hpp
				# ${src_core}/cpd_rigid.hpp
				# ${src_core}/cpd_affine.hpp
				# ${src_core}/cpd_nonrigid.hpp)

# set(srcs_fast   ${src_fast}/low_rank.hpp
                # ${src_fast}/fgt_wrapper.hpp)
                
# set(srcs_disp   ${src_disp}/render_thread.hpp)
				
# set(srcs_test	src/test.cpp)  

# #Organize the files
# set(Sources		"Source Files")

# set(srcfiles_base		${Sources}\\base)
# set(srcfiles_core		${Sources}\\core)
# set(srcfiles_fast		${Sources}\\fast)
# set(srcfiles_disp		${Sources}\\disp)
# set(srcfiles_test		${Sources})

# source_group(${srcfiles_base} FILES ${srcs_base})
# source_group(${srcfiles_core} FILES ${srcs_core})
# source_group(${srcfiles_fast} FILES ${srcs_fast})
# source_group(${srcfiles_disp} FILES ${srcs_disp})
# source_group(${srcfiles_test} FILES ${srcs_test})


# set(LINK_LIBRARY ${LINK_LIBRARY} ${OPENSCENEGRAPH_LIBRARIES} ${FIGTREE_LIBRARY})
# set(srcs		${srcs_base} ${srcs_core} ${srcs_fast} ${srcs_disp} ${srcs_test})
# #set(srcs		${srcs_test})
# #set(incs        ${srcs_base} ${srcs_core})

# #include_directories(${incs})
# add_executable(CPD ${srcs})
# target_link_libraries(CPD ${LINK_LIBRARY})

# set(CMAKE_INSTALL_PREFIX 	${PROJECT_SOURCE_DIR})

# set(CPD_BASE        ${CMAKE_INSTALL_PREFIX}/cpd/include/base)
# set(CPD_CORE        ${CMAKE_INSTALL_PREFIX}/cpd/include/core)
# set(CPD_FAST        ${CMAKE_INSTALL_PREFIX}/cpd/include/fast)
# set(CPD_DISP        ${CMAKE_INSTALL_PREFIX}/cpd/include/disp)
# set(CPD_3RDPARTY    ${CMAKE_INSTALL_PREFIX}/cpd/include/3rdparty/figtree)

# set(CPD_LIBRARY     ${CMAKE_INSTALL_PREFIX}/cpd/lib)

# install(FILES ${srcs_base} DESTINATION ${CPD_BASE})
# install(FILES ${srcs_core} DESTINATION ${CPD_CORE})
# install(FILES ${srcs_fast} DESTINATION ${CPD_FAST})
# install(FILES ${srcs_disp} DESTINATION ${CPD_DISP})
# install(FILES ${FIGTREE_INCLUDE_DIR}/figtree.h DESTINATION ${CPD_3RDPARTY})

# set(figtree_libs ${FIGTREE_LIBRARY_DIR}/figtree_debug.lib
                 # ${FIGTREE_LIBRARY_DIR}/figtree_release.lib
                 # ${FIGTREE_LIBRARY_DIR}/figtree_debug.dll
                 # ${FIGTREE_LIBRARY_DIR}/figtree_release.dll)
# install(FILES ${figtree_libs} DESTINATION ${CPD_LIBRARY})
