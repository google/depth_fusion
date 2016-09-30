cmake_minimum_required( VERSION 3.4 )
project( depth_fusion )

# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# GLEW
# GLEW needs to come before CUDA's samples
find_package( OpenGL REQUIRED )
find_package( GLEW REQUIRED )
include_directories( ${GLEW_INCLUDE_DIRS} )

# CUDA
find_package( CUDA REQUIRED )

#set( CUDA_SEPARABLE_COMPILATION ON )
include_directories( ${CUDA_TOOLKIT_ROOT_DIR}/include )

# Define CUDA_SAMPLES_ROOT:
if( WIN32 )
    if( DEFINED ENV{NVCUDASAMPLES_ROOT} )
        set( CUDA_SAMPLES_ROOT $ENV{NVCUDASAMPLES_ROOT} )
    else()
        message( "Environment variable NVCUDASAMPLES_ROOT must be set." )
        return()
    endif()
    # Apparently, leaving this on interferes with parallel builds.
    set( CUDA_ATTACH_VS_BUILD_RULE_TO_CUDA_FILE OFF )
else()
    set( CUDA_SAMPLES_ROOT ${CUDA_TOOLKIT_ROOT_DIR}/samples )
endif()
include_directories( ${CUDA_SAMPLES_ROOT}/common/inc )

# Qt 5
set( CMAKE_AUTOMOC ON )
find_package( Qt5Core )
find_package( Qt5OpenGL )
find_package( Qt5Widgets )

# libcgt
include_directories( ../libcgt/build/lib/include )
include_directories( ../libcgt/build/lib/include/core )
include_directories( ../libcgt/build/lib/include/GL/common )
include_directories( ../libcgt/build/lib/include/GL/GL_45 )
link_directories( ../libcgt/build/lib/lib )

# third_party
include_directories( third_party )

file( GLOB_RECURSE HEADERS src/*.h src/*.inl src/*.cuh )
file( GLOB_RECURSE SOURCES_CPP src/*.cpp )
file( GLOB_RECURSE SOURCES_CU src/*.cu )

CUDA_ADD_EXECUTABLE( depth_fusion ${HEADERS} ${SOURCES_CPP} ${SOURCES_CU} )
set_property( TARGET depth_fusion PROPERTY CXX_STANDARD 11 )

# TODO: use import and export library
target_link_libraries( depth_fusion
	opengl32 GLEW::GLEW
	${CUDA_LIBRARIES}
	Qt5::Core Qt5::OpenGL Qt5::Widgets
	${OpenCV_LIBS}
	debug cgt_cored
	debug cgt_cudad
	debug cgt_gld
	debug cgt_camera_wrappersd
	debug cgt_opencv_interopd
	debug cgt_qt_interopd
	optimized cgt_core
	optimized cgt_cuda
	optimized cgt_gl
	optimized cgt_camera_wrappers
	optimized cgt_opencv_interop
	optimized cgt_qt_interop )

# TODO: make this build on Linux. It might need -l GL.
#target_link_libraries( depth_fusion GL GLEW::GLEW Qt5::Core Qt5::OpenGL
#    Qt5::Widgets ${OpenCV_LIBS} cgt_core cgt_gl cgt_opencv_interop libpxc )