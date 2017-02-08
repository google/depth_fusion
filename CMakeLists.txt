cmake_minimum_required( VERSION 3.4 )
project( depth_fusion )

set_property( DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    PROPERTY VS_STARTUP_PROJECT depth_fusion
)

# gflags
add_subdirectory( "third_party/gflags" )

# OpenCV
find_package( OpenCV REQUIRED )

# OpenGL and GLEW.
# GLEW needs to come before CUDA's samples
find_package( OpenGL REQUIRED )
find_package( GLEW REQUIRED )

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
find_package( libcgt REQUIRED )
# BUG(CMake): target_include_directories doesn't work with CUDA.
# Or maybe CUDA targets just don't get dependent paths correctly.
include_directories( ${libcgt_DIR}/../.. )

file( GLOB_RECURSE SOURCES_CPP src/*.cpp )
file( GLOB_RECURSE SOURCES_CU src/*.cu )

# CUDA flags
set( CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -lineinfo -use_fast_math )
set( CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-gencode arch=compute_60,code=sm_60" )

# TODO: Look into -Xptxas -dlcm=cg
# TODO: Look into gcc -f no-strict-aliasing

# depth_fusion executable
set( DEPTH_FUSION_HEADERS
    src/aruco/aruco_pose_estimator.h
    src/aruco/cube_fiducial.h
    src/calibrated_posed_depth_camera.h
    src/control_widget.h
    src/depth_processor.h
    src/fuse.h
    src/icp_least_squares_data.h
    src/input_buffer.h
    src/main_controller.h
    src/main_widget.h
    src/marching_cubes.h
    src/multi_static_camera_gl_state.h
    src/multi_static_camera_pipeline.h
    src/pipeline_data_type.h
    src/pose_frame.h
    src/pose_utils.h
    src/projective_point_plane_icp.h
    src/raycast.h
    src/regular_grid_fusion_pipeline.h
    src/regular_grid_tsdf.h
    src/rgbd_camera_parameters.h
    src/rgbd_input.h
    src/single_moving_camera_gl_state.h
    src/tsdf.h
)

set( DEPTH_FUSION_SOURCES_CPP
    src/depth_fusion.cpp
    src/aruco/aruco_pose_estimator.cpp
    src/aruco/cube_fiducial.cpp
    src/control_widget.cpp
    src/draw_camera_path.cpp
    src/icp_least_squares_data.cpp
    src/input_buffer.cpp
    src/main_controller.cpp
    src/main_widget.cpp
    src/marching_cubes.cpp
    src/multi_static_camera_gl_state.cpp
    src/multi_static_camera_pipeline.cpp
    src/pose_utils.cpp
    src/regular_grid_fusion_pipeline.cpp
    src/rgbd_camera_parameters.cpp
    src/rgbd_input.cpp
    src/single_moving_camera_gl_state.cpp
)

set( DEPTH_FUSION_SOURCES_CU
    src/depth_processor.cu
    src/fuse.cu
    src/projective_point_plane_icp.cu
    src/raycast.cu
    src/regular_grid_tsdf.cu
)

cuda_add_executable( depth_fusion
    ${DEPTH_FUSION_HEADERS}
    ${DEPTH_FUSION_SOURCES_CPP}
    ${DEPTH_FUSION_SOURCES_CU}
)
set_property( TARGET depth_fusion PROPERTY CXX_STANDARD 11 )
target_compile_definitions( depth_fusion
    PRIVATE GL_PLATFORM_45 _USE_MATH_DEFINES )
target_include_directories( depth_fusion PRIVATE . )
target_link_libraries( depth_fusion
    gflags
    opengl32 GLEW::GLEW
    ${CUDA_LIBRARIES}
    Qt5::Core Qt5::OpenGL Qt5::Widgets
    ${OpenCV_LIBS}
	cgt_core
    cgt_cuda
    cgt_gl
    cgt_camera_wrappers
    cgt_opencv_interop
    cgt_qt_interop	
)

# rgbd2pose executable
add_executable( rgbd2pose
    src/rgbd2pose.cpp
	src/aruco/aruco_pose_estimator.h
	src/aruco/aruco_pose_estimator.cpp
	src/aruco/cube_fiducial.h
	src/aruco/cube_fiducial.cpp
	src/input_buffer.h
	src/input_buffer.cpp
	src/rgbd_camera_parameters.h
	src/rgbd_camera_parameters.cpp
	src/rgbd_input.h
	src/rgbd_input.cpp
)
target_include_directories( rgbd2pose PRIVATE . )
target_link_libraries( rgbd2pose
    gflags
    opengl32 GLEW::GLEW
    Qt5::Core Qt5::OpenGL Qt5::Widgets
    ${OpenCV_LIBS}
	cgt_core
    cgt_cuda
    cgt_gl
    cgt_camera_wrappers
    cgt_opencv_interop
    cgt_qt_interop	
)

# TODO: make this build on Linux. It might need -l GL.
#target_link_libraries( depth_fusion GL GLEW::GLEW Qt5::Core Qt5::OpenGL
#    Qt5::Widgets ${OpenCV_LIBS} cgt_core cgt_gl cgt_opencv_interop libpxc )