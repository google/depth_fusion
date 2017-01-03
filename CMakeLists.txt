cmake_minimum_required( VERSION 3.4 )
project( depth_fusion )

set_property( DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    PROPERTY VS_STARTUP_PROJECT depth_fusion
)

# gflags
if( EXISTS "third_party/gflags/CMakeLists.txt" )
    add_subdirectory( "third_party/gflags" )
else()
    find_package( gflags REQUIRED )
endif()

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

# ARToolkit
# TODO: package ARToolkit under third_party.
include_directories( c:/opt/local/include )
link_directories( c:/opt/local/lib )

# libcgt
find_package( libcgt REQUIRED )
# BUG(CMake): target_include_directories doesn't work with CUDA.
# Or maybe CUDA targets just don't get dependent paths correctly.
include_directories( ${libcgt_DIR}/../.. )

file( GLOB_RECURSE HEADERS src/*.h src/*.inl src/*.cuh )
file( GLOB_RECURSE SOURCES_CPP src/*.cpp )
file( GLOB_RECURSE SOURCES_CU src/*.cu )

# TODO: fast math:
# LIST(APPEND CUDA_NVCC_FLAGS --compiler-options -fno-strict-aliasing -lineinfo -use_fast_math -Xptxas -dlcm=cg)
# LIST(APPEND CUDA_NVCC_FLAGS -gencode arch=compute_20,code=sm_20)
cuda_add_executable( depth_fusion ${HEADERS} ${SOURCES_CPP} ${SOURCES_CU} )
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
	# HACK: ARToolkit
    debug ARd
    debug AR2d
    debug ARICPd
    debug ARgsub_lited
    debug ARMultid
    debug ARUtild
    optimized AR
    optimized AR2
    optimized ARgsub_lite
    optimized ARICP
    optimized ARMulti
    optimized ARUtil
)

# TODO: make this build on Linux. It might need -l GL.
#target_link_libraries( depth_fusion GL GLEW::GLEW Qt5::Core Qt5::OpenGL
#    Qt5::Widgets ${OpenCV_LIBS} cgt_core cgt_gl cgt_opencv_interop libpxc )