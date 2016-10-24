cmake_minimum_required(VERSION 3.6)

project(artoolkit_sample)

set (COMPILER_FLAGS "-WX -wd4800 -FAs")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COMPILER_FLAGS}")
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${COMPILER_FLAGS}")
add_definitions(-D_CRT_SECURE_NO_WARNINGS)
add_definitions(-D_SCL_SECURE_NO_WARNINGS)
add_definitions(-DGLEW_STATIC)

set(SRCDIR "./")
set(TPDIR "../third_party/")

list (APPEND AR_INCLUDES
  ${SRCDIR}
  ${TPDIR}/glew/include
  ${TPDIR}
  ${TPDIR}/artoolkit/include
)
list (APPEND LIBS artoolkit 
mfuuid mfreadwrite mf mfplat # media foundation
opengl32
)

list (APPEND AR_SRC
  ${SRCDIR}lum_gl.cpp
  ${SRCDIR}lum_glprogram2.cpp
  ${SRCDIR}lum_glwindow.cpp
  ${SRCDIR}lum_win32tools.cpp
  ${SRCDIR}lum_mediasource.cpp
  ${SRCDIR}lum_quadrenderer.cpp
  ${SRCDIR}lum_transform.cpp
  ${SRCDIR}lum_unitquad.cpp
  ${SRCDIR}lum_util_gl.cpp
  ${SRCDIR}lum_vertexrecorder.cpp
  ${TPDIR}/glew/src/glew.c
)
list (APPEND AR_HEADER
  ${SRCDIR}lum_gl.h
  ${SRCDIR}lum_constants.h
  ${SRCDIR}lum_glprogram2.h
  ${SRCDIR}lum_glwindow.h
  ${SRCDIR}lum_win32tools.h
  ${SRCDIR}lum_mediasource.h
  ${SRCDIR}lum_quadrenderer.h
  ${SRCDIR}lum_transform.h
  ${SRCDIR}lum_unitquad.h
  ${SRCDIR}lum_util_gl.h
  ${SRCDIR}lum_vertexrecorder.h
)
set(AR_SHADER
  fshader.glsl
  vshader.glsl)
source_group(shader FILES ${AR_SHADER})
add_executable(arsimple simpleTest.cpp ${AR_SRC} ${AR_HEADER} ${AR_SHADER})
target_include_directories(arsimple PUBLIC ${AR_INCLUDES})
target_link_libraries(arsimple ${LIBS})

set(AR_SHADER
  fshader.glsl
  vshader.glsl
  vsprojection.glsl
  fsprojection.glsl)
source_group(shader FILES ${AR_SHADER})
add_executable(arprojection main_arprojection.cpp ${AR_SRC} ${AR_HEADER} ${AR_SHADER})
target_include_directories(arprojection PUBLIC ${AR_INCLUDES})
target_link_libraries(arprojection ${LIBS})

add_subdirectory(${TPDIR}artoolkit artoolkit)

