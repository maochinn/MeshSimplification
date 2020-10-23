cmake_minimum_required(VERSION 2.8)

project(MeshSimplification)
set(SRC_DIR ${PROJECT_SOURCE_DIR}/src/)
set(INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include/)
set(LIB_DIR ${PROJECT_SOURCE_DIR}/lib/)

set(SRC_SHADER 
    ${SRC_DIR}shaders/common_lib.glsl
    ${SRC_DIR}shaders/material_lib.glsl
    ${SRC_DIR}shaders/simple.vert
    ${SRC_DIR}shaders/simple.frag
    ${SRC_DIR}shaders/cubeMap.vert
    ${SRC_DIR}shaders/cubeMap.frag)

set(SRC_RENDER_UTILITIES
    ${SRC_DIR}RenderUtilities/BufferObject.h
    ${SRC_DIR}RenderUtilities/Shader.h
    ${SRC_DIR}RenderUtilities/Texture.h
    ${SRC_DIR}RenderUtilities/CubeMap.h
    ${SRC_DIR}RenderUtilities/MeshObject.cpp
    ${SRC_DIR}RenderUtilities/MeshObject.h)

include_directories(${INCLUDE_DIR})
include_directories(${INCLUDE_DIR}eigen3/)
include_directories(${INCLUDE_DIR}glad4.6/include/)
include_directories(${INCLUDE_DIR}glm-0.9.8.5/glm/)
include_directories(${INCLUDE_DIR}OpenMesh/)

add_Definitions("-D_XKEYCHECK_H")
add_Definitions("-D_USE_MATH_DEFINES")

add_executable(MeshSimplification
    ${SRC_DIR}CallBack.h
    ${SRC_DIR}MyView.h
    ${SRC_DIR}MyWindow.h

    ${SRC_DIR}main.cpp
    ${SRC_DIR}MyView.cpp
    ${SRC_DIR}MyWindow.cpp

    ${SRC_SHADER}
    ${SRC_RENDER_UTILITIES}

    ${INCLUDE_DIR}glad4.6/src/glad.c
)
source_group("shaders" FILES ${SRC_SHADER})
source_group("RenderUtilities" FILES ${SRC_RENDER_UTILITIES})


add_library(Utilities 
    ${SRC_DIR}Utilities/ArcBallCam.h
    ${SRC_DIR}Utilities/3DUtils.h
    ${SRC_DIR}Utilities/Pnt3f.h
    ${SRC_DIR}Utilities/ArcBallCam.cpp
    ${SRC_DIR}Utilities/3DUtils.cpp
    ${SRC_DIR}Utilities/Pnt3f.cpp)

target_link_libraries(MeshSimplification 
debug ${LIB_DIR}Debug/fltk_formsd.lib      optimized ${LIB_DIR}Release/fltk_forms.lib
debug ${LIB_DIR}Debug/fltk_gld.lib         optimized ${LIB_DIR}Release/fltk_gl.lib
debug ${LIB_DIR}Debug/fltk_imagesd.lib     optimized ${LIB_DIR}Release/fltk_images.lib
debug ${LIB_DIR}Debug/fltk_jpegd.lib       optimized ${LIB_DIR}Release/fltk_jpeg.lib
debug ${LIB_DIR}Debug/fltk_pngd.lib        optimized ${LIB_DIR}Release/fltk_png.lib
debug ${LIB_DIR}Debug/fltk_zd.lib          optimized ${LIB_DIR}Release/fltk_z.lib
debug ${LIB_DIR}Debug/fltkd.lib            optimized ${LIB_DIR}Release/fltk.lib
debug ${LIB_DIR}Debug/opencv_world341d.lib optimized ${LIB_DIR}Release/opencv_world341.lib
debug ${LIB_DIR}Debug/OpenMeshCored.lib    optimized ${LIB_DIR}Release/OpenMeshCore.lib
debug ${LIB_DIR}Debug/OpenMeshToolsd.lib   optimized ${LIB_DIR}Release/OpenMeshTools.lib)

target_link_libraries(MeshSimplification 
    ${LIB_DIR}OpenGL32.lib
    ${LIB_DIR}glu32.lib)

target_link_libraries(MeshSimplification Utilities)
    