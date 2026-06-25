cmake_minimum_required(VERSION 3.10)
SET(CMAKE_BUILD_TYPE release)
project(normalDriven)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# libigl
option(LIBIGL_WITH_OPENGL            "Use OpenGL"         ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           ON)
option(LIBIGL_WITH_VIEWER            "Use OpenGL viewer"  ON)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use ImGui"          ON)

find_package(LIBIGL REQUIRED)

# Add your project files
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/utils")

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()

file(GLOB SRCFILES *.cpp utils/*.cpp)
add_executable(app ${SRCFILES})
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/libigl/external/imgui)

target_link_libraries(app igl::core igl::opengl igl::opengl_glfw igl::opengl_glfw_imgui)



