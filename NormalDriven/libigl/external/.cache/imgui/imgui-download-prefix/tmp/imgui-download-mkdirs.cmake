# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "C:/Users/KABLA/OneDrive/Documents/InfoTelecom/IGR/IG08_CubicStylization/my-project/libigl/cmake/../external/imgui")
  file(MAKE_DIRECTORY "C:/Users/KABLA/OneDrive/Documents/InfoTelecom/IGR/IG08_CubicStylization/my-project/libigl/cmake/../external/imgui")
endif()
file(MAKE_DIRECTORY
  "C:/Users/KABLA/OneDrive/Documents/InfoTelecom/IGR/IG08_CubicStylization/my-project/build/imgui-build"
  "C:/Users/KABLA/OneDrive/Documents/InfoTelecom/IGR/IG08_CubicStylization/my-project/libigl/external/.cache/imgui/imgui-download-prefix"
  "C:/Users/KABLA/OneDrive/Documents/InfoTelecom/IGR/IG08_CubicStylization/my-project/libigl/external/.cache/imgui/imgui-download-prefix/tmp"
  "C:/Users/KABLA/OneDrive/Documents/InfoTelecom/IGR/IG08_CubicStylization/my-project/libigl/external/.cache/imgui/imgui-download-prefix/src/imgui-download-stamp"
  "C:/Users/KABLA/OneDrive/Documents/InfoTelecom/IGR/IG08_CubicStylization/my-project/libigl/external/.cache/imgui/imgui-download-prefix/src"
  "C:/Users/KABLA/OneDrive/Documents/InfoTelecom/IGR/IG08_CubicStylization/my-project/libigl/external/.cache/imgui/imgui-download-prefix/src/imgui-download-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/KABLA/OneDrive/Documents/InfoTelecom/IGR/IG08_CubicStylization/my-project/libigl/external/.cache/imgui/imgui-download-prefix/src/imgui-download-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/KABLA/OneDrive/Documents/InfoTelecom/IGR/IG08_CubicStylization/my-project/libigl/external/.cache/imgui/imgui-download-prefix/src/imgui-download-stamp${cfgdir}") # cfgdir has leading slash
endif()
