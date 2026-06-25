# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "C:/Users/augie/OneDrive/Bureau/IGR/IGR08/IGR08-InteractiveStylization/final-code/libigl/cmake/../external/libigl-imgui")
  file(MAKE_DIRECTORY "C:/Users/augie/OneDrive/Bureau/IGR/IGR08/IGR08-InteractiveStylization/final-code/libigl/cmake/../external/libigl-imgui")
endif()
file(MAKE_DIRECTORY
  "C:/Users/augie/OneDrive/Bureau/IGR/IGR08/IGR08-InteractiveStylization/final-code/build/libigl-imgui-build"
  "C:/Users/augie/OneDrive/Bureau/IGR/IGR08/IGR08-InteractiveStylization/final-code/libigl/external/.cache/libigl-imgui/libigl-imgui-download-prefix"
  "C:/Users/augie/OneDrive/Bureau/IGR/IGR08/IGR08-InteractiveStylization/final-code/libigl/external/.cache/libigl-imgui/libigl-imgui-download-prefix/tmp"
  "C:/Users/augie/OneDrive/Bureau/IGR/IGR08/IGR08-InteractiveStylization/final-code/libigl/external/.cache/libigl-imgui/libigl-imgui-download-prefix/src/libigl-imgui-download-stamp"
  "C:/Users/augie/OneDrive/Bureau/IGR/IGR08/IGR08-InteractiveStylization/final-code/libigl/external/.cache/libigl-imgui/libigl-imgui-download-prefix/src"
  "C:/Users/augie/OneDrive/Bureau/IGR/IGR08/IGR08-InteractiveStylization/final-code/libigl/external/.cache/libigl-imgui/libigl-imgui-download-prefix/src/libigl-imgui-download-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/augie/OneDrive/Bureau/IGR/IGR08/IGR08-InteractiveStylization/final-code/libigl/external/.cache/libigl-imgui/libigl-imgui-download-prefix/src/libigl-imgui-download-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/augie/OneDrive/Bureau/IGR/IGR08/IGR08-InteractiveStylization/final-code/libigl/external/.cache/libigl-imgui/libigl-imgui-download-prefix/src/libigl-imgui-download-stamp${cfgdir}") # cfgdir has leading slash
endif()
