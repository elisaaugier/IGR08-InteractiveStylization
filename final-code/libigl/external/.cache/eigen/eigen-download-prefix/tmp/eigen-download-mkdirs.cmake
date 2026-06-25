# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "C:/Users/augie/OneDrive/Bureau/IGR/IGR08/IGR08-InteractiveStylization/final-code/libigl/cmake/../external/eigen")
  file(MAKE_DIRECTORY "C:/Users/augie/OneDrive/Bureau/IGR/IGR08/IGR08-InteractiveStylization/final-code/libigl/cmake/../external/eigen")
endif()
file(MAKE_DIRECTORY
  "C:/Users/augie/OneDrive/Bureau/IGR/IGR08/IGR08-InteractiveStylization/final-code/build/eigen-build"
  "C:/Users/augie/OneDrive/Bureau/IGR/IGR08/IGR08-InteractiveStylization/final-code/libigl/external/.cache/eigen/eigen-download-prefix"
  "C:/Users/augie/OneDrive/Bureau/IGR/IGR08/IGR08-InteractiveStylization/final-code/libigl/external/.cache/eigen/eigen-download-prefix/tmp"
  "C:/Users/augie/OneDrive/Bureau/IGR/IGR08/IGR08-InteractiveStylization/final-code/libigl/external/.cache/eigen/eigen-download-prefix/src/eigen-download-stamp"
  "C:/Users/augie/OneDrive/Bureau/IGR/IGR08/IGR08-InteractiveStylization/final-code/libigl/external/.cache/eigen/eigen-download-prefix/src"
  "C:/Users/augie/OneDrive/Bureau/IGR/IGR08/IGR08-InteractiveStylization/final-code/libigl/external/.cache/eigen/eigen-download-prefix/src/eigen-download-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/augie/OneDrive/Bureau/IGR/IGR08/IGR08-InteractiveStylization/final-code/libigl/external/.cache/eigen/eigen-download-prefix/src/eigen-download-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/augie/OneDrive/Bureau/IGR/IGR08/IGR08-InteractiveStylization/final-code/libigl/external/.cache/eigen/eigen-download-prefix/src/eigen-download-stamp${cfgdir}") # cfgdir has leading slash
endif()
