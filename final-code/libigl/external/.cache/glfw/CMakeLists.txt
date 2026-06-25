# Distributed under the OSI-approved MIT License.  See accompanying
# file LICENSE or https://github.com/Crascit/DownloadProject for details.

cmake_minimum_required(VERSION 3.10)

project(glfw-download NONE)

include(ExternalProject)
ExternalProject_Add(glfw-download
                    TLS_VERIFY;OFF;GIT_CONFIG;advice.detachedHead=false;GIT_REPOSITORY;https://github.com/glfw/glfw.git;GIT_TAG;3.3
                    SOURCE_DIR          "C:/Users/augie/OneDrive/Bureau/IGR/IGR08/IGR08-InteractiveStylization/final-code/libigl/cmake/../external/glfw"
                    BINARY_DIR          "C:/Users/augie/OneDrive/Bureau/IGR/IGR08/IGR08-InteractiveStylization/final-code/build/glfw-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
)
