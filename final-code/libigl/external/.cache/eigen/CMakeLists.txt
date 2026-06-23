# Distributed under the OSI-approved MIT License.  See accompanying
# file LICENSE or https://github.com/Crascit/DownloadProject for details.

cmake_minimum_required(VERSION 3.10)

project(eigen-download NONE)

include(ExternalProject)
ExternalProject_Add(eigen-download
                    TLS_VERIFY;OFF;GIT_CONFIG;advice.detachedHead=false;GIT_REPOSITORY;https://github.com/eigenteam/eigen-git-mirror.git;GIT_TAG;3.2.10
                    SOURCE_DIR          "C:/Users/KABLA/OneDrive/Documents/InfoTelecom/IGR/IG08_CubicStylization/my-project/libigl/cmake/../external/eigen"
                    BINARY_DIR          "C:/Users/KABLA/OneDrive/Documents/InfoTelecom/IGR/IG08_CubicStylization/my-project/build/eigen-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
)
