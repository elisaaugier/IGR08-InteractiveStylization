# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

if(EXISTS "C:/Users/augie/OneDrive/Bureau/IGR/IGR08/IGR08-InteractiveStylization/Sketch2Mesh/libigl/external/.cache/imgui/imgui-download-prefix/src/imgui-download-stamp/imgui-download-gitclone-lastrun.txt" AND EXISTS "C:/Users/augie/OneDrive/Bureau/IGR/IGR08/IGR08-InteractiveStylization/Sketch2Mesh/libigl/external/.cache/imgui/imgui-download-prefix/src/imgui-download-stamp/imgui-download-gitinfo.txt" AND
  "C:/Users/augie/OneDrive/Bureau/IGR/IGR08/IGR08-InteractiveStylization/Sketch2Mesh/libigl/external/.cache/imgui/imgui-download-prefix/src/imgui-download-stamp/imgui-download-gitclone-lastrun.txt" IS_NEWER_THAN "C:/Users/augie/OneDrive/Bureau/IGR/IGR08/IGR08-InteractiveStylization/Sketch2Mesh/libigl/external/.cache/imgui/imgui-download-prefix/src/imgui-download-stamp/imgui-download-gitinfo.txt")
  message(VERBOSE
    "Avoiding repeated git clone, stamp file is up to date: "
    "'C:/Users/augie/OneDrive/Bureau/IGR/IGR08/IGR08-InteractiveStylization/Sketch2Mesh/libigl/external/.cache/imgui/imgui-download-prefix/src/imgui-download-stamp/imgui-download-gitclone-lastrun.txt'"
  )
  return()
endif()

# Even at VERBOSE level, we don't want to see the commands executed, but
# enabling them to be shown for DEBUG may be useful to help diagnose problems.
cmake_language(GET_MESSAGE_LOG_LEVEL active_log_level)
if(active_log_level MATCHES "DEBUG|TRACE")
  set(maybe_show_command COMMAND_ECHO STDOUT)
else()
  set(maybe_show_command "")
endif()

execute_process(
  COMMAND ${CMAKE_COMMAND} -E rm -rf "C:/Users/augie/OneDrive/Bureau/IGR/IGR08/IGR08-InteractiveStylization/Sketch2Mesh/libigl/cmake/../external/imgui"
  RESULT_VARIABLE error_code
  ${maybe_show_command}
)
if(error_code)
  message(FATAL_ERROR "Failed to remove directory: 'C:/Users/augie/OneDrive/Bureau/IGR/IGR08/IGR08-InteractiveStylization/Sketch2Mesh/libigl/cmake/../external/imgui'")
endif()

# try the clone 3 times in case there is an odd git clone issue
set(error_code 1)
set(number_of_tries 0)
while(error_code AND number_of_tries LESS 3)
  execute_process(
    COMMAND "C:/Program Files/Git/cmd/git.exe"
            clone --no-checkout --config "advice.detachedHead=false" --config "advice.detachedHead=false" -c http.sslVerify=false "https://github.com/ocornut/imgui.git" "imgui"
    WORKING_DIRECTORY "C:/Users/augie/OneDrive/Bureau/IGR/IGR08/IGR08-InteractiveStylization/Sketch2Mesh/libigl/cmake/../external"
    RESULT_VARIABLE error_code
    ${maybe_show_command}
  )
  math(EXPR number_of_tries "${number_of_tries} + 1")
endwhile()
if(number_of_tries GREATER 1)
  message(NOTICE "Had to git clone more than once: ${number_of_tries} times.")
endif()
if(error_code)
  message(FATAL_ERROR "Failed to clone repository: 'https://github.com/ocornut/imgui.git'")
endif()

execute_process(
  COMMAND "C:/Program Files/Git/cmd/git.exe"
          checkout "v1.69" --
  WORKING_DIRECTORY "C:/Users/augie/OneDrive/Bureau/IGR/IGR08/IGR08-InteractiveStylization/Sketch2Mesh/libigl/cmake/../external/imgui"
  RESULT_VARIABLE error_code
  ${maybe_show_command}
)
if(error_code)
  message(FATAL_ERROR "Failed to checkout tag: 'v1.69'")
endif()

set(init_submodules TRUE)
if(init_submodules)
  execute_process(
    COMMAND "C:/Program Files/Git/cmd/git.exe" -c;http.sslVerify=false
            submodule update --recursive --init 
    WORKING_DIRECTORY "C:/Users/augie/OneDrive/Bureau/IGR/IGR08/IGR08-InteractiveStylization/Sketch2Mesh/libigl/cmake/../external/imgui"
    RESULT_VARIABLE error_code
    ${maybe_show_command}
  )
endif()
if(error_code)
  message(FATAL_ERROR "Failed to update submodules in: 'C:/Users/augie/OneDrive/Bureau/IGR/IGR08/IGR08-InteractiveStylization/Sketch2Mesh/libigl/cmake/../external/imgui'")
endif()

# Complete success, update the script-last-run stamp file:
#
execute_process(
  COMMAND ${CMAKE_COMMAND} -E copy "C:/Users/augie/OneDrive/Bureau/IGR/IGR08/IGR08-InteractiveStylization/Sketch2Mesh/libigl/external/.cache/imgui/imgui-download-prefix/src/imgui-download-stamp/imgui-download-gitinfo.txt" "C:/Users/augie/OneDrive/Bureau/IGR/IGR08/IGR08-InteractiveStylization/Sketch2Mesh/libigl/external/.cache/imgui/imgui-download-prefix/src/imgui-download-stamp/imgui-download-gitclone-lastrun.txt"
  RESULT_VARIABLE error_code
  ${maybe_show_command}
)
if(error_code)
  message(FATAL_ERROR "Failed to copy script-last-run stamp file: 'C:/Users/augie/OneDrive/Bureau/IGR/IGR08/IGR08-InteractiveStylization/Sketch2Mesh/libigl/external/.cache/imgui/imgui-download-prefix/src/imgui-download-stamp/imgui-download-gitclone-lastrun.txt'")
endif()
