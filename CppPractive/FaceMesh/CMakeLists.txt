cmake_minimum_required(VERSION 3.8.2)
set(LibraryName "my_lib_project")
project(${LibraryName} VERSION 1.0)


# Leave C++ standard up to the root application, so set it only if this is the
# current top-level CMake project.
if(CMAKE_SOURCE_DIR STREQUAL ${LibraryName}_SOURCE_DIR)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

add_subdirectory(abseil-cpp)


#add_library(my_lib main.cpp)
file(GLOB SOURCES "*.cc")
file(GLOB SOURCES main.cpp)
add_executable(my_lib ${SOURCES})

target_link_libraries(my_lib absl::base absl::synchronization absl::strings absl::memory)

target_compile_features(my_lib PUBLIC cxx_std_17)
if(CMAKE_CXX_STANDARD LESS 17)
  message(FATAL_ERROR
      "my_lib_project requires CMAKE_CXX_STANDARD >= 17 (got: ${CMAKE_CXX_STANDARD})")
endif()