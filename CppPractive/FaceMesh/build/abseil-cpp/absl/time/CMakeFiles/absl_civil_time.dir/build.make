# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.23

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.23.2/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.23.2/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/3i-21-331/workspace/CppPractive/CppPractive/FaceMesh

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/3i-21-331/workspace/CppPractive/CppPractive/FaceMesh/build

# Include any dependencies generated for this target.
include abseil-cpp/absl/time/CMakeFiles/absl_civil_time.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include abseil-cpp/absl/time/CMakeFiles/absl_civil_time.dir/compiler_depend.make

# Include the progress variables for this target.
include abseil-cpp/absl/time/CMakeFiles/absl_civil_time.dir/progress.make

# Include the compile flags for this target's objects.
include abseil-cpp/absl/time/CMakeFiles/absl_civil_time.dir/flags.make

abseil-cpp/absl/time/CMakeFiles/absl_civil_time.dir/internal/cctz/src/civil_time_detail.cc.o: abseil-cpp/absl/time/CMakeFiles/absl_civil_time.dir/flags.make
abseil-cpp/absl/time/CMakeFiles/absl_civil_time.dir/internal/cctz/src/civil_time_detail.cc.o: ../abseil-cpp/absl/time/internal/cctz/src/civil_time_detail.cc
abseil-cpp/absl/time/CMakeFiles/absl_civil_time.dir/internal/cctz/src/civil_time_detail.cc.o: abseil-cpp/absl/time/CMakeFiles/absl_civil_time.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/3i-21-331/workspace/CppPractive/CppPractive/FaceMesh/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object abseil-cpp/absl/time/CMakeFiles/absl_civil_time.dir/internal/cctz/src/civil_time_detail.cc.o"
	cd /Users/3i-21-331/workspace/CppPractive/CppPractive/FaceMesh/build/abseil-cpp/absl/time && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT abseil-cpp/absl/time/CMakeFiles/absl_civil_time.dir/internal/cctz/src/civil_time_detail.cc.o -MF CMakeFiles/absl_civil_time.dir/internal/cctz/src/civil_time_detail.cc.o.d -o CMakeFiles/absl_civil_time.dir/internal/cctz/src/civil_time_detail.cc.o -c /Users/3i-21-331/workspace/CppPractive/CppPractive/FaceMesh/abseil-cpp/absl/time/internal/cctz/src/civil_time_detail.cc

abseil-cpp/absl/time/CMakeFiles/absl_civil_time.dir/internal/cctz/src/civil_time_detail.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/absl_civil_time.dir/internal/cctz/src/civil_time_detail.cc.i"
	cd /Users/3i-21-331/workspace/CppPractive/CppPractive/FaceMesh/build/abseil-cpp/absl/time && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/3i-21-331/workspace/CppPractive/CppPractive/FaceMesh/abseil-cpp/absl/time/internal/cctz/src/civil_time_detail.cc > CMakeFiles/absl_civil_time.dir/internal/cctz/src/civil_time_detail.cc.i

abseil-cpp/absl/time/CMakeFiles/absl_civil_time.dir/internal/cctz/src/civil_time_detail.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/absl_civil_time.dir/internal/cctz/src/civil_time_detail.cc.s"
	cd /Users/3i-21-331/workspace/CppPractive/CppPractive/FaceMesh/build/abseil-cpp/absl/time && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/3i-21-331/workspace/CppPractive/CppPractive/FaceMesh/abseil-cpp/absl/time/internal/cctz/src/civil_time_detail.cc -o CMakeFiles/absl_civil_time.dir/internal/cctz/src/civil_time_detail.cc.s

# Object files for target absl_civil_time
absl_civil_time_OBJECTS = \
"CMakeFiles/absl_civil_time.dir/internal/cctz/src/civil_time_detail.cc.o"

# External object files for target absl_civil_time
absl_civil_time_EXTERNAL_OBJECTS =

abseil-cpp/absl/time/libabsl_civil_time.a: abseil-cpp/absl/time/CMakeFiles/absl_civil_time.dir/internal/cctz/src/civil_time_detail.cc.o
abseil-cpp/absl/time/libabsl_civil_time.a: abseil-cpp/absl/time/CMakeFiles/absl_civil_time.dir/build.make
abseil-cpp/absl/time/libabsl_civil_time.a: abseil-cpp/absl/time/CMakeFiles/absl_civil_time.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/3i-21-331/workspace/CppPractive/CppPractive/FaceMesh/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libabsl_civil_time.a"
	cd /Users/3i-21-331/workspace/CppPractive/CppPractive/FaceMesh/build/abseil-cpp/absl/time && $(CMAKE_COMMAND) -P CMakeFiles/absl_civil_time.dir/cmake_clean_target.cmake
	cd /Users/3i-21-331/workspace/CppPractive/CppPractive/FaceMesh/build/abseil-cpp/absl/time && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/absl_civil_time.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
abseil-cpp/absl/time/CMakeFiles/absl_civil_time.dir/build: abseil-cpp/absl/time/libabsl_civil_time.a
.PHONY : abseil-cpp/absl/time/CMakeFiles/absl_civil_time.dir/build

abseil-cpp/absl/time/CMakeFiles/absl_civil_time.dir/clean:
	cd /Users/3i-21-331/workspace/CppPractive/CppPractive/FaceMesh/build/abseil-cpp/absl/time && $(CMAKE_COMMAND) -P CMakeFiles/absl_civil_time.dir/cmake_clean.cmake
.PHONY : abseil-cpp/absl/time/CMakeFiles/absl_civil_time.dir/clean

abseil-cpp/absl/time/CMakeFiles/absl_civil_time.dir/depend:
	cd /Users/3i-21-331/workspace/CppPractive/CppPractive/FaceMesh/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/3i-21-331/workspace/CppPractive/CppPractive/FaceMesh /Users/3i-21-331/workspace/CppPractive/CppPractive/FaceMesh/abseil-cpp/absl/time /Users/3i-21-331/workspace/CppPractive/CppPractive/FaceMesh/build /Users/3i-21-331/workspace/CppPractive/CppPractive/FaceMesh/build/abseil-cpp/absl/time /Users/3i-21-331/workspace/CppPractive/CppPractive/FaceMesh/build/abseil-cpp/absl/time/CMakeFiles/absl_civil_time.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : abseil-cpp/absl/time/CMakeFiles/absl_civil_time.dir/depend

