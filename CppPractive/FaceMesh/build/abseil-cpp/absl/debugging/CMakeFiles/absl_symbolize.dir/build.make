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
include abseil-cpp/absl/debugging/CMakeFiles/absl_symbolize.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include abseil-cpp/absl/debugging/CMakeFiles/absl_symbolize.dir/compiler_depend.make

# Include the progress variables for this target.
include abseil-cpp/absl/debugging/CMakeFiles/absl_symbolize.dir/progress.make

# Include the compile flags for this target's objects.
include abseil-cpp/absl/debugging/CMakeFiles/absl_symbolize.dir/flags.make

abseil-cpp/absl/debugging/CMakeFiles/absl_symbolize.dir/symbolize.cc.o: abseil-cpp/absl/debugging/CMakeFiles/absl_symbolize.dir/flags.make
abseil-cpp/absl/debugging/CMakeFiles/absl_symbolize.dir/symbolize.cc.o: ../abseil-cpp/absl/debugging/symbolize.cc
abseil-cpp/absl/debugging/CMakeFiles/absl_symbolize.dir/symbolize.cc.o: abseil-cpp/absl/debugging/CMakeFiles/absl_symbolize.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/3i-21-331/workspace/CppPractive/CppPractive/FaceMesh/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object abseil-cpp/absl/debugging/CMakeFiles/absl_symbolize.dir/symbolize.cc.o"
	cd /Users/3i-21-331/workspace/CppPractive/CppPractive/FaceMesh/build/abseil-cpp/absl/debugging && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT abseil-cpp/absl/debugging/CMakeFiles/absl_symbolize.dir/symbolize.cc.o -MF CMakeFiles/absl_symbolize.dir/symbolize.cc.o.d -o CMakeFiles/absl_symbolize.dir/symbolize.cc.o -c /Users/3i-21-331/workspace/CppPractive/CppPractive/FaceMesh/abseil-cpp/absl/debugging/symbolize.cc

abseil-cpp/absl/debugging/CMakeFiles/absl_symbolize.dir/symbolize.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/absl_symbolize.dir/symbolize.cc.i"
	cd /Users/3i-21-331/workspace/CppPractive/CppPractive/FaceMesh/build/abseil-cpp/absl/debugging && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/3i-21-331/workspace/CppPractive/CppPractive/FaceMesh/abseil-cpp/absl/debugging/symbolize.cc > CMakeFiles/absl_symbolize.dir/symbolize.cc.i

abseil-cpp/absl/debugging/CMakeFiles/absl_symbolize.dir/symbolize.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/absl_symbolize.dir/symbolize.cc.s"
	cd /Users/3i-21-331/workspace/CppPractive/CppPractive/FaceMesh/build/abseil-cpp/absl/debugging && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/3i-21-331/workspace/CppPractive/CppPractive/FaceMesh/abseil-cpp/absl/debugging/symbolize.cc -o CMakeFiles/absl_symbolize.dir/symbolize.cc.s

# Object files for target absl_symbolize
absl_symbolize_OBJECTS = \
"CMakeFiles/absl_symbolize.dir/symbolize.cc.o"

# External object files for target absl_symbolize
absl_symbolize_EXTERNAL_OBJECTS =

abseil-cpp/absl/debugging/libabsl_symbolize.a: abseil-cpp/absl/debugging/CMakeFiles/absl_symbolize.dir/symbolize.cc.o
abseil-cpp/absl/debugging/libabsl_symbolize.a: abseil-cpp/absl/debugging/CMakeFiles/absl_symbolize.dir/build.make
abseil-cpp/absl/debugging/libabsl_symbolize.a: abseil-cpp/absl/debugging/CMakeFiles/absl_symbolize.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/3i-21-331/workspace/CppPractive/CppPractive/FaceMesh/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libabsl_symbolize.a"
	cd /Users/3i-21-331/workspace/CppPractive/CppPractive/FaceMesh/build/abseil-cpp/absl/debugging && $(CMAKE_COMMAND) -P CMakeFiles/absl_symbolize.dir/cmake_clean_target.cmake
	cd /Users/3i-21-331/workspace/CppPractive/CppPractive/FaceMesh/build/abseil-cpp/absl/debugging && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/absl_symbolize.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
abseil-cpp/absl/debugging/CMakeFiles/absl_symbolize.dir/build: abseil-cpp/absl/debugging/libabsl_symbolize.a
.PHONY : abseil-cpp/absl/debugging/CMakeFiles/absl_symbolize.dir/build

abseil-cpp/absl/debugging/CMakeFiles/absl_symbolize.dir/clean:
	cd /Users/3i-21-331/workspace/CppPractive/CppPractive/FaceMesh/build/abseil-cpp/absl/debugging && $(CMAKE_COMMAND) -P CMakeFiles/absl_symbolize.dir/cmake_clean.cmake
.PHONY : abseil-cpp/absl/debugging/CMakeFiles/absl_symbolize.dir/clean

abseil-cpp/absl/debugging/CMakeFiles/absl_symbolize.dir/depend:
	cd /Users/3i-21-331/workspace/CppPractive/CppPractive/FaceMesh/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/3i-21-331/workspace/CppPractive/CppPractive/FaceMesh /Users/3i-21-331/workspace/CppPractive/CppPractive/FaceMesh/abseil-cpp/absl/debugging /Users/3i-21-331/workspace/CppPractive/CppPractive/FaceMesh/build /Users/3i-21-331/workspace/CppPractive/CppPractive/FaceMesh/build/abseil-cpp/absl/debugging /Users/3i-21-331/workspace/CppPractive/CppPractive/FaceMesh/build/abseil-cpp/absl/debugging/CMakeFiles/absl_symbolize.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : abseil-cpp/absl/debugging/CMakeFiles/absl_symbolize.dir/depend

