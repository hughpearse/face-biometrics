# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/cmake-gui

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/hughpear/Desktop/biometrics/face/flandmark-master

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hughpear/Desktop/biometrics/face/flandmark-master/bin

# Include any dependencies generated for this target.
include examples/CMakeFiles/flandmark_simple_example.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/flandmark_simple_example.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/flandmark_simple_example.dir/flags.make

examples/CMakeFiles/flandmark_simple_example.dir/simple_example.cpp.o: examples/CMakeFiles/flandmark_simple_example.dir/flags.make
examples/CMakeFiles/flandmark_simple_example.dir/simple_example.cpp.o: ../examples/simple_example.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/hughpear/Desktop/biometrics/face/flandmark-master/bin/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object examples/CMakeFiles/flandmark_simple_example.dir/simple_example.cpp.o"
	cd /home/hughpear/Desktop/biometrics/face/flandmark-master/bin/examples && /usr/lib64/ccache/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/flandmark_simple_example.dir/simple_example.cpp.o -c /home/hughpear/Desktop/biometrics/face/flandmark-master/examples/simple_example.cpp

examples/CMakeFiles/flandmark_simple_example.dir/simple_example.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/flandmark_simple_example.dir/simple_example.cpp.i"
	cd /home/hughpear/Desktop/biometrics/face/flandmark-master/bin/examples && /usr/lib64/ccache/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/hughpear/Desktop/biometrics/face/flandmark-master/examples/simple_example.cpp > CMakeFiles/flandmark_simple_example.dir/simple_example.cpp.i

examples/CMakeFiles/flandmark_simple_example.dir/simple_example.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/flandmark_simple_example.dir/simple_example.cpp.s"
	cd /home/hughpear/Desktop/biometrics/face/flandmark-master/bin/examples && /usr/lib64/ccache/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/hughpear/Desktop/biometrics/face/flandmark-master/examples/simple_example.cpp -o CMakeFiles/flandmark_simple_example.dir/simple_example.cpp.s

examples/CMakeFiles/flandmark_simple_example.dir/simple_example.cpp.o.requires:
.PHONY : examples/CMakeFiles/flandmark_simple_example.dir/simple_example.cpp.o.requires

examples/CMakeFiles/flandmark_simple_example.dir/simple_example.cpp.o.provides: examples/CMakeFiles/flandmark_simple_example.dir/simple_example.cpp.o.requires
	$(MAKE) -f examples/CMakeFiles/flandmark_simple_example.dir/build.make examples/CMakeFiles/flandmark_simple_example.dir/simple_example.cpp.o.provides.build
.PHONY : examples/CMakeFiles/flandmark_simple_example.dir/simple_example.cpp.o.provides

examples/CMakeFiles/flandmark_simple_example.dir/simple_example.cpp.o.provides.build: examples/CMakeFiles/flandmark_simple_example.dir/simple_example.cpp.o

# Object files for target flandmark_simple_example
flandmark_simple_example_OBJECTS = \
"CMakeFiles/flandmark_simple_example.dir/simple_example.cpp.o"

# External object files for target flandmark_simple_example
flandmark_simple_example_EXTERNAL_OBJECTS =

examples/flandmark_simple_example: examples/CMakeFiles/flandmark_simple_example.dir/simple_example.cpp.o
examples/flandmark_simple_example: examples/CMakeFiles/flandmark_simple_example.dir/build.make
examples/flandmark_simple_example: libflandmark/libflandmark_static.a
examples/flandmark_simple_example: examples/CMakeFiles/flandmark_simple_example.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable flandmark_simple_example"
	cd /home/hughpear/Desktop/biometrics/face/flandmark-master/bin/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/flandmark_simple_example.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/flandmark_simple_example.dir/build: examples/flandmark_simple_example
.PHONY : examples/CMakeFiles/flandmark_simple_example.dir/build

examples/CMakeFiles/flandmark_simple_example.dir/requires: examples/CMakeFiles/flandmark_simple_example.dir/simple_example.cpp.o.requires
.PHONY : examples/CMakeFiles/flandmark_simple_example.dir/requires

examples/CMakeFiles/flandmark_simple_example.dir/clean:
	cd /home/hughpear/Desktop/biometrics/face/flandmark-master/bin/examples && $(CMAKE_COMMAND) -P CMakeFiles/flandmark_simple_example.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/flandmark_simple_example.dir/clean

examples/CMakeFiles/flandmark_simple_example.dir/depend:
	cd /home/hughpear/Desktop/biometrics/face/flandmark-master/bin && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hughpear/Desktop/biometrics/face/flandmark-master /home/hughpear/Desktop/biometrics/face/flandmark-master/examples /home/hughpear/Desktop/biometrics/face/flandmark-master/bin /home/hughpear/Desktop/biometrics/face/flandmark-master/bin/examples /home/hughpear/Desktop/biometrics/face/flandmark-master/bin/examples/CMakeFiles/flandmark_simple_example.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/flandmark_simple_example.dir/depend
