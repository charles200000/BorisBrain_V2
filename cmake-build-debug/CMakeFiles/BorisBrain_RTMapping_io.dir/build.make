# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Produce verbose output by default.
VERBOSE = 1

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
CMAKE_COMMAND = /home/hades/ProgramFiles/clion/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/hades/ProgramFiles/clion/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/hades/Projects/BorisBrain_v2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hades/Projects/BorisBrain_v2/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/BorisBrain_RTMapping_io.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/BorisBrain_RTMapping_io.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/BorisBrain_RTMapping_io.dir/flags.make

CMakeFiles/BorisBrain_RTMapping_io.dir/src/io/aerial-mapper-io.cc.o: CMakeFiles/BorisBrain_RTMapping_io.dir/flags.make
CMakeFiles/BorisBrain_RTMapping_io.dir/src/io/aerial-mapper-io.cc.o: ../src/io/aerial-mapper-io.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hades/Projects/BorisBrain_v2/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/BorisBrain_RTMapping_io.dir/src/io/aerial-mapper-io.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/BorisBrain_RTMapping_io.dir/src/io/aerial-mapper-io.cc.o -c /home/hades/Projects/BorisBrain_v2/src/io/aerial-mapper-io.cc

CMakeFiles/BorisBrain_RTMapping_io.dir/src/io/aerial-mapper-io.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/BorisBrain_RTMapping_io.dir/src/io/aerial-mapper-io.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hades/Projects/BorisBrain_v2/src/io/aerial-mapper-io.cc > CMakeFiles/BorisBrain_RTMapping_io.dir/src/io/aerial-mapper-io.cc.i

CMakeFiles/BorisBrain_RTMapping_io.dir/src/io/aerial-mapper-io.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/BorisBrain_RTMapping_io.dir/src/io/aerial-mapper-io.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hades/Projects/BorisBrain_v2/src/io/aerial-mapper-io.cc -o CMakeFiles/BorisBrain_RTMapping_io.dir/src/io/aerial-mapper-io.cc.s

# Object files for target BorisBrain_RTMapping_io
BorisBrain_RTMapping_io_OBJECTS = \
"CMakeFiles/BorisBrain_RTMapping_io.dir/src/io/aerial-mapper-io.cc.o"

# External object files for target BorisBrain_RTMapping_io
BorisBrain_RTMapping_io_EXTERNAL_OBJECTS =

../lib/libBorisBrain_RTMapping_io.a: CMakeFiles/BorisBrain_RTMapping_io.dir/src/io/aerial-mapper-io.cc.o
../lib/libBorisBrain_RTMapping_io.a: CMakeFiles/BorisBrain_RTMapping_io.dir/build.make
../lib/libBorisBrain_RTMapping_io.a: CMakeFiles/BorisBrain_RTMapping_io.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hades/Projects/BorisBrain_v2/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library ../lib/libBorisBrain_RTMapping_io.a"
	$(CMAKE_COMMAND) -P CMakeFiles/BorisBrain_RTMapping_io.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/BorisBrain_RTMapping_io.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/BorisBrain_RTMapping_io.dir/build: ../lib/libBorisBrain_RTMapping_io.a

.PHONY : CMakeFiles/BorisBrain_RTMapping_io.dir/build

CMakeFiles/BorisBrain_RTMapping_io.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/BorisBrain_RTMapping_io.dir/cmake_clean.cmake
.PHONY : CMakeFiles/BorisBrain_RTMapping_io.dir/clean

CMakeFiles/BorisBrain_RTMapping_io.dir/depend:
	cd /home/hades/Projects/BorisBrain_v2/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hades/Projects/BorisBrain_v2 /home/hades/Projects/BorisBrain_v2 /home/hades/Projects/BorisBrain_v2/cmake-build-debug /home/hades/Projects/BorisBrain_v2/cmake-build-debug /home/hades/Projects/BorisBrain_v2/cmake-build-debug/CMakeFiles/BorisBrain_RTMapping_io.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/BorisBrain_RTMapping_io.dir/depend
