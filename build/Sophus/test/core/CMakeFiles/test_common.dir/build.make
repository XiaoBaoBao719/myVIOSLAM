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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/xbbao/SLAM_book_ws/src/myVIOSLAM/Thirdparty/Sophus

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/xbbao/SLAM_book_ws/src/myVIOSLAM/build/Sophus

# Include any dependencies generated for this target.
include test/core/CMakeFiles/test_common.dir/depend.make

# Include the progress variables for this target.
include test/core/CMakeFiles/test_common.dir/progress.make

# Include the compile flags for this target's objects.
include test/core/CMakeFiles/test_common.dir/flags.make

test/core/CMakeFiles/test_common.dir/test_common.cpp.o: test/core/CMakeFiles/test_common.dir/flags.make
test/core/CMakeFiles/test_common.dir/test_common.cpp.o: /home/xbbao/SLAM_book_ws/src/myVIOSLAM/Thirdparty/Sophus/test/core/test_common.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/xbbao/SLAM_book_ws/src/myVIOSLAM/build/Sophus/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/core/CMakeFiles/test_common.dir/test_common.cpp.o"
	cd /home/xbbao/SLAM_book_ws/src/myVIOSLAM/build/Sophus/test/core && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_common.dir/test_common.cpp.o -c /home/xbbao/SLAM_book_ws/src/myVIOSLAM/Thirdparty/Sophus/test/core/test_common.cpp

test/core/CMakeFiles/test_common.dir/test_common.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_common.dir/test_common.cpp.i"
	cd /home/xbbao/SLAM_book_ws/src/myVIOSLAM/build/Sophus/test/core && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/xbbao/SLAM_book_ws/src/myVIOSLAM/Thirdparty/Sophus/test/core/test_common.cpp > CMakeFiles/test_common.dir/test_common.cpp.i

test/core/CMakeFiles/test_common.dir/test_common.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_common.dir/test_common.cpp.s"
	cd /home/xbbao/SLAM_book_ws/src/myVIOSLAM/build/Sophus/test/core && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/xbbao/SLAM_book_ws/src/myVIOSLAM/Thirdparty/Sophus/test/core/test_common.cpp -o CMakeFiles/test_common.dir/test_common.cpp.s

# Object files for target test_common
test_common_OBJECTS = \
"CMakeFiles/test_common.dir/test_common.cpp.o"

# External object files for target test_common
test_common_EXTERNAL_OBJECTS =

test/core/test_common: test/core/CMakeFiles/test_common.dir/test_common.cpp.o
test/core/test_common: test/core/CMakeFiles/test_common.dir/build.make
test/core/test_common: test/core/CMakeFiles/test_common.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/xbbao/SLAM_book_ws/src/myVIOSLAM/build/Sophus/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable test_common"
	cd /home/xbbao/SLAM_book_ws/src/myVIOSLAM/build/Sophus/test/core && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_common.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/core/CMakeFiles/test_common.dir/build: test/core/test_common

.PHONY : test/core/CMakeFiles/test_common.dir/build

test/core/CMakeFiles/test_common.dir/clean:
	cd /home/xbbao/SLAM_book_ws/src/myVIOSLAM/build/Sophus/test/core && $(CMAKE_COMMAND) -P CMakeFiles/test_common.dir/cmake_clean.cmake
.PHONY : test/core/CMakeFiles/test_common.dir/clean

test/core/CMakeFiles/test_common.dir/depend:
	cd /home/xbbao/SLAM_book_ws/src/myVIOSLAM/build/Sophus && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/xbbao/SLAM_book_ws/src/myVIOSLAM/Thirdparty/Sophus /home/xbbao/SLAM_book_ws/src/myVIOSLAM/Thirdparty/Sophus/test/core /home/xbbao/SLAM_book_ws/src/myVIOSLAM/build/Sophus /home/xbbao/SLAM_book_ws/src/myVIOSLAM/build/Sophus/test/core /home/xbbao/SLAM_book_ws/src/myVIOSLAM/build/Sophus/test/core/CMakeFiles/test_common.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/core/CMakeFiles/test_common.dir/depend

