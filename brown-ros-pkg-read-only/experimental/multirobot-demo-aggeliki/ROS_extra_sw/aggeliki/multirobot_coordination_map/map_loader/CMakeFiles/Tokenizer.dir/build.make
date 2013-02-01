# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.6

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canoncical targets will work.
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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /opt/ros/boxturtle/ros/aggeliki/map_loader

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /opt/ros/boxturtle/ros/aggeliki/map_loader

# Include any dependencies generated for this target.
include CMakeFiles/Tokenizer.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Tokenizer.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Tokenizer.dir/flags.make

CMakeFiles/Tokenizer.dir/src/Tokenizer.o: CMakeFiles/Tokenizer.dir/flags.make
CMakeFiles/Tokenizer.dir/src/Tokenizer.o: src/Tokenizer.cpp
CMakeFiles/Tokenizer.dir/src/Tokenizer.o: manifest.xml
CMakeFiles/Tokenizer.dir/src/Tokenizer.o: /opt/ros/boxturtle/ros/core/genmsg_cpp/manifest.xml
CMakeFiles/Tokenizer.dir/src/Tokenizer.o: /opt/ros/boxturtle/ros/tools/rospack/manifest.xml
CMakeFiles/Tokenizer.dir/src/Tokenizer.o: /opt/ros/boxturtle/ros/core/roslib/manifest.xml
CMakeFiles/Tokenizer.dir/src/Tokenizer.o: /opt/ros/boxturtle/ros/std_msgs/manifest.xml
CMakeFiles/Tokenizer.dir/src/Tokenizer.o: /opt/ros/boxturtle/ros/core/roslang/manifest.xml
CMakeFiles/Tokenizer.dir/src/Tokenizer.o: /opt/ros/boxturtle/ros/3rdparty/xmlrpcpp/manifest.xml
CMakeFiles/Tokenizer.dir/src/Tokenizer.o: /opt/ros/boxturtle/ros/core/rosconsole/manifest.xml
CMakeFiles/Tokenizer.dir/src/Tokenizer.o: /opt/ros/boxturtle/ros/core/roscpp/manifest.xml
CMakeFiles/Tokenizer.dir/src/Tokenizer.o: /opt/ros/boxturtle/ros/core/rospy/manifest.xml
CMakeFiles/Tokenizer.dir/src/Tokenizer.o: /opt/ros/boxturtle/ros/3rdparty/pycrypto/manifest.xml
CMakeFiles/Tokenizer.dir/src/Tokenizer.o: /opt/ros/boxturtle/ros/3rdparty/paramiko/manifest.xml
CMakeFiles/Tokenizer.dir/src/Tokenizer.o: /opt/ros/boxturtle/ros/core/rosout/manifest.xml
CMakeFiles/Tokenizer.dir/src/Tokenizer.o: /opt/ros/boxturtle/ros/tools/roslaunch/manifest.xml
CMakeFiles/Tokenizer.dir/src/Tokenizer.o: /opt/ros/boxturtle/ros/test/rostest/manifest.xml
CMakeFiles/Tokenizer.dir/src/Tokenizer.o: /opt/ros/boxturtle/ros/tools/topic_tools/manifest.xml
CMakeFiles/Tokenizer.dir/src/Tokenizer.o: /opt/ros/boxturtle/ros/tools/rosrecord/manifest.xml
CMakeFiles/Tokenizer.dir/src/Tokenizer.o: /opt/ros/boxturtle/ros/tools/rosbagmigration/manifest.xml
CMakeFiles/Tokenizer.dir/src/Tokenizer.o: /opt/ros/boxturtle/stacks/common_msgs/geometry_msgs/manifest.xml
CMakeFiles/Tokenizer.dir/src/Tokenizer.o: /opt/ros/boxturtle/stacks/common_msgs/nav_msgs/manifest.xml
CMakeFiles/Tokenizer.dir/src/Tokenizer.o: /opt/ros/boxturtle/ros/brown-ros-pkg/position_tracker/manifest.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /opt/ros/boxturtle/ros/aggeliki/map_loader/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/Tokenizer.dir/src/Tokenizer.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/Tokenizer.dir/src/Tokenizer.o -c /opt/ros/boxturtle/ros/aggeliki/map_loader/src/Tokenizer.cpp

CMakeFiles/Tokenizer.dir/src/Tokenizer.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Tokenizer.dir/src/Tokenizer.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /opt/ros/boxturtle/ros/aggeliki/map_loader/src/Tokenizer.cpp > CMakeFiles/Tokenizer.dir/src/Tokenizer.i

CMakeFiles/Tokenizer.dir/src/Tokenizer.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Tokenizer.dir/src/Tokenizer.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /opt/ros/boxturtle/ros/aggeliki/map_loader/src/Tokenizer.cpp -o CMakeFiles/Tokenizer.dir/src/Tokenizer.s

CMakeFiles/Tokenizer.dir/src/Tokenizer.o.requires:
.PHONY : CMakeFiles/Tokenizer.dir/src/Tokenizer.o.requires

CMakeFiles/Tokenizer.dir/src/Tokenizer.o.provides: CMakeFiles/Tokenizer.dir/src/Tokenizer.o.requires
	$(MAKE) -f CMakeFiles/Tokenizer.dir/build.make CMakeFiles/Tokenizer.dir/src/Tokenizer.o.provides.build
.PHONY : CMakeFiles/Tokenizer.dir/src/Tokenizer.o.provides

CMakeFiles/Tokenizer.dir/src/Tokenizer.o.provides.build: CMakeFiles/Tokenizer.dir/src/Tokenizer.o
.PHONY : CMakeFiles/Tokenizer.dir/src/Tokenizer.o.provides.build

# Object files for target Tokenizer
Tokenizer_OBJECTS = \
"CMakeFiles/Tokenizer.dir/src/Tokenizer.o"

# External object files for target Tokenizer
Tokenizer_EXTERNAL_OBJECTS =

lib/libTokenizer.so: CMakeFiles/Tokenizer.dir/src/Tokenizer.o
lib/libTokenizer.so: CMakeFiles/Tokenizer.dir/build.make
lib/libTokenizer.so: CMakeFiles/Tokenizer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library lib/libTokenizer.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Tokenizer.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Tokenizer.dir/build: lib/libTokenizer.so
.PHONY : CMakeFiles/Tokenizer.dir/build

CMakeFiles/Tokenizer.dir/requires: CMakeFiles/Tokenizer.dir/src/Tokenizer.o.requires
.PHONY : CMakeFiles/Tokenizer.dir/requires

CMakeFiles/Tokenizer.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Tokenizer.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Tokenizer.dir/clean

CMakeFiles/Tokenizer.dir/depend:
	cd /opt/ros/boxturtle/ros/aggeliki/map_loader && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /opt/ros/boxturtle/ros/aggeliki/map_loader /opt/ros/boxturtle/ros/aggeliki/map_loader /opt/ros/boxturtle/ros/aggeliki/map_loader /opt/ros/boxturtle/ros/aggeliki/map_loader /opt/ros/boxturtle/ros/aggeliki/map_loader/CMakeFiles/Tokenizer.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Tokenizer.dir/depend
