# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/southriver/hw_min_snap_full/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/southriver/hw_min_snap_full/build

# Include any dependencies generated for this target.
include planner/traj_server/CMakeFiles/my_traj_server.dir/depend.make

# Include the progress variables for this target.
include planner/traj_server/CMakeFiles/my_traj_server.dir/progress.make

# Include the compile flags for this target's objects.
include planner/traj_server/CMakeFiles/my_traj_server.dir/flags.make

planner/traj_server/CMakeFiles/my_traj_server.dir/src/my_traj_server.cpp.o: planner/traj_server/CMakeFiles/my_traj_server.dir/flags.make
planner/traj_server/CMakeFiles/my_traj_server.dir/src/my_traj_server.cpp.o: /home/southriver/hw_min_snap_full/src/planner/traj_server/src/my_traj_server.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/southriver/hw_min_snap_full/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object planner/traj_server/CMakeFiles/my_traj_server.dir/src/my_traj_server.cpp.o"
	cd /home/southriver/hw_min_snap_full/build/planner/traj_server && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/my_traj_server.dir/src/my_traj_server.cpp.o -c /home/southriver/hw_min_snap_full/src/planner/traj_server/src/my_traj_server.cpp

planner/traj_server/CMakeFiles/my_traj_server.dir/src/my_traj_server.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/my_traj_server.dir/src/my_traj_server.cpp.i"
	cd /home/southriver/hw_min_snap_full/build/planner/traj_server && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/southriver/hw_min_snap_full/src/planner/traj_server/src/my_traj_server.cpp > CMakeFiles/my_traj_server.dir/src/my_traj_server.cpp.i

planner/traj_server/CMakeFiles/my_traj_server.dir/src/my_traj_server.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/my_traj_server.dir/src/my_traj_server.cpp.s"
	cd /home/southriver/hw_min_snap_full/build/planner/traj_server && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/southriver/hw_min_snap_full/src/planner/traj_server/src/my_traj_server.cpp -o CMakeFiles/my_traj_server.dir/src/my_traj_server.cpp.s

planner/traj_server/CMakeFiles/my_traj_server.dir/src/my_traj_server.cpp.o.requires:

.PHONY : planner/traj_server/CMakeFiles/my_traj_server.dir/src/my_traj_server.cpp.o.requires

planner/traj_server/CMakeFiles/my_traj_server.dir/src/my_traj_server.cpp.o.provides: planner/traj_server/CMakeFiles/my_traj_server.dir/src/my_traj_server.cpp.o.requires
	$(MAKE) -f planner/traj_server/CMakeFiles/my_traj_server.dir/build.make planner/traj_server/CMakeFiles/my_traj_server.dir/src/my_traj_server.cpp.o.provides.build
.PHONY : planner/traj_server/CMakeFiles/my_traj_server.dir/src/my_traj_server.cpp.o.provides

planner/traj_server/CMakeFiles/my_traj_server.dir/src/my_traj_server.cpp.o.provides.build: planner/traj_server/CMakeFiles/my_traj_server.dir/src/my_traj_server.cpp.o


# Object files for target my_traj_server
my_traj_server_OBJECTS = \
"CMakeFiles/my_traj_server.dir/src/my_traj_server.cpp.o"

# External object files for target my_traj_server
my_traj_server_EXTERNAL_OBJECTS =

/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: planner/traj_server/CMakeFiles/my_traj_server.dir/src/my_traj_server.cpp.o
/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: planner/traj_server/CMakeFiles/my_traj_server.dir/build.make
/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: /opt/ros/melodic/lib/libroscpp.so
/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: /opt/ros/melodic/lib/librosconsole.so
/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: /home/southriver/hw_min_snap_full/devel/lib/libencode_msgs.so
/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: /home/southriver/hw_min_snap_full/devel/lib/libdecode_msgs.so
/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: /opt/ros/melodic/lib/librostime.so
/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: /opt/ros/melodic/lib/libcpp_common.so
/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: /home/southriver/hw_min_snap_full/devel/lib/libtraj_utils.so
/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: /opt/ros/melodic/lib/libroscpp.so
/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: /opt/ros/melodic/lib/libcv_bridge.so
/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: /opt/ros/melodic/lib/librosconsole.so
/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: /opt/ros/melodic/lib/librostime.so
/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: /opt/ros/melodic/lib/libcpp_common.so
/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server: planner/traj_server/CMakeFiles/my_traj_server.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/southriver/hw_min_snap_full/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server"
	cd /home/southriver/hw_min_snap_full/build/planner/traj_server && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/my_traj_server.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
planner/traj_server/CMakeFiles/my_traj_server.dir/build: /home/southriver/hw_min_snap_full/devel/lib/traj_server/my_traj_server

.PHONY : planner/traj_server/CMakeFiles/my_traj_server.dir/build

planner/traj_server/CMakeFiles/my_traj_server.dir/requires: planner/traj_server/CMakeFiles/my_traj_server.dir/src/my_traj_server.cpp.o.requires

.PHONY : planner/traj_server/CMakeFiles/my_traj_server.dir/requires

planner/traj_server/CMakeFiles/my_traj_server.dir/clean:
	cd /home/southriver/hw_min_snap_full/build/planner/traj_server && $(CMAKE_COMMAND) -P CMakeFiles/my_traj_server.dir/cmake_clean.cmake
.PHONY : planner/traj_server/CMakeFiles/my_traj_server.dir/clean

planner/traj_server/CMakeFiles/my_traj_server.dir/depend:
	cd /home/southriver/hw_min_snap_full/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/southriver/hw_min_snap_full/src /home/southriver/hw_min_snap_full/src/planner/traj_server /home/southriver/hw_min_snap_full/build /home/southriver/hw_min_snap_full/build/planner/traj_server /home/southriver/hw_min_snap_full/build/planner/traj_server/CMakeFiles/my_traj_server.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : planner/traj_server/CMakeFiles/my_traj_server.dir/depend

