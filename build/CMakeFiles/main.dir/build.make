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
CMAKE_SOURCE_DIR = /home/feng_shihao/fsh_file/MicroStrain_IMU

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/feng_shihao/fsh_file/MicroStrain_IMU/build

# Include any dependencies generated for this target.
include CMakeFiles/main.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/main.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/main.dir/flags.make

CMakeFiles/main.dir/src/main.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/feng_shihao/fsh_file/MicroStrain_IMU/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/main.dir/src/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/src/main.cpp.o -c /home/feng_shihao/fsh_file/MicroStrain_IMU/src/main.cpp

CMakeFiles/main.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/feng_shihao/fsh_file/MicroStrain_IMU/src/main.cpp > CMakeFiles/main.dir/src/main.cpp.i

CMakeFiles/main.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/feng_shihao/fsh_file/MicroStrain_IMU/src/main.cpp -o CMakeFiles/main.dir/src/main.cpp.s

CMakeFiles/main.dir/src/microstrain_imu.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/microstrain_imu.cpp.o: ../src/microstrain_imu.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/feng_shihao/fsh_file/MicroStrain_IMU/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/main.dir/src/microstrain_imu.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/src/microstrain_imu.cpp.o -c /home/feng_shihao/fsh_file/MicroStrain_IMU/src/microstrain_imu.cpp

CMakeFiles/main.dir/src/microstrain_imu.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/microstrain_imu.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/feng_shihao/fsh_file/MicroStrain_IMU/src/microstrain_imu.cpp > CMakeFiles/main.dir/src/microstrain_imu.cpp.i

CMakeFiles/main.dir/src/microstrain_imu.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/microstrain_imu.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/feng_shihao/fsh_file/MicroStrain_IMU/src/microstrain_imu.cpp -o CMakeFiles/main.dir/src/microstrain_imu.cpp.s

CMakeFiles/main.dir/src/periodic_rt_task.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/periodic_rt_task.cpp.o: ../src/periodic_rt_task.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/feng_shihao/fsh_file/MicroStrain_IMU/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/main.dir/src/periodic_rt_task.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/src/periodic_rt_task.cpp.o -c /home/feng_shihao/fsh_file/MicroStrain_IMU/src/periodic_rt_task.cpp

CMakeFiles/main.dir/src/periodic_rt_task.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/periodic_rt_task.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/feng_shihao/fsh_file/MicroStrain_IMU/src/periodic_rt_task.cpp > CMakeFiles/main.dir/src/periodic_rt_task.cpp.i

CMakeFiles/main.dir/src/periodic_rt_task.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/periodic_rt_task.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/feng_shihao/fsh_file/MicroStrain_IMU/src/periodic_rt_task.cpp -o CMakeFiles/main.dir/src/periodic_rt_task.cpp.s

# Object files for target main
main_OBJECTS = \
"CMakeFiles/main.dir/src/main.cpp.o" \
"CMakeFiles/main.dir/src/microstrain_imu.cpp.o" \
"CMakeFiles/main.dir/src/periodic_rt_task.cpp.o"

# External object files for target main
main_EXTERNAL_OBJECTS =

../bin/main: CMakeFiles/main.dir/src/main.cpp.o
../bin/main: CMakeFiles/main.dir/src/microstrain_imu.cpp.o
../bin/main: CMakeFiles/main.dir/src/periodic_rt_task.cpp.o
../bin/main: CMakeFiles/main.dir/build.make
../bin/main: CMakeFiles/main.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/feng_shihao/fsh_file/MicroStrain_IMU/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable ../bin/main"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/main.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/main.dir/build: ../bin/main

.PHONY : CMakeFiles/main.dir/build

CMakeFiles/main.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/main.dir/cmake_clean.cmake
.PHONY : CMakeFiles/main.dir/clean

CMakeFiles/main.dir/depend:
	cd /home/feng_shihao/fsh_file/MicroStrain_IMU/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/feng_shihao/fsh_file/MicroStrain_IMU /home/feng_shihao/fsh_file/MicroStrain_IMU /home/feng_shihao/fsh_file/MicroStrain_IMU/build /home/feng_shihao/fsh_file/MicroStrain_IMU/build /home/feng_shihao/fsh_file/MicroStrain_IMU/build/CMakeFiles/main.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/main.dir/depend

