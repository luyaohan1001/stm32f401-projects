# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.20

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0

# Include any dependencies generated for this target.
include CMakeFiles/st-flash.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/st-flash.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/st-flash.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/st-flash.dir/flags.make

CMakeFiles/st-flash.dir/src/st-flash/flash.c.o: CMakeFiles/st-flash.dir/flags.make
CMakeFiles/st-flash.dir/src/st-flash/flash.c.o: src/st-flash/flash.c
CMakeFiles/st-flash.dir/src/st-flash/flash.c.o: CMakeFiles/st-flash.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/st-flash.dir/src/st-flash/flash.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/st-flash.dir/src/st-flash/flash.c.o -MF CMakeFiles/st-flash.dir/src/st-flash/flash.c.o.d -o CMakeFiles/st-flash.dir/src/st-flash/flash.c.o -c /home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/src/st-flash/flash.c

CMakeFiles/st-flash.dir/src/st-flash/flash.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/st-flash.dir/src/st-flash/flash.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/src/st-flash/flash.c > CMakeFiles/st-flash.dir/src/st-flash/flash.c.i

CMakeFiles/st-flash.dir/src/st-flash/flash.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/st-flash.dir/src/st-flash/flash.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/src/st-flash/flash.c -o CMakeFiles/st-flash.dir/src/st-flash/flash.c.s

CMakeFiles/st-flash.dir/src/st-flash/flash_opts.c.o: CMakeFiles/st-flash.dir/flags.make
CMakeFiles/st-flash.dir/src/st-flash/flash_opts.c.o: src/st-flash/flash_opts.c
CMakeFiles/st-flash.dir/src/st-flash/flash_opts.c.o: CMakeFiles/st-flash.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/st-flash.dir/src/st-flash/flash_opts.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/st-flash.dir/src/st-flash/flash_opts.c.o -MF CMakeFiles/st-flash.dir/src/st-flash/flash_opts.c.o.d -o CMakeFiles/st-flash.dir/src/st-flash/flash_opts.c.o -c /home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/src/st-flash/flash_opts.c

CMakeFiles/st-flash.dir/src/st-flash/flash_opts.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/st-flash.dir/src/st-flash/flash_opts.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/src/st-flash/flash_opts.c > CMakeFiles/st-flash.dir/src/st-flash/flash_opts.c.i

CMakeFiles/st-flash.dir/src/st-flash/flash_opts.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/st-flash.dir/src/st-flash/flash_opts.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/src/st-flash/flash_opts.c -o CMakeFiles/st-flash.dir/src/st-flash/flash_opts.c.s

# Object files for target st-flash
st__flash_OBJECTS = \
"CMakeFiles/st-flash.dir/src/st-flash/flash.c.o" \
"CMakeFiles/st-flash.dir/src/st-flash/flash_opts.c.o"

# External object files for target st-flash
st__flash_EXTERNAL_OBJECTS =

bin/st-flash: CMakeFiles/st-flash.dir/src/st-flash/flash.c.o
bin/st-flash: CMakeFiles/st-flash.dir/src/st-flash/flash_opts.c.o
bin/st-flash: CMakeFiles/st-flash.dir/build.make
bin/st-flash: lib/libstlink.so.1.7.0
bin/st-flash: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
bin/st-flash: CMakeFiles/st-flash.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking C executable bin/st-flash"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/st-flash.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/st-flash.dir/build: bin/st-flash
.PHONY : CMakeFiles/st-flash.dir/build

CMakeFiles/st-flash.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/st-flash.dir/cmake_clean.cmake
.PHONY : CMakeFiles/st-flash.dir/clean

CMakeFiles/st-flash.dir/depend:
	cd /home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0 /home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0 /home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0 /home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0 /home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/CMakeFiles/st-flash.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/st-flash.dir/depend

