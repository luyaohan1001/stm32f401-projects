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
include CMakeFiles/stlink-shared.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/stlink-shared.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/stlink-shared.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/stlink-shared.dir/flags.make

CMakeFiles/stlink-shared.dir/src/common.c.o: CMakeFiles/stlink-shared.dir/flags.make
CMakeFiles/stlink-shared.dir/src/common.c.o: src/common.c
CMakeFiles/stlink-shared.dir/src/common.c.o: CMakeFiles/stlink-shared.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/stlink-shared.dir/src/common.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/stlink-shared.dir/src/common.c.o -MF CMakeFiles/stlink-shared.dir/src/common.c.o.d -o CMakeFiles/stlink-shared.dir/src/common.c.o -c /home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/src/common.c

CMakeFiles/stlink-shared.dir/src/common.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/stlink-shared.dir/src/common.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/src/common.c > CMakeFiles/stlink-shared.dir/src/common.c.i

CMakeFiles/stlink-shared.dir/src/common.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/stlink-shared.dir/src/common.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/src/common.c -o CMakeFiles/stlink-shared.dir/src/common.c.s

CMakeFiles/stlink-shared.dir/src/stlink-lib/chipid.c.o: CMakeFiles/stlink-shared.dir/flags.make
CMakeFiles/stlink-shared.dir/src/stlink-lib/chipid.c.o: src/stlink-lib/chipid.c
CMakeFiles/stlink-shared.dir/src/stlink-lib/chipid.c.o: CMakeFiles/stlink-shared.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/stlink-shared.dir/src/stlink-lib/chipid.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/stlink-shared.dir/src/stlink-lib/chipid.c.o -MF CMakeFiles/stlink-shared.dir/src/stlink-lib/chipid.c.o.d -o CMakeFiles/stlink-shared.dir/src/stlink-lib/chipid.c.o -c /home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/src/stlink-lib/chipid.c

CMakeFiles/stlink-shared.dir/src/stlink-lib/chipid.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/stlink-shared.dir/src/stlink-lib/chipid.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/src/stlink-lib/chipid.c > CMakeFiles/stlink-shared.dir/src/stlink-lib/chipid.c.i

CMakeFiles/stlink-shared.dir/src/stlink-lib/chipid.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/stlink-shared.dir/src/stlink-lib/chipid.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/src/stlink-lib/chipid.c -o CMakeFiles/stlink-shared.dir/src/stlink-lib/chipid.c.s

CMakeFiles/stlink-shared.dir/src/stlink-lib/flash_loader.c.o: CMakeFiles/stlink-shared.dir/flags.make
CMakeFiles/stlink-shared.dir/src/stlink-lib/flash_loader.c.o: src/stlink-lib/flash_loader.c
CMakeFiles/stlink-shared.dir/src/stlink-lib/flash_loader.c.o: CMakeFiles/stlink-shared.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object CMakeFiles/stlink-shared.dir/src/stlink-lib/flash_loader.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/stlink-shared.dir/src/stlink-lib/flash_loader.c.o -MF CMakeFiles/stlink-shared.dir/src/stlink-lib/flash_loader.c.o.d -o CMakeFiles/stlink-shared.dir/src/stlink-lib/flash_loader.c.o -c /home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/src/stlink-lib/flash_loader.c

CMakeFiles/stlink-shared.dir/src/stlink-lib/flash_loader.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/stlink-shared.dir/src/stlink-lib/flash_loader.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/src/stlink-lib/flash_loader.c > CMakeFiles/stlink-shared.dir/src/stlink-lib/flash_loader.c.i

CMakeFiles/stlink-shared.dir/src/stlink-lib/flash_loader.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/stlink-shared.dir/src/stlink-lib/flash_loader.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/src/stlink-lib/flash_loader.c -o CMakeFiles/stlink-shared.dir/src/stlink-lib/flash_loader.c.s

CMakeFiles/stlink-shared.dir/src/stlink-lib/logging.c.o: CMakeFiles/stlink-shared.dir/flags.make
CMakeFiles/stlink-shared.dir/src/stlink-lib/logging.c.o: src/stlink-lib/logging.c
CMakeFiles/stlink-shared.dir/src/stlink-lib/logging.c.o: CMakeFiles/stlink-shared.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object CMakeFiles/stlink-shared.dir/src/stlink-lib/logging.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/stlink-shared.dir/src/stlink-lib/logging.c.o -MF CMakeFiles/stlink-shared.dir/src/stlink-lib/logging.c.o.d -o CMakeFiles/stlink-shared.dir/src/stlink-lib/logging.c.o -c /home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/src/stlink-lib/logging.c

CMakeFiles/stlink-shared.dir/src/stlink-lib/logging.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/stlink-shared.dir/src/stlink-lib/logging.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/src/stlink-lib/logging.c > CMakeFiles/stlink-shared.dir/src/stlink-lib/logging.c.i

CMakeFiles/stlink-shared.dir/src/stlink-lib/logging.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/stlink-shared.dir/src/stlink-lib/logging.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/src/stlink-lib/logging.c -o CMakeFiles/stlink-shared.dir/src/stlink-lib/logging.c.s

CMakeFiles/stlink-shared.dir/src/stlink-lib/md5.c.o: CMakeFiles/stlink-shared.dir/flags.make
CMakeFiles/stlink-shared.dir/src/stlink-lib/md5.c.o: src/stlink-lib/md5.c
CMakeFiles/stlink-shared.dir/src/stlink-lib/md5.c.o: CMakeFiles/stlink-shared.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object CMakeFiles/stlink-shared.dir/src/stlink-lib/md5.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/stlink-shared.dir/src/stlink-lib/md5.c.o -MF CMakeFiles/stlink-shared.dir/src/stlink-lib/md5.c.o.d -o CMakeFiles/stlink-shared.dir/src/stlink-lib/md5.c.o -c /home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/src/stlink-lib/md5.c

CMakeFiles/stlink-shared.dir/src/stlink-lib/md5.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/stlink-shared.dir/src/stlink-lib/md5.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/src/stlink-lib/md5.c > CMakeFiles/stlink-shared.dir/src/stlink-lib/md5.c.i

CMakeFiles/stlink-shared.dir/src/stlink-lib/md5.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/stlink-shared.dir/src/stlink-lib/md5.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/src/stlink-lib/md5.c -o CMakeFiles/stlink-shared.dir/src/stlink-lib/md5.c.s

CMakeFiles/stlink-shared.dir/src/stlink-lib/sg.c.o: CMakeFiles/stlink-shared.dir/flags.make
CMakeFiles/stlink-shared.dir/src/stlink-lib/sg.c.o: src/stlink-lib/sg.c
CMakeFiles/stlink-shared.dir/src/stlink-lib/sg.c.o: CMakeFiles/stlink-shared.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building C object CMakeFiles/stlink-shared.dir/src/stlink-lib/sg.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/stlink-shared.dir/src/stlink-lib/sg.c.o -MF CMakeFiles/stlink-shared.dir/src/stlink-lib/sg.c.o.d -o CMakeFiles/stlink-shared.dir/src/stlink-lib/sg.c.o -c /home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/src/stlink-lib/sg.c

CMakeFiles/stlink-shared.dir/src/stlink-lib/sg.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/stlink-shared.dir/src/stlink-lib/sg.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/src/stlink-lib/sg.c > CMakeFiles/stlink-shared.dir/src/stlink-lib/sg.c.i

CMakeFiles/stlink-shared.dir/src/stlink-lib/sg.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/stlink-shared.dir/src/stlink-lib/sg.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/src/stlink-lib/sg.c -o CMakeFiles/stlink-shared.dir/src/stlink-lib/sg.c.s

CMakeFiles/stlink-shared.dir/src/stlink-lib/usb.c.o: CMakeFiles/stlink-shared.dir/flags.make
CMakeFiles/stlink-shared.dir/src/stlink-lib/usb.c.o: src/stlink-lib/usb.c
CMakeFiles/stlink-shared.dir/src/stlink-lib/usb.c.o: CMakeFiles/stlink-shared.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building C object CMakeFiles/stlink-shared.dir/src/stlink-lib/usb.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/stlink-shared.dir/src/stlink-lib/usb.c.o -MF CMakeFiles/stlink-shared.dir/src/stlink-lib/usb.c.o.d -o CMakeFiles/stlink-shared.dir/src/stlink-lib/usb.c.o -c /home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/src/stlink-lib/usb.c

CMakeFiles/stlink-shared.dir/src/stlink-lib/usb.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/stlink-shared.dir/src/stlink-lib/usb.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/src/stlink-lib/usb.c > CMakeFiles/stlink-shared.dir/src/stlink-lib/usb.c.i

CMakeFiles/stlink-shared.dir/src/stlink-lib/usb.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/stlink-shared.dir/src/stlink-lib/usb.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/src/stlink-lib/usb.c -o CMakeFiles/stlink-shared.dir/src/stlink-lib/usb.c.s

CMakeFiles/stlink-shared.dir/src/stlink-lib/helper.c.o: CMakeFiles/stlink-shared.dir/flags.make
CMakeFiles/stlink-shared.dir/src/stlink-lib/helper.c.o: src/stlink-lib/helper.c
CMakeFiles/stlink-shared.dir/src/stlink-lib/helper.c.o: CMakeFiles/stlink-shared.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building C object CMakeFiles/stlink-shared.dir/src/stlink-lib/helper.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/stlink-shared.dir/src/stlink-lib/helper.c.o -MF CMakeFiles/stlink-shared.dir/src/stlink-lib/helper.c.o.d -o CMakeFiles/stlink-shared.dir/src/stlink-lib/helper.c.o -c /home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/src/stlink-lib/helper.c

CMakeFiles/stlink-shared.dir/src/stlink-lib/helper.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/stlink-shared.dir/src/stlink-lib/helper.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/src/stlink-lib/helper.c > CMakeFiles/stlink-shared.dir/src/stlink-lib/helper.c.i

CMakeFiles/stlink-shared.dir/src/stlink-lib/helper.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/stlink-shared.dir/src/stlink-lib/helper.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/src/stlink-lib/helper.c -o CMakeFiles/stlink-shared.dir/src/stlink-lib/helper.c.s

# Object files for target stlink-shared
stlink__shared_OBJECTS = \
"CMakeFiles/stlink-shared.dir/src/common.c.o" \
"CMakeFiles/stlink-shared.dir/src/stlink-lib/chipid.c.o" \
"CMakeFiles/stlink-shared.dir/src/stlink-lib/flash_loader.c.o" \
"CMakeFiles/stlink-shared.dir/src/stlink-lib/logging.c.o" \
"CMakeFiles/stlink-shared.dir/src/stlink-lib/md5.c.o" \
"CMakeFiles/stlink-shared.dir/src/stlink-lib/sg.c.o" \
"CMakeFiles/stlink-shared.dir/src/stlink-lib/usb.c.o" \
"CMakeFiles/stlink-shared.dir/src/stlink-lib/helper.c.o"

# External object files for target stlink-shared
stlink__shared_EXTERNAL_OBJECTS =

lib/libstlink.so.1.7.0: CMakeFiles/stlink-shared.dir/src/common.c.o
lib/libstlink.so.1.7.0: CMakeFiles/stlink-shared.dir/src/stlink-lib/chipid.c.o
lib/libstlink.so.1.7.0: CMakeFiles/stlink-shared.dir/src/stlink-lib/flash_loader.c.o
lib/libstlink.so.1.7.0: CMakeFiles/stlink-shared.dir/src/stlink-lib/logging.c.o
lib/libstlink.so.1.7.0: CMakeFiles/stlink-shared.dir/src/stlink-lib/md5.c.o
lib/libstlink.so.1.7.0: CMakeFiles/stlink-shared.dir/src/stlink-lib/sg.c.o
lib/libstlink.so.1.7.0: CMakeFiles/stlink-shared.dir/src/stlink-lib/usb.c.o
lib/libstlink.so.1.7.0: CMakeFiles/stlink-shared.dir/src/stlink-lib/helper.c.o
lib/libstlink.so.1.7.0: CMakeFiles/stlink-shared.dir/build.make
lib/libstlink.so.1.7.0: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
lib/libstlink.so.1.7.0: CMakeFiles/stlink-shared.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Linking C shared library lib/libstlink.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/stlink-shared.dir/link.txt --verbose=$(VERBOSE)
	$(CMAKE_COMMAND) -E cmake_symlink_library lib/libstlink.so.1.7.0 lib/libstlink.so.1 lib/libstlink.so

lib/libstlink.so.1: lib/libstlink.so.1.7.0
	@$(CMAKE_COMMAND) -E touch_nocreate lib/libstlink.so.1

lib/libstlink.so: lib/libstlink.so.1.7.0
	@$(CMAKE_COMMAND) -E touch_nocreate lib/libstlink.so

# Rule to build all files generated by this target.
CMakeFiles/stlink-shared.dir/build: lib/libstlink.so
.PHONY : CMakeFiles/stlink-shared.dir/build

CMakeFiles/stlink-shared.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/stlink-shared.dir/cmake_clean.cmake
.PHONY : CMakeFiles/stlink-shared.dir/clean

CMakeFiles/stlink-shared.dir/depend:
	cd /home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0 /home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0 /home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0 /home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0 /home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/CMakeFiles/stlink-shared.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/stlink-shared.dir/depend
