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
CMAKE_SOURCE_DIR = /home/limenet1/Work/PCIe_5GRadio_3v1/D20221027/LMS8FE_1v1

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/limenet1/Work/PCIe_5GRadio_3v1/D20221027/LMS8FE_1v1/build

# Include any dependencies generated for this target.
include LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/depend.make

# Include the progress variables for this target.
include LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/progress.make

# Include the compile flags for this target's objects.
include LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/flags.make

LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/lms8suiteApp.cpp.o: LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/flags.make
LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/lms8suiteApp.cpp.o: ../LMS8FE/LMS8001/lms8suiteApp.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/limenet1/Work/PCIe_5GRadio_3v1/D20221027/LMS8FE_1v1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/lms8suiteApp.cpp.o"
	cd /home/limenet1/Work/PCIe_5GRadio_3v1/D20221027/LMS8FE_1v1/build/LMS8FE/LMS8001 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/lms8suite.dir/lms8suiteApp.cpp.o -c /home/limenet1/Work/PCIe_5GRadio_3v1/D20221027/LMS8FE_1v1/LMS8FE/LMS8001/lms8suiteApp.cpp

LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/lms8suiteApp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lms8suite.dir/lms8suiteApp.cpp.i"
	cd /home/limenet1/Work/PCIe_5GRadio_3v1/D20221027/LMS8FE_1v1/build/LMS8FE/LMS8001 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/limenet1/Work/PCIe_5GRadio_3v1/D20221027/LMS8FE_1v1/LMS8FE/LMS8001/lms8suiteApp.cpp > CMakeFiles/lms8suite.dir/lms8suiteApp.cpp.i

LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/lms8suiteApp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lms8suite.dir/lms8suiteApp.cpp.s"
	cd /home/limenet1/Work/PCIe_5GRadio_3v1/D20221027/LMS8FE_1v1/build/LMS8FE/LMS8001 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/limenet1/Work/PCIe_5GRadio_3v1/D20221027/LMS8FE_1v1/LMS8FE/LMS8001/lms8suiteApp.cpp -o CMakeFiles/lms8suite.dir/lms8suiteApp.cpp.s

LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/lms8suiteApp.cpp.o.requires:

.PHONY : LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/lms8suiteApp.cpp.o.requires

LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/lms8suiteApp.cpp.o.provides: LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/lms8suiteApp.cpp.o.requires
	$(MAKE) -f LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/build.make LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/lms8suiteApp.cpp.o.provides.build
.PHONY : LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/lms8suiteApp.cpp.o.provides

LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/lms8suiteApp.cpp.o.provides.build: LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/lms8suiteApp.cpp.o


LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/lms8suiteAppFrame.cpp.o: LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/flags.make
LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/lms8suiteAppFrame.cpp.o: ../LMS8FE/LMS8001/lms8suiteAppFrame.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/limenet1/Work/PCIe_5GRadio_3v1/D20221027/LMS8FE_1v1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/lms8suiteAppFrame.cpp.o"
	cd /home/limenet1/Work/PCIe_5GRadio_3v1/D20221027/LMS8FE_1v1/build/LMS8FE/LMS8001 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/lms8suite.dir/lms8suiteAppFrame.cpp.o -c /home/limenet1/Work/PCIe_5GRadio_3v1/D20221027/LMS8FE_1v1/LMS8FE/LMS8001/lms8suiteAppFrame.cpp

LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/lms8suiteAppFrame.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lms8suite.dir/lms8suiteAppFrame.cpp.i"
	cd /home/limenet1/Work/PCIe_5GRadio_3v1/D20221027/LMS8FE_1v1/build/LMS8FE/LMS8001 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/limenet1/Work/PCIe_5GRadio_3v1/D20221027/LMS8FE_1v1/LMS8FE/LMS8001/lms8suiteAppFrame.cpp > CMakeFiles/lms8suite.dir/lms8suiteAppFrame.cpp.i

LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/lms8suiteAppFrame.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lms8suite.dir/lms8suiteAppFrame.cpp.s"
	cd /home/limenet1/Work/PCIe_5GRadio_3v1/D20221027/LMS8FE_1v1/build/LMS8FE/LMS8001 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/limenet1/Work/PCIe_5GRadio_3v1/D20221027/LMS8FE_1v1/LMS8FE/LMS8001/lms8suiteAppFrame.cpp -o CMakeFiles/lms8suite.dir/lms8suiteAppFrame.cpp.s

LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/lms8suiteAppFrame.cpp.o.requires:

.PHONY : LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/lms8suiteAppFrame.cpp.o.requires

LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/lms8suiteAppFrame.cpp.o.provides: LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/lms8suiteAppFrame.cpp.o.requires
	$(MAKE) -f LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/build.make LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/lms8suiteAppFrame.cpp.o.provides.build
.PHONY : LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/lms8suiteAppFrame.cpp.o.provides

LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/lms8suiteAppFrame.cpp.o.provides.build: LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/lms8suiteAppFrame.cpp.o


LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/lms8suiteApp_gui.cpp.o: LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/flags.make
LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/lms8suiteApp_gui.cpp.o: ../LMS8FE/LMS8001/lms8suiteApp_gui.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/limenet1/Work/PCIe_5GRadio_3v1/D20221027/LMS8FE_1v1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/lms8suiteApp_gui.cpp.o"
	cd /home/limenet1/Work/PCIe_5GRadio_3v1/D20221027/LMS8FE_1v1/build/LMS8FE/LMS8001 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/lms8suite.dir/lms8suiteApp_gui.cpp.o -c /home/limenet1/Work/PCIe_5GRadio_3v1/D20221027/LMS8FE_1v1/LMS8FE/LMS8001/lms8suiteApp_gui.cpp

LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/lms8suiteApp_gui.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lms8suite.dir/lms8suiteApp_gui.cpp.i"
	cd /home/limenet1/Work/PCIe_5GRadio_3v1/D20221027/LMS8FE_1v1/build/LMS8FE/LMS8001 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/limenet1/Work/PCIe_5GRadio_3v1/D20221027/LMS8FE_1v1/LMS8FE/LMS8001/lms8suiteApp_gui.cpp > CMakeFiles/lms8suite.dir/lms8suiteApp_gui.cpp.i

LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/lms8suiteApp_gui.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lms8suite.dir/lms8suiteApp_gui.cpp.s"
	cd /home/limenet1/Work/PCIe_5GRadio_3v1/D20221027/LMS8FE_1v1/build/LMS8FE/LMS8001 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/limenet1/Work/PCIe_5GRadio_3v1/D20221027/LMS8FE_1v1/LMS8FE/LMS8001/lms8suiteApp_gui.cpp -o CMakeFiles/lms8suite.dir/lms8suiteApp_gui.cpp.s

LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/lms8suiteApp_gui.cpp.o.requires:

.PHONY : LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/lms8suiteApp_gui.cpp.o.requires

LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/lms8suiteApp_gui.cpp.o.provides: LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/lms8suiteApp_gui.cpp.o.requires
	$(MAKE) -f LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/build.make LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/lms8suiteApp_gui.cpp.o.provides.build
.PHONY : LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/lms8suiteApp_gui.cpp.o.provides

LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/lms8suiteApp_gui.cpp.o.provides.build: LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/lms8suiteApp_gui.cpp.o


LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/dlgAbout.cpp.o: LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/flags.make
LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/dlgAbout.cpp.o: ../LMS8FE/LMS8001/dlgAbout.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/limenet1/Work/PCIe_5GRadio_3v1/D20221027/LMS8FE_1v1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/dlgAbout.cpp.o"
	cd /home/limenet1/Work/PCIe_5GRadio_3v1/D20221027/LMS8FE_1v1/build/LMS8FE/LMS8001 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/lms8suite.dir/dlgAbout.cpp.o -c /home/limenet1/Work/PCIe_5GRadio_3v1/D20221027/LMS8FE_1v1/LMS8FE/LMS8001/dlgAbout.cpp

LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/dlgAbout.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lms8suite.dir/dlgAbout.cpp.i"
	cd /home/limenet1/Work/PCIe_5GRadio_3v1/D20221027/LMS8FE_1v1/build/LMS8FE/LMS8001 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/limenet1/Work/PCIe_5GRadio_3v1/D20221027/LMS8FE_1v1/LMS8FE/LMS8001/dlgAbout.cpp > CMakeFiles/lms8suite.dir/dlgAbout.cpp.i

LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/dlgAbout.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lms8suite.dir/dlgAbout.cpp.s"
	cd /home/limenet1/Work/PCIe_5GRadio_3v1/D20221027/LMS8FE_1v1/build/LMS8FE/LMS8001 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/limenet1/Work/PCIe_5GRadio_3v1/D20221027/LMS8FE_1v1/LMS8FE/LMS8001/dlgAbout.cpp -o CMakeFiles/lms8suite.dir/dlgAbout.cpp.s

LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/dlgAbout.cpp.o.requires:

.PHONY : LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/dlgAbout.cpp.o.requires

LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/dlgAbout.cpp.o.provides: LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/dlgAbout.cpp.o.requires
	$(MAKE) -f LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/build.make LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/dlgAbout.cpp.o.provides.build
.PHONY : LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/dlgAbout.cpp.o.provides

LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/dlgAbout.cpp.o.provides.build: LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/dlgAbout.cpp.o


# Object files for target lms8suite
lms8suite_OBJECTS = \
"CMakeFiles/lms8suite.dir/lms8suiteApp.cpp.o" \
"CMakeFiles/lms8suite.dir/lms8suiteAppFrame.cpp.o" \
"CMakeFiles/lms8suite.dir/lms8suiteApp_gui.cpp.o" \
"CMakeFiles/lms8suite.dir/dlgAbout.cpp.o"

# External object files for target lms8suite
lms8suite_EXTERNAL_OBJECTS =

LMS8FE/LMS8001/liblms8suite.a: LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/lms8suiteApp.cpp.o
LMS8FE/LMS8001/liblms8suite.a: LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/lms8suiteAppFrame.cpp.o
LMS8FE/LMS8001/liblms8suite.a: LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/lms8suiteApp_gui.cpp.o
LMS8FE/LMS8001/liblms8suite.a: LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/dlgAbout.cpp.o
LMS8FE/LMS8001/liblms8suite.a: LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/build.make
LMS8FE/LMS8001/liblms8suite.a: LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/limenet1/Work/PCIe_5GRadio_3v1/D20221027/LMS8FE_1v1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX static library liblms8suite.a"
	cd /home/limenet1/Work/PCIe_5GRadio_3v1/D20221027/LMS8FE_1v1/build/LMS8FE/LMS8001 && $(CMAKE_COMMAND) -P CMakeFiles/lms8suite.dir/cmake_clean_target.cmake
	cd /home/limenet1/Work/PCIe_5GRadio_3v1/D20221027/LMS8FE_1v1/build/LMS8FE/LMS8001 && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lms8suite.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/build: LMS8FE/LMS8001/liblms8suite.a

.PHONY : LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/build

LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/requires: LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/lms8suiteApp.cpp.o.requires
LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/requires: LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/lms8suiteAppFrame.cpp.o.requires
LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/requires: LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/lms8suiteApp_gui.cpp.o.requires
LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/requires: LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/dlgAbout.cpp.o.requires

.PHONY : LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/requires

LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/clean:
	cd /home/limenet1/Work/PCIe_5GRadio_3v1/D20221027/LMS8FE_1v1/build/LMS8FE/LMS8001 && $(CMAKE_COMMAND) -P CMakeFiles/lms8suite.dir/cmake_clean.cmake
.PHONY : LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/clean

LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/depend:
	cd /home/limenet1/Work/PCIe_5GRadio_3v1/D20221027/LMS8FE_1v1/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/limenet1/Work/PCIe_5GRadio_3v1/D20221027/LMS8FE_1v1 /home/limenet1/Work/PCIe_5GRadio_3v1/D20221027/LMS8FE_1v1/LMS8FE/LMS8001 /home/limenet1/Work/PCIe_5GRadio_3v1/D20221027/LMS8FE_1v1/build /home/limenet1/Work/PCIe_5GRadio_3v1/D20221027/LMS8FE_1v1/build/LMS8FE/LMS8001 /home/limenet1/Work/PCIe_5GRadio_3v1/D20221027/LMS8FE_1v1/build/LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : LMS8FE/LMS8001/CMakeFiles/lms8suite.dir/depend

