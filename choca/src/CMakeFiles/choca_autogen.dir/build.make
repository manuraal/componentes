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
CMAKE_SOURCE_DIR = /home/salabeta/robocomp/components/componentes/choca

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/salabeta/robocomp/components/componentes/choca

# Utility rule file for choca_autogen.

# Include the progress variables for this target.
include src/CMakeFiles/choca_autogen.dir/progress.make

src/CMakeFiles/choca_autogen:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/salabeta/robocomp/components/componentes/choca/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Automatic MOC and UIC for target choca"
	cd /home/salabeta/robocomp/components/componentes/choca/src && /usr/bin/cmake -E cmake_autogen /home/salabeta/robocomp/components/componentes/choca/src/CMakeFiles/choca_autogen.dir ""

choca_autogen: src/CMakeFiles/choca_autogen
choca_autogen: src/CMakeFiles/choca_autogen.dir/build.make

.PHONY : choca_autogen

# Rule to build all files generated by this target.
src/CMakeFiles/choca_autogen.dir/build: choca_autogen

.PHONY : src/CMakeFiles/choca_autogen.dir/build

src/CMakeFiles/choca_autogen.dir/clean:
	cd /home/salabeta/robocomp/components/componentes/choca/src && $(CMAKE_COMMAND) -P CMakeFiles/choca_autogen.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/choca_autogen.dir/clean

src/CMakeFiles/choca_autogen.dir/depend:
	cd /home/salabeta/robocomp/components/componentes/choca && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/salabeta/robocomp/components/componentes/choca /home/salabeta/robocomp/components/componentes/choca/src /home/salabeta/robocomp/components/componentes/choca /home/salabeta/robocomp/components/componentes/choca/src /home/salabeta/robocomp/components/componentes/choca/src/CMakeFiles/choca_autogen.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/choca_autogen.dir/depend

