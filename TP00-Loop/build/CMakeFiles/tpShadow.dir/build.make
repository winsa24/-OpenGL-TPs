# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.18

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
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.18.3/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.18.3/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/weixiang/Downloads/TP00-Loop

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/weixiang/Downloads/TP00-Loop/build

# Include any dependencies generated for this target.
include CMakeFiles/tpShadow.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/tpShadow.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/tpShadow.dir/flags.make

CMakeFiles/tpShadow.dir/src/main.cpp.o: CMakeFiles/tpShadow.dir/flags.make
CMakeFiles/tpShadow.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/weixiang/Downloads/TP00-Loop/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/tpShadow.dir/src/main.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/tpShadow.dir/src/main.cpp.o -c /Users/weixiang/Downloads/TP00-Loop/src/main.cpp

CMakeFiles/tpShadow.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tpShadow.dir/src/main.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/weixiang/Downloads/TP00-Loop/src/main.cpp > CMakeFiles/tpShadow.dir/src/main.cpp.i

CMakeFiles/tpShadow.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tpShadow.dir/src/main.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/weixiang/Downloads/TP00-Loop/src/main.cpp -o CMakeFiles/tpShadow.dir/src/main.cpp.s

CMakeFiles/tpShadow.dir/src/Mesh.cpp.o: CMakeFiles/tpShadow.dir/flags.make
CMakeFiles/tpShadow.dir/src/Mesh.cpp.o: ../src/Mesh.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/weixiang/Downloads/TP00-Loop/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/tpShadow.dir/src/Mesh.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/tpShadow.dir/src/Mesh.cpp.o -c /Users/weixiang/Downloads/TP00-Loop/src/Mesh.cpp

CMakeFiles/tpShadow.dir/src/Mesh.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tpShadow.dir/src/Mesh.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/weixiang/Downloads/TP00-Loop/src/Mesh.cpp > CMakeFiles/tpShadow.dir/src/Mesh.cpp.i

CMakeFiles/tpShadow.dir/src/Mesh.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tpShadow.dir/src/Mesh.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/weixiang/Downloads/TP00-Loop/src/Mesh.cpp -o CMakeFiles/tpShadow.dir/src/Mesh.cpp.s

CMakeFiles/tpShadow.dir/src/ShaderProgram.cpp.o: CMakeFiles/tpShadow.dir/flags.make
CMakeFiles/tpShadow.dir/src/ShaderProgram.cpp.o: ../src/ShaderProgram.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/weixiang/Downloads/TP00-Loop/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/tpShadow.dir/src/ShaderProgram.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/tpShadow.dir/src/ShaderProgram.cpp.o -c /Users/weixiang/Downloads/TP00-Loop/src/ShaderProgram.cpp

CMakeFiles/tpShadow.dir/src/ShaderProgram.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tpShadow.dir/src/ShaderProgram.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/weixiang/Downloads/TP00-Loop/src/ShaderProgram.cpp > CMakeFiles/tpShadow.dir/src/ShaderProgram.cpp.i

CMakeFiles/tpShadow.dir/src/ShaderProgram.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tpShadow.dir/src/ShaderProgram.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/weixiang/Downloads/TP00-Loop/src/ShaderProgram.cpp -o CMakeFiles/tpShadow.dir/src/ShaderProgram.cpp.s

# Object files for target tpShadow
tpShadow_OBJECTS = \
"CMakeFiles/tpShadow.dir/src/main.cpp.o" \
"CMakeFiles/tpShadow.dir/src/Mesh.cpp.o" \
"CMakeFiles/tpShadow.dir/src/ShaderProgram.cpp.o"

# External object files for target tpShadow
tpShadow_EXTERNAL_OBJECTS =

tpShadow: CMakeFiles/tpShadow.dir/src/main.cpp.o
tpShadow: CMakeFiles/tpShadow.dir/src/Mesh.cpp.o
tpShadow: CMakeFiles/tpShadow.dir/src/ShaderProgram.cpp.o
tpShadow: CMakeFiles/tpShadow.dir/build.make
tpShadow: dep/glad/libglad.a
tpShadow: /usr/local/lib/libglfw.3.3.dylib
tpShadow: CMakeFiles/tpShadow.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/weixiang/Downloads/TP00-Loop/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable tpShadow"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tpShadow.dir/link.txt --verbose=$(VERBOSE)
	/usr/local/Cellar/cmake/3.18.3/bin/cmake -E copy /Users/weixiang/Downloads/TP00-Loop/build/tpShadow /Users/weixiang/Downloads/TP00-Loop

# Rule to build all files generated by this target.
CMakeFiles/tpShadow.dir/build: tpShadow

.PHONY : CMakeFiles/tpShadow.dir/build

CMakeFiles/tpShadow.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/tpShadow.dir/cmake_clean.cmake
.PHONY : CMakeFiles/tpShadow.dir/clean

CMakeFiles/tpShadow.dir/depend:
	cd /Users/weixiang/Downloads/TP00-Loop/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/weixiang/Downloads/TP00-Loop /Users/weixiang/Downloads/TP00-Loop /Users/weixiang/Downloads/TP00-Loop/build /Users/weixiang/Downloads/TP00-Loop/build /Users/weixiang/Downloads/TP00-Loop/build/CMakeFiles/tpShadow.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/tpShadow.dir/depend
