# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/hanc/Code/mynt/mynteye-1.8-linux-x64-gcc5-opencv-3.4.0/samples

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hanc/Code/mynt/mynteye-1.8-linux-x64-gcc5-opencv-3.4.0/samples/build

# Include any dependencies generated for this target.
include CMakeFiles/camera_kitti.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/camera_kitti.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/camera_kitti.dir/flags.make

CMakeFiles/camera_kitti.dir/src/camera_kitti.cc.o: CMakeFiles/camera_kitti.dir/flags.make
CMakeFiles/camera_kitti.dir/src/camera_kitti.cc.o: ../src/camera_kitti.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hanc/Code/mynt/mynteye-1.8-linux-x64-gcc5-opencv-3.4.0/samples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/camera_kitti.dir/src/camera_kitti.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/camera_kitti.dir/src/camera_kitti.cc.o -c /home/hanc/Code/mynt/mynteye-1.8-linux-x64-gcc5-opencv-3.4.0/samples/src/camera_kitti.cc

CMakeFiles/camera_kitti.dir/src/camera_kitti.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camera_kitti.dir/src/camera_kitti.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hanc/Code/mynt/mynteye-1.8-linux-x64-gcc5-opencv-3.4.0/samples/src/camera_kitti.cc > CMakeFiles/camera_kitti.dir/src/camera_kitti.cc.i

CMakeFiles/camera_kitti.dir/src/camera_kitti.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camera_kitti.dir/src/camera_kitti.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hanc/Code/mynt/mynteye-1.8-linux-x64-gcc5-opencv-3.4.0/samples/src/camera_kitti.cc -o CMakeFiles/camera_kitti.dir/src/camera_kitti.cc.s

CMakeFiles/camera_kitti.dir/src/camera_kitti.cc.o.requires:

.PHONY : CMakeFiles/camera_kitti.dir/src/camera_kitti.cc.o.requires

CMakeFiles/camera_kitti.dir/src/camera_kitti.cc.o.provides: CMakeFiles/camera_kitti.dir/src/camera_kitti.cc.o.requires
	$(MAKE) -f CMakeFiles/camera_kitti.dir/build.make CMakeFiles/camera_kitti.dir/src/camera_kitti.cc.o.provides.build
.PHONY : CMakeFiles/camera_kitti.dir/src/camera_kitti.cc.o.provides

CMakeFiles/camera_kitti.dir/src/camera_kitti.cc.o.provides.build: CMakeFiles/camera_kitti.dir/src/camera_kitti.cc.o


CMakeFiles/camera_kitti.dir/src/compat/compat.cc.o: CMakeFiles/camera_kitti.dir/flags.make
CMakeFiles/camera_kitti.dir/src/compat/compat.cc.o: ../src/compat/compat.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hanc/Code/mynt/mynteye-1.8-linux-x64-gcc5-opencv-3.4.0/samples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/camera_kitti.dir/src/compat/compat.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/camera_kitti.dir/src/compat/compat.cc.o -c /home/hanc/Code/mynt/mynteye-1.8-linux-x64-gcc5-opencv-3.4.0/samples/src/compat/compat.cc

CMakeFiles/camera_kitti.dir/src/compat/compat.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camera_kitti.dir/src/compat/compat.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hanc/Code/mynt/mynteye-1.8-linux-x64-gcc5-opencv-3.4.0/samples/src/compat/compat.cc > CMakeFiles/camera_kitti.dir/src/compat/compat.cc.i

CMakeFiles/camera_kitti.dir/src/compat/compat.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camera_kitti.dir/src/compat/compat.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hanc/Code/mynt/mynteye-1.8-linux-x64-gcc5-opencv-3.4.0/samples/src/compat/compat.cc -o CMakeFiles/camera_kitti.dir/src/compat/compat.cc.s

CMakeFiles/camera_kitti.dir/src/compat/compat.cc.o.requires:

.PHONY : CMakeFiles/camera_kitti.dir/src/compat/compat.cc.o.requires

CMakeFiles/camera_kitti.dir/src/compat/compat.cc.o.provides: CMakeFiles/camera_kitti.dir/src/compat/compat.cc.o.requires
	$(MAKE) -f CMakeFiles/camera_kitti.dir/build.make CMakeFiles/camera_kitti.dir/src/compat/compat.cc.o.provides.build
.PHONY : CMakeFiles/camera_kitti.dir/src/compat/compat.cc.o.provides

CMakeFiles/camera_kitti.dir/src/compat/compat.cc.o.provides.build: CMakeFiles/camera_kitti.dir/src/compat/compat.cc.o


CMakeFiles/camera_kitti.dir/src/dataset/dataset.cc.o: CMakeFiles/camera_kitti.dir/flags.make
CMakeFiles/camera_kitti.dir/src/dataset/dataset.cc.o: ../src/dataset/dataset.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hanc/Code/mynt/mynteye-1.8-linux-x64-gcc5-opencv-3.4.0/samples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/camera_kitti.dir/src/dataset/dataset.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/camera_kitti.dir/src/dataset/dataset.cc.o -c /home/hanc/Code/mynt/mynteye-1.8-linux-x64-gcc5-opencv-3.4.0/samples/src/dataset/dataset.cc

CMakeFiles/camera_kitti.dir/src/dataset/dataset.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camera_kitti.dir/src/dataset/dataset.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hanc/Code/mynt/mynteye-1.8-linux-x64-gcc5-opencv-3.4.0/samples/src/dataset/dataset.cc > CMakeFiles/camera_kitti.dir/src/dataset/dataset.cc.i

CMakeFiles/camera_kitti.dir/src/dataset/dataset.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camera_kitti.dir/src/dataset/dataset.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hanc/Code/mynt/mynteye-1.8-linux-x64-gcc5-opencv-3.4.0/samples/src/dataset/dataset.cc -o CMakeFiles/camera_kitti.dir/src/dataset/dataset.cc.s

CMakeFiles/camera_kitti.dir/src/dataset/dataset.cc.o.requires:

.PHONY : CMakeFiles/camera_kitti.dir/src/dataset/dataset.cc.o.requires

CMakeFiles/camera_kitti.dir/src/dataset/dataset.cc.o.provides: CMakeFiles/camera_kitti.dir/src/dataset/dataset.cc.o.requires
	$(MAKE) -f CMakeFiles/camera_kitti.dir/build.make CMakeFiles/camera_kitti.dir/src/dataset/dataset.cc.o.provides.build
.PHONY : CMakeFiles/camera_kitti.dir/src/dataset/dataset.cc.o.provides

CMakeFiles/camera_kitti.dir/src/dataset/dataset.cc.o.provides.build: CMakeFiles/camera_kitti.dir/src/dataset/dataset.cc.o


# Object files for target camera_kitti
camera_kitti_OBJECTS = \
"CMakeFiles/camera_kitti.dir/src/camera_kitti.cc.o" \
"CMakeFiles/camera_kitti.dir/src/compat/compat.cc.o" \
"CMakeFiles/camera_kitti.dir/src/dataset/dataset.cc.o"

# External object files for target camera_kitti
camera_kitti_EXTERNAL_OBJECTS =

../bin/camera_kitti: CMakeFiles/camera_kitti.dir/src/camera_kitti.cc.o
../bin/camera_kitti: CMakeFiles/camera_kitti.dir/src/compat/compat.cc.o
../bin/camera_kitti: CMakeFiles/camera_kitti.dir/src/dataset/dataset.cc.o
../bin/camera_kitti: CMakeFiles/camera_kitti.dir/build.make
../bin/camera_kitti: /home/hanc/Code/mynt/mynteye-1.8-linux-x64-gcc5-opencv-3.4.0/lib/libmynteye_core.so
../bin/camera_kitti: /usr/local/lib/libopencv_cudabgsegm.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_cudaobjdetect.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_cudastereo.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_stitching.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_superres.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_videostab.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_aruco.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_bgsegm.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_bioinspired.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_ccalib.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_dpm.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_face.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_freetype.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_fuzzy.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_hdf.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_img_hash.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_line_descriptor.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_optflow.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_reg.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_rgbd.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_saliency.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_stereo.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_structured_light.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_surface_matching.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_tracking.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_xfeatures2d.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_ximgproc.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_xobjdetect.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_xphoto.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_cudafeatures2d.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_shape.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_cudacodec.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_cudaoptflow.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_cudalegacy.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_cudawarping.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_photo.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_cudaimgproc.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_cudafilters.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_cudaarithm.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_datasets.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_plot.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_text.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_dnn.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_ml.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_video.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_calib3d.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_features2d.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_highgui.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_videoio.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_phase_unwrapping.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_flann.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_imgcodecs.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_objdetect.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_imgproc.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_core.so.3.4.0
../bin/camera_kitti: /usr/local/lib/libopencv_cudev.so.3.4.0
../bin/camera_kitti: CMakeFiles/camera_kitti.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hanc/Code/mynt/mynteye-1.8-linux-x64-gcc5-opencv-3.4.0/samples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable ../bin/camera_kitti"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/camera_kitti.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/camera_kitti.dir/build: ../bin/camera_kitti

.PHONY : CMakeFiles/camera_kitti.dir/build

CMakeFiles/camera_kitti.dir/requires: CMakeFiles/camera_kitti.dir/src/camera_kitti.cc.o.requires
CMakeFiles/camera_kitti.dir/requires: CMakeFiles/camera_kitti.dir/src/compat/compat.cc.o.requires
CMakeFiles/camera_kitti.dir/requires: CMakeFiles/camera_kitti.dir/src/dataset/dataset.cc.o.requires

.PHONY : CMakeFiles/camera_kitti.dir/requires

CMakeFiles/camera_kitti.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/camera_kitti.dir/cmake_clean.cmake
.PHONY : CMakeFiles/camera_kitti.dir/clean

CMakeFiles/camera_kitti.dir/depend:
	cd /home/hanc/Code/mynt/mynteye-1.8-linux-x64-gcc5-opencv-3.4.0/samples/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hanc/Code/mynt/mynteye-1.8-linux-x64-gcc5-opencv-3.4.0/samples /home/hanc/Code/mynt/mynteye-1.8-linux-x64-gcc5-opencv-3.4.0/samples /home/hanc/Code/mynt/mynteye-1.8-linux-x64-gcc5-opencv-3.4.0/samples/build /home/hanc/Code/mynt/mynteye-1.8-linux-x64-gcc5-opencv-3.4.0/samples/build /home/hanc/Code/mynt/mynteye-1.8-linux-x64-gcc5-opencv-3.4.0/samples/build/CMakeFiles/camera_kitti.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/camera_kitti.dir/depend

