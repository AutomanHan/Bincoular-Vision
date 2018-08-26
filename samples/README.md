
# SDK Samples

About option `[name]`, which means the device name:

* On Windows, it must be MYNTEYE.
* On Linux/Mac, it must be a number, such as 0 and 1.

> On Linux/Mac, you could run `<sdk>/tools/list_devices` in the Terminal to see the indexes, and decide which number is MYNTEYE. Besides, you could also simply run `ls /dev/video*`.

## `src/camera.cc`

    camera [name]

* How to retrieve left & right images.

## `src/camera2.cc`

    camera2 [name] [calib config file]

* How to retrieve images, IMU, etc.
* How to retrieve depth map only when changed.
* How to scale the grabbed images.
* How to use callbacks to compute time cost.
* How to display kinds of informations on images.

> If do time-consuming tasks in main loop, you colud `ActivateAsyncGrabFeature()` to easily grab images in a standalone thread.

## `src/camera3.cc`

* How to show depth of point cloud.

## `src/camera_ctrl.cc`

* How to control gain, brightness, etc.

## `src/camera_kitti.cc`

    camera_kitti [name] [output directory]

* How to capture kitti dataset.
* How to reduce the fps from 60 to 20.

## `src/camera_test.cc`

    camera_test [name]

* Check the accuracy of the camera data.

## `src/camera_with_plugin.cc`

    camera_with_plugin [name]

* How to create plugin and process depth map by yourself.
