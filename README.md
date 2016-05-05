ViCoS Lab ROS packages
======================

This repository contains various ROS packages, created in Visual Cognitive Systems Laboratory, Faculty of Computer and Information Science, University of Ljubljana, Slovenia mainly to be used withing our courses on mobile robotics.


Detection
---------

 * detection_msgs - Common object detection resources and nodes.
 * opencv_detector - Integration of OpenCV object detection using cascades in ROS.
 * dlib_detector - Integration of dlib object detection using HOG features in ROS.
 * ferns_detector - Simple integration of the [Ferns detector of planar objects](http://cvlab.epfl.ch/software/ferns/index.php) demo code to ROS.
 * zbar_detector - Integration of the [ZBar bar code reader library](http://zbar.sourceforge.net/) to ROS. The primary purpose of the package is to enable mobile robots to read QR codes from video streams.

Depth
-----

 * localizer - A package that determines depth of RGB camera detections using depth images.
 * pcl_demos - A collection of PCL demos.

Communication
-------------

 * speech_proxy - A simple proxy for external speech recognition software using TCP server.
 * voice_navigation - Navigate robot using voice commands.
 * julius_ros - Integration of the [Julius speech recognition](http://julius.sourceforge.jp/en_index.php) to ROS. The package is designed as a simple replacement to the existing [pocketsphinx package](http://wiki.ros.org/pocketsphinx).

Navigation
----------

 * costmap_layers - Custom layers for navigation costmap.

turtlebot_vicos
---------------

Support launch files and scripts that help us set up our Turtlebots.
