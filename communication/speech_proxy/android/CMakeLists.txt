cmake_minimum_required(VERSION 2.8)
PROJECT(VoiceCommander)

SET(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake) # Adding path to CMake scripts
INCLUDE(Android REQUIRED)

add_custom_target(VoiceCommander ALL)

ANDROID_CREATE_APK(VoiceCommander "si.vicos" "${CMAKE_BINARY_DIR}/apk")

