#*********************************************************#
#*  File: Android.cmake                                    
#*    Android apk tools
#*
#*********************************************************#

IF(NOT ANDROID_SDK)
  MESSAGE(FATAL_ERROR "ANDROID_SDK variable not set")
ENDIF()

set(ANDROID_APK_API_LEVEL "14" CACHE STRING "Android APK API level")
set(ANDROID_APK_INSTALL "0" CACHE BOOL "Install created apk file on the device automatically?")
set(ANDROID_APK_RUN "0" CACHE BOOL "Run created apk file on the device automatically? (installs it automatically as well, \"ANDROID_APK_INSTALL\"-option is ignored)")
set(ANDROID_APK_SIGNER_KEYSTORE	"~/releasekeys.keystore" CACHE STRING "Keystore for signing the apk file (only required for release apk)")
set(ANDROID_APK_SIGNER_ALIAS "username" CACHE STRING "Alias for signing the apk file (only required for release apk)")

##################################################
## Variables
##################################################
set(ANDROID_THIS_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})	# Directory this CMake file is in

##################################################
## MACRO: android_create_apk
##
## Create/copy Android apk related files
##
## @param name
##   Name of the project (e.g. "MyProject"), this will also be the name of the created apk file
## @param apk_pacakge_name
##   Pacakge name of the application
## @param apk_directory
##   Directory were to construct the apk file in (e.g. "${CMAKE_BINARY_DIR}/apk")
##   
## @remarks
##   Requires the following tools to be found automatically
##   - "android" (part of the Android SDK)
##   - "adb" (part of the Android SDK)
##   - "ant" (type e.g. "sudo apt-get install ant" on your Linux system to install Ant)
##   - "jarsigner" (part of the JDK)
##   - "zipalign" (part of the Android SDK)
##################################################
macro(android_create_apk name apk_package_name apk_directory)
  set(ANDROID_NAME ${name})
  set(ANDROID_APK_PACKAGE ${apk_package_name})

  if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/AndroidManifest.xml")
    SET(CUSTOM_MANIFEST "true")
  else()
    SET(CUSTOM_MANIFEST "false")
  endif()


  # Create the directory for the libraries
  add_custom_command(TARGET ${ANDROID_NAME} PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E remove_directory "${apk_directory}/libs")
  add_custom_command(TARGET ${ANDROID_NAME} PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${apk_directory}/libs")
  add_custom_command(TARGET ${ANDROID_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${LIBRARY_OUTPUT_PATH_ROOT}/libs" "${apk_directory}/libs/")
  
  # Create "build.xml", "default.properties", "local.properties" and "proguard.cfg" files
  if(CMAKE_BUILD_TYPE MATCHES Release)
    set(ANDROID_APK_DEBUGGABLE "false")
  else()
    set(ANDROID_APK_DEBUGGABLE "true")
  endif()

  file(GLOB_RECURSE JAVA_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.java")

  if (CUSTOM_MANIFEST)
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/AndroidManifest.xml" "${apk_directory}/AndroidManifest.xml")
  else()
    configure_file("${ANDROID_THIS_DIRECTORY}/AndroidManifest.xml.in" "${apk_directory}/AndroidManifest.xml")
  endif()

  # Copy assets
  add_custom_command(TARGET ${ANDROID_NAME} PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E remove_directory "${apk_directory}/assets")
  add_custom_command(TARGET ${ANDROID_NAME} PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${apk_directory}/assets/")
  add_custom_command(TARGET ${ANDROID_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${CMAKE_SOURCE_DIR}/assets" "${apk_directory}/assets/")

  # Build the apk file
  if(CMAKE_BUILD_TYPE MATCHES Release)
    # Let Ant create the unsigned apk file
    SET(ANDROID_APK ${apk_directory}/bin/${ANDROID_NAME}.apk)

    add_custom_command(OUTPUT ${ANDROID_APK}
      COMMAND ${CMAKE_COMMAND} -E make_directory "${apk_directory}/res"
      COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/res" "${apk_directory}/res/"
      COMMAND ${CMAKE_COMMAND} -E make_directory "${apk_directory}/src"
      COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/src" "${apk_directory}/src/"
      COMMAND ${ANDROID_SDK}/tools/android update project -t android-${ANDROID_APK_API_LEVEL} --name ${ANDROID_NAME} --path "${apk_directory}"
      COMMAND ant release
      WORKING_DIRECTORY "${apk_directory}" DEPENDS ${ANDROID_NAME} ${JAVA_SOURCES} COMMENT "Building Java wrapper")

    # Sign the apk file
    add_custom_command(TARGET ${ANDROID_NAME}App
      COMMAND jarsigner -verbose -keystore ${ANDROID_APK_SIGNER_KEYSTORE} bin/${ANDROID_NAME}-unsigned.apk ${ANDROID_APK_SIGNER_ALIAS}
      WORKING_DIRECTORY "${apk_directory}")

    # Align the apk file
    add_custom_command(TARGET ${ANDROID_NAME}App
      COMMAND zipalign -v -f 4 bin/${ANDROID_NAME}-unsigned.apk bin/${ANDROID_NAME}.apk
      WORKING_DIRECTORY "${apk_directory}")
    
  else()
    SET(ANDROID_APK ${apk_directory}/bin/${ANDROID_NAME}-debug.apk)

    # Let Ant create the unsigned apk file
    add_custom_command(OUTPUT "${ANDROID_APK}"
      COMMAND ${CMAKE_COMMAND} -E make_directory "${apk_directory}/res"
      COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/res" "${apk_directory}/res/"
      COMMAND ${CMAKE_COMMAND} -E make_directory "${apk_directory}/src"
      COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/src" "${apk_directory}/src/"
      COMMAND ${ANDROID_SDK}/tools/android update project -t android-${ANDROID_APK_API_LEVEL} --name ${ANDROID_NAME} --path "${apk_directory}"
      COMMAND ant debug
      WORKING_DIRECTORY "${apk_directory}" DEPENDS ${ANDROID_NAME} ${JAVA_SOURCES} COMMENT "Building Java wrapper")
  endif()

  add_custom_target(${ANDROID_NAME}App ALL DEPENDS ${ANDROID_NAME} ${ANDROID_APK})
  
  add_custom_target(${ANDROID_NAME}AppInstall DEPENDS ${ANDROID_NAME}App)
  add_custom_command(TARGET ${ANDROID_NAME}AppInstall COMMAND ${ANDROID_SDK}/platform-tools/adb install -r ${ANDROID_APK} WORKING_DIRECTORY "${apk_directory}")

  add_custom_target(${ANDROID_NAME}AppRun DEPENDS ${ANDROID_NAME}AppInstall)
  add_custom_command(TARGET ${ANDROID_NAME}AppRun COMMAND ${ANDROID_SDK}/platform-tools/adb adb shell monkey -p ${ANDROID_APK_PACKAGE} 1)

endmacro(android_create_apk name apk_directory)
