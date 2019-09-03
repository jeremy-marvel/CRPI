# Install script for directory: C:/CRPI/Libraries/Estimate_Pose_Aruco/aruco-1.2.4/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/CRPI/Libraries/Estimate_Pose_Aruco/aruco-1.2.4/build")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "main" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "C:/CRPI/Libraries/Estimate_Pose_Aruco/aruco-1.2.4/build/bin/Debug/aruco124.lib")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "C:/CRPI/Libraries/Estimate_Pose_Aruco/aruco-1.2.4/build/bin/Release/aruco124.lib")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "C:/CRPI/Libraries/Estimate_Pose_Aruco/aruco-1.2.4/build/bin/MinSizeRel/aruco124.lib")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "C:/CRPI/Libraries/Estimate_Pose_Aruco/aruco-1.2.4/build/bin/RelWithDebInfo/aruco124.lib")
  endif()
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "main" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "C:/CRPI/Libraries/Estimate_Pose_Aruco/aruco-1.2.4/build/bin/Debug/aruco124.dll")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "C:/CRPI/Libraries/Estimate_Pose_Aruco/aruco-1.2.4/build/bin/Release/aruco124.dll")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "C:/CRPI/Libraries/Estimate_Pose_Aruco/aruco-1.2.4/build/bin/MinSizeRel/aruco124.dll")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "C:/CRPI/Libraries/Estimate_Pose_Aruco/aruco-1.2.4/build/bin/RelWithDebInfo/aruco124.dll")
  endif()
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "main" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/aruco" TYPE FILE FILES
    "C:/CRPI/Libraries/Estimate_Pose_Aruco/aruco-1.2.4/src/aruco.h"
    "C:/CRPI/Libraries/Estimate_Pose_Aruco/aruco-1.2.4/src/arucofidmarkers.h"
    "C:/CRPI/Libraries/Estimate_Pose_Aruco/aruco-1.2.4/src/board.h"
    "C:/CRPI/Libraries/Estimate_Pose_Aruco/aruco-1.2.4/src/boarddetector.h"
    "C:/CRPI/Libraries/Estimate_Pose_Aruco/aruco-1.2.4/src/cameraparameters.h"
    "C:/CRPI/Libraries/Estimate_Pose_Aruco/aruco-1.2.4/src/cvdrawingutils.h"
    "C:/CRPI/Libraries/Estimate_Pose_Aruco/aruco-1.2.4/src/exports.h"
    "C:/CRPI/Libraries/Estimate_Pose_Aruco/aruco-1.2.4/src/marker.h"
    "C:/CRPI/Libraries/Estimate_Pose_Aruco/aruco-1.2.4/src/markerdetector.h"
    )
endif()

