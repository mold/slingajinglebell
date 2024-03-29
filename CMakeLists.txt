#-----------------------------------------------------------------------------
# CS277 - Experimental Haptics
#
# Homework assignment configuration file - use CMake (www.cmake.org) to
# generate a project file for your desired platform and environment.  The
# CHAI3D library directory (chai3d-2.1.0) should be installed as a sibling
# directory to the one containing this project.
#
# Created by Sonny Chan
# Edited by Adam Leeper
#	January 2011, Stanford University
#-----------------------------------------------------------------------------
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT(SlingAJingleBell)

SET(CMAKE_CONFIGURATION_TYPES "Debug;Release" 
    CACHE STRING "Available build configurations." FORCE)

#-----------------------------------------------------------------------------
# Library dependencies

FIND_PATH(CHAI3D_BASE
	NAMES	src/chai3d.h 
	PATHS	"/afs/nada.kth.se/home/u/u1avzhxu/public/chai3d-2.1.0" "../chai3d-2.0.0" ENV CHAI3D_BASE
)

INCLUDE_DIRECTORIES("${CHAI3D_BASE}/src")
INCLUDE_DIRECTORIES("${CHAI3D_BASE}/modules")
INCLUDE_DIRECTORIES("${CHAI3D_BASE}/external/ODE/include")
INCLUDE_DIRECTORIES("${CHAI3D_BASE}/external/ODE/lib")
    
#-----------------------------------------------------------------------------
# Platform-specific definitions and binary directory configuration.

IF(MSVC)
	ADD_DEFINITIONS(-D_MSVC -D_CRT_SECURE_NO_DEPRECATE)

	# Condition on MSVC version (a little bit)
	IF(MSVC80)
		LINK_DIRECTORIES("${CHAI3D_BASE}/lib/msvc8")
	ELSE(MSVC80)
		LINK_DIRECTORIES("${CHAI3D_BASE}/lib/msvc9")
	ENDIF(MSVC80)

	INCLUDE_DIRECTORIES("${CHAI3D_BASE}/external/OpenGL/msvc")
	LINK_DIRECTORIES("${CHAI3D_BASE}/external/OpenGL/msvc")

	CONFIGURE_FILE(${CHAI3D_BASE}/bin/hdFalcon.dll	${CMAKE_CURRENT_BINARY_DIR} COPYONLY)
	CONFIGURE_FILE(${CHAI3D_BASE}/bin/glut32.dll	${CMAKE_CURRENT_BINARY_DIR} COPYONLY)
	CONFIGURE_FILE(${CHAI3D_BASE}/bin/VirtualDevice.exe	${CMAKE_CURRENT_BINARY_DIR} COPYONLY)
ENDIF(MSVC)

IF(UNIX)
	IF (APPLE)
		ADD_DEFINITIONS(-D_MACOSX)
		FIND_LIBRARY(COREFOUNDATION_LIBRARY CoreFoundation)
		FIND_LIBRARY(IOKIT_LIBRARY IOKit)
		FIND_LIBRARY(OPENGL_LIBRARY OpenGL)
		FIND_LIBRARY(GLUT_LIBRARY GLUT)
		LINK_DIRECTORIES("${CHAI3D_BASE}/build" "${CHAI3D_BASE}/external/DHD/lib")
	ELSE(APPLE)
		ADD_DEFINITIONS(-D_LINUX)
		IF(${CMAKE_SIZEOF_VOID_P} EQUAL 8)
			SET(CHAI3D_LIBPATH "lin-x86_64")
		ELSE()
			SET(CHAI3D_LIBPATH "lin-i686")
		ENDIF()
		LINK_DIRECTORIES("${CHAI3D_BASE}/lib/${CHAI3D_LIBPATH}" "${CHAI3D_BASE}/external/DHD/lib/${CHAI3D_LIBPATH}")
	ENDIF(APPLE)
ENDIF(UNIX)

#-----------------------------------------------------------------------------
# Add project executable, source files, and dependencies

ADD_EXECUTABLE(slingajinglebell
	slingajinglebell.cpp
)

IF(MSVC)
	TARGET_LINK_LIBRARIES(Haptics
		debug		chai3d-debug
		optimized	chai3d-release
	)
ENDIF(MSVC)

IF (UNIX)
	IF(APPLE)
		TARGET_LINK_LIBRARIES(Haptics
			chai3d dhd
			${COREFOUNDATION_LIBRARY}
			${IOKIT_LIBRARY}
			${OPENGL_LIBRARY}
			${GLUT_LIBRARY}
		)
	ELSE(APPLE)
		TARGET_LINK_LIBRARIES(slingajinglebell
			chai3d dhd
			pthread rt usb-1.0
			GL GLU glut
		)
	ENDIF(APPLE)
ENDIF(UNIX)

#-----------------------------------------------------------------------------
