# Requires at least CMake 2.6 to configure the package.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(CXX_DISABLE_WERROR TRUE)
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/kineo.cmake)

SET(PROJECT_NAME kcd-capsule)
SET(PROJECT_VERSION 0.1)
SET(PROJECT_DESCRIPTION "Implementation of capsule objects in Kineo Collision Detection library.")
SET(PROJECT_URL "")

# Where to compile shared objects
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

SET(${PROJECT_NAME}_HEADERS
  include/kcd/fwd.hh
  include/kcd/capsule.hh
  include/kcd/poly-capsule.hh
  include/kcd/test-tree-capsule.hh
  include/kcd/detector-capsule-capsule.hh
  include/kcd/segment.hh
  include/kcd/poly-segment.hh
  include/kcd/test-tree-segment.hh
  include/kcd/detector-segment-segment.hh
)

SETUP_PROJECT()

# Declare dependencies
SEARCH_FOR_BOOST()
ADD_REQUIRED_DEPENDENCY("KineoController >= 2.06")

# Search for GeometricTools library.
SET(GEOMETRIC_TOOLS_INSTALL_PREFIX "" CACHE PATH "GeometricTools installation prefix")
SET(LIB_GEOMETRIC_TOOLS_CORE LIB_GEOMETRIC_TOOLS_CORE-NOTFOUND)
SET(LIB_GEOMETRIC_TOOLS_MATH LIB_GEOMETRIC_TOOLS_MATH-NOTFOUND)
FIND_LIBRARY(LIB_GEOMETRIC_TOOLS_CORE
  libWm5Core.so
  PATH
  ${GEOMETRIC_TOOLS_INSTALL_PREFIX}/lib)
IF (NOT LIB_GEOMETRIC_TOOLS_CORE)
  MESSAGE(FATAL_ERROR
    "Failed to find GeometricTools Core library, check that GeometricTools is installed and set the GEOMETRIC_TOOLS_INSTALL_PREFIX CMake variable.")
ENDIF()
FIND_LIBRARY(LIB_GEOMETRIC_TOOLS_MATH
  libWm5Mathematics.so
  PATH
  ${GEOMETRIC_TOOLS_INSTALL_PREFIX}/lib)
IF (NOT LIB_GEOMETRIC_TOOLS_MATH)
  MESSAGE(FATAL_ERROR
    "Failed to find GeometricTools Mathematics library, check that GeometricTools is installed and set the GEOMETRIC_TOOLS_INSTALL_PREFIX CMake variable.")
ENDIF()
SET(GEOMETRIC_TOOLS_H GEOMETRIC_TOOLS-NOTFOUND)
FIND_PATH (GEOMETRIC_TOOLS_H
  Wm5DistSegment3Segment3.h
  "${GEOMETRIC_TOOLS_INSTALL_PREFIX}/include/geometric-tools")
IF (NOT GEOMETRIC_TOOLS_H)
  MESSAGE(FATAL_ERROR
    "Failed to find geometric-tools/Wm5DistSegment3Segment3.h, check that Ipopt is installed.")
ENDIF()
INCLUDE_DIRECTORIES("${GEOMETRIC_TOOLS_INSTALL_PREFIX}/include")
LINK_DIRECTORIES("${GEOMETRIC_TOOLS_INSTALL_PREFIX}/lib")

PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(tests)

SETUP_PROJECT_FINALIZE()
