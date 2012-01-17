# Requires at least CMake 2.6 to configure the package.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(CXX_DISABLE_WERROR TRUE)
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/kineo.cmake)

SET(PROJECT_NAME hpp-geometry)
SET(PROJECT_VERSION 1.0)
SET(PROJECT_DESCRIPTION "Implementation of capsule objects in Kineo Collision Detection library.")
SET(PROJECT_URL "")

# Where to compile shared objects
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

SET(${PROJECT_NAME}_COLLISION_HEADERS
  include/hpp/geometry/collision/fwd.hh
  include/hpp/geometry/collision/util.hh
  include/hpp/geometry/collision/capsule.hh
  include/hpp/geometry/collision/poly-capsule.hh
  include/hpp/geometry/collision/test-tree-capsule.hh
  include/hpp/geometry/collision/detector-capsule-capsule.hh
  include/hpp/geometry/collision/detector-capsule-obb.hh
  include/hpp/geometry/collision/detector-obb-capsule.hh
  include/hpp/geometry/collision/detector-capsule-triangle.hh
  include/hpp/geometry/collision/detector-triangle-capsule.hh
  include/hpp/geometry/collision/segment.hh
  include/hpp/geometry/collision/poly-segment.hh
  include/hpp/geometry/collision/test-tree-segment.hh
  include/hpp/geometry/collision/detector-segment-segment.hh
  include/hpp/geometry/collision/detector-segment-obb.hh
  include/hpp/geometry/collision/detector-obb-segment.hh
  include/hpp/geometry/collision/detector-segment-triangle.hh
  include/hpp/geometry/collision/detector-triangle-segment.hh
  )

SET(${PROJECT_NAME}_COMPONENT_HEADERS
  include/hpp/geometry/component/fwd.hh
  include/hpp/geometry/component/util.hh
  include/hpp/geometry/component/capsule.hh
  )

INSTALL(FILES ${${PROJECT_NAME}_COLLISION_HEADERS}
  DESTINATION include/hpp/geometry/collision
  )

INSTALL(FILES ${${PROJECT_NAME}_COMPONENT_HEADERS}
  DESTINATION include/hpp/geometry/component
  )

SETUP_PROJECT()

# Declare dependencies
SEARCH_FOR_BOOST()
ADD_REQUIRED_DEPENDENCY("KineoController >= 2.06")

# Search for GeometricTools library.
SET(GEOMETRIC_TOOLS_INSTALL_PREFIX "" CACHE PATH "geometric-tools installation prefix")
SET(LIB_GEOMETRIC_TOOLS_CORE LIB_GEOMETRIC_TOOLS_CORE-NOTFOUND)
SET(LIB_GEOMETRIC_TOOLS_MATH LIB_GEOMETRIC_TOOLS_MATH-NOTFOUND)
FIND_LIBRARY(LIB_GEOMETRIC_TOOLS_CORE
  libWm5Core.so
  PATH
  ${GEOMETRIC_TOOLS_INSTALL_PREFIX}/lib)
IF (NOT LIB_GEOMETRIC_TOOLS_CORE)
  MESSAGE(FATAL_ERROR
    "Failed to find geometric-tools Core library, check that geometric-tools is installed and set the GEOMETRIC_TOOLS_INSTALL_PREFIX CMake variable.")
ENDIF()
FIND_LIBRARY(LIB_GEOMETRIC_TOOLS_MATH
  libWm5Mathematics.so
  PATH
  ${GEOMETRIC_TOOLS_INSTALL_PREFIX}/lib)
IF (NOT LIB_GEOMETRIC_TOOLS_MATH)
  MESSAGE(FATAL_ERROR
    "Failed to find geometric-tools Mathematics library, check that geometric-tools is installed and set the GEOMETRIC_TOOLS_INSTALL_PREFIX CMake variable.")
ENDIF()
SET(GEOMETRIC_TOOLS_H GEOMETRIC_TOOLS-NOTFOUND)
FIND_PATH (GEOMETRIC_TOOLS_H
  Wm5DistSegment3Segment3.h
  "${GEOMETRIC_TOOLS_INSTALL_PREFIX}/include/geometric-tools")
IF (NOT GEOMETRIC_TOOLS_H)
  MESSAGE(FATAL_ERROR
    "Failed to find geometric-tools/Wm5DistSegment3Segment3.h, check that geometric-tools is installed.")
ENDIF()
INCLUDE_DIRECTORIES("${GEOMETRIC_TOOLS_INSTALL_PREFIX}/include")
LINK_DIRECTORIES("${GEOMETRIC_TOOLS_INSTALL_PREFIX}/lib")

PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(tests)

SETUP_PROJECT_FINALIZE()
