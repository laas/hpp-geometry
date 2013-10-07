# Requires at least CMake 2.6 to configure the package.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(CXX_DISABLE_WERROR TRUE)
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/kineo.cmake)
INCLUDE(cmake/geometric-tools.cmake)

SET(PROJECT_NAME hpp-geometry)
SET(PROJECT_DESCRIPTION
  "Implementation of capsule objects in Kineo Collision Detection library.")
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
  include/hpp/geometry/collision/detector-segment-capsule.hh
  include/hpp/geometry/collision/detector-capsule-segment.hh
  include/hpp/geometry/collision/detector-capsule-box.hh
  include/hpp/geometry/collision/detector-box-capsule.hh
  )

SET(${PROJECT_NAME}_COMPONENT_HEADERS
  include/hpp/geometry/component/fwd.hh
  include/hpp/geometry/component/util.hh
  include/hpp/geometry/component/capsule.hh
  include/hpp/geometry/component/segment.hh
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
SEARCH_FOR_GEOMETRIC_TOOLS()
ADD_REQUIRED_DEPENDENCY("KineoController >= 2.07")
ADD_REQUIRED_DEPENDENCY("hpp-util >= 0.7")

PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(tests)

SETUP_PROJECT_FINALIZE()
