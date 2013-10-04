SET(LIBRARY_NAME ${PROJECT_NAME})

ADD_LIBRARY(${LIBRARY_NAME}
  SHARED
  collision/util.cc
  collision/capsule.cc
  collision/poly-capsule.cc
  collision/test-tree-capsule.cc
  collision/detector-capsule-capsule.cc
  collision/detector-capsule-triangle.cc
  collision/detector-triangle-capsule.cc
  collision/detector-capsule-obb.cc
  collision/detector-obb-capsule.cc
  collision/segment.cc
  collision/poly-segment.cc
  collision/test-tree-segment.cc
  collision/detector-segment-segment.cc
  collision/detector-segment-obb.cc
  collision/detector-obb-segment.cc
  collision/detector-segment-triangle.cc
  collision/detector-triangle-segment.cc
  collision/detector-capsule-segment.cc
  collision/detector-segment-capsule.cc
  collision/detector-capsule-box.cc
  collision/detector-box-capsule.cc
  component/util.cc
  component/capsule.cc
  component/segment.cc
  )

SET_TARGET_PROPERTIES(${LIBRARY_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})

PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} KineoController)
TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${GEOMETRIC_TOOLS_LIBRARIES})

INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION lib)
