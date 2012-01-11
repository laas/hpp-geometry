SET(LIBRARY_NAME ${PROJECT_NAME})

ADD_LIBRARY(${LIBRARY_NAME}
  SHARED
  kcd/util.cc
  kcd/capsule.cc
  kcd/poly-capsule.cc
  kcd/test-tree-capsule.cc
  kcd/detector-capsule-capsule.cc
  kcd/detector-capsule-triangle.cc
  kcd/detector-triangle-capsule.cc
  kcd/detector-capsule-obb.cc
  kcd/detector-obb-capsule.cc
  kcd/segment.cc
  kcd/poly-segment.cc
  kcd/test-tree-segment.cc
  kcd/detector-segment-segment.cc
  kcd/detector-segment-obb.cc
  kcd/detector-obb-segment.cc
  kcd/detector-segment-triangle.cc
  kcd/detector-triangle-segment.cc
  kpp/util.cc
  kpp/component/capsule.cc
  )

SET_TARGET_PROPERTIES(${LIBRARY_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})

PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} KineoController)
TARGET_LINK_LIBRARIES(${LIBRARY_NAME}
  ${LIB_GEOMETRIC_TOOLS_CORE}
  ${LIB_GEOMETRIC_TOOLS_MATH})

INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION lib)
