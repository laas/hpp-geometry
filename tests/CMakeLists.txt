# Add Boost path to include directories.
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

# Make Boost.Test generates the main function in test cases.
ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)

# ADD_TESTCASE(NAME)
# ------------------------
#
# Define a test named `NAME'.
#
# This macro will create a binary from `NAME.cc', link it against
# Boost and add it to the test suite.
#
MACRO(ADD_TESTCASE NAME)
  ADD_EXECUTABLE(${NAME} ${CMAKE_CURRENT_SOURCE_DIR}/${NAME}.cc)
  ADD_TEST(${NAME} ${RUNTIME_OUTPUT_DIRECTORY}/${NAME})

  PKG_CONFIG_USE_DEPENDENCY(${NAME} KineoController)

  # Link against package library.
  TARGET_LINK_LIBRARIES(${NAME} 
    ${Boost_LIBRARIES}
    ${PROJECT_NAME})
ENDMACRO(ADD_TESTCASE)

# Generated test.
ADD_TESTCASE(collision-detection-capsule)
KINEO_STANDALONE(collision-detection-capsule)

ADD_TESTCASE(collision-detection-capsule-obb)
KINEO_STANDALONE(collision-detection-capsule-obb)

ADD_TESTCASE(proximity-query-capsule)
KINEO_STANDALONE(proximity-query-capsule)

ADD_TESTCASE(proximity-query-capsule-obb)
KINEO_STANDALONE(proximity-query-capsule-obb)

ADD_TESTCASE(proximity-query-segment)
KINEO_STANDALONE(proximity-query-segment)

ADD_TESTCASE(proximity-query-segment-obb)
KINEO_STANDALONE(proximity-query-segment-obb)

ADD_TESTCASE(gradient-segment-obb)
KINEO_STANDALONE(gradient-segment-obb)

ADD_TESTCASE(component-capsule)
KINEO_STANDALONE(component-capsule)

ADD_TESTCASE(component-segment)
KINEO_STANDALONE(component-segment)

ADD_TESTCASE(component-capsule-collision)
KINEO_STANDALONE(component-capsule-collision)

ADD_TESTCASE(bounding-capsule)
KINEO_STANDALONE(bounding-capsule)

ADD_TESTCASE(component-segment-proximity-query)
KINEO_STANDALONE(component-segment-proximity-query)

ADD_TESTCASE(segment-proximity-query-performance)
KINEO_STANDALONE(segment-proximity-query-performance)

ADD_TESTCASE(segment-obb-proximity-query-performance)
KINEO_STANDALONE(segment-obb-proximity-query-performance)
