ament_add_gtest(A1_wrapper_test A1_wrapper_test.cpp)
ament_target_dependencies(A1_wrapper_test ${dependencies})
target_link_libraries(A1_wrapper_test
  a1_ros
  lcm)
find_package(Boost COMPONENTS system REQUIRED)
ament_add_gtest(A1_ros_test A1_ros_test.cpp)
ament_target_dependencies(A1_ros_test ${dependencies})
target_link_libraries(A1_ros_test
  a1_ros
  lcm
  boost_thread
  boost_system)
