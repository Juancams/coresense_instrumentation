cmake_minimum_required(VERSION 3.5)
project(coresense_instrumentation_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(coresense_instrumentation_interfaces REQUIRED)

set(dependencies
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  std_msgs
  sensor_msgs
  geometry_msgs
  image_transport
  coresense_instrumentation_interfaces
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/coresense_instrumentation_driver/InstrumentationProducerGeneric.cpp
  src/coresense_instrumentation_driver/InstrumentationProducerImage.cpp
  src/coresense_instrumentation_driver/InstrumentationConsumerGeneric.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

rclcpp_components_register_nodes(${PROJECT_NAME}
  "coresense_instrumentation_driver::InstrumentationProducer<std_msgs::msg::String>"
  "coresense_instrumentation_driver::InstrumentationProducer<sensor_msgs::msg::LaserScan>"
  "coresense_instrumentation_driver::InstrumentationProducer<sensor_msgs::msg::Image>"
  "coresense_instrumentation_driver::InstrumentationProducer<geometry_msgs::msg::Twist>"
  "coresense_instrumentation_driver::InstrumentationConsumer<std_msgs::msg::String>"
  "coresense_instrumentation_driver::InstrumentationConsumer<sensor_msgs::msg::LaserScan>"
  "coresense_instrumentation_driver::InstrumentationConsumer<sensor_msgs::msg::Image>"
  "coresense_instrumentation_driver::InstrumentationConsumer<geometry_msgs::msg::Twist>"
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_package()
