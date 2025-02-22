cmake_minimum_required(VERSION 3.8)
project(safe_corridor_generator)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# 包含目录
include_directories(
  ${PROJECT_SOURCE_DIR}/include/safe_corridor_generator
)
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/safe_corridor_generator.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN safe_corridor_generator::SafeCorridorGenerator
  EXECUTABLE safe_corridor_generator_node
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()


install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)


ament_auto_package()