cmake_minimum_required(VERSION 3.8)
project(multiple_orca)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
#find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Install project launch files
# install(
#   DIRECTORY
#     launch/
#   DESTINATION share/${PROJECT_NAME}/launch
# )

# # Install project configuration files
# install(
#   DIRECTORY
#     config/
#   DESTINATION share/${PROJECT_NAME}/config
# )

# Install scripts
install(
  PROGRAMS  
  scripts/mission_runner.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#ament_package()
ament_auto_package(INSTALL_TO_SHARE models worlds launch cfg params behavior_trees)