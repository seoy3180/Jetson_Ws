cmake_minimum_required(VERSION 3.8)
project(ff_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)  # 메시지 생성 패키지
find_package(geometry_msgs REQUIRED)             # geometry_msgs 의존성 추가
find_package(rosidl_typesupport_c REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArucoDetections.msg"
  "srv/ConveyorRun.srv"
  "srv/ConveyorConnection.srv"
  "srv/SelectedJob.srv"
  "srv/BoxDetectandPick.srv"
  "action/Job.action"
  DEPENDENCIES geometry_msgs
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

ament_export_dependencies(rosidl_default_runtime)
ament_package()
