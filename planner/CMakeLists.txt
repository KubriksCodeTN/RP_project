cmake_minimum_required(VERSION 3.8)
project(planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(obstacles_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(visilibity1
  "include/VisiLibity1/src/visilibity.cpp"
)
target_compile_features(visilibity1 PUBLIC c_std_99 cxx_std_17)
target_compile_options(visilibity1 PRIVATE -w)
target_include_directories(visilibity1 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/VisiLibity1/src>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

add_library(multi_dubins
  "include/MultiDubins/multi_dubins.cpp"
  "include/MultiDubins/dubins.cpp"
)
target_compile_features(multi_dubins PUBLIC c_std_99 cxx_std_20)
target_include_directories(multi_dubins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/MultiDubins/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/MultiDubins>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

add_library(clipper2
  "include/CPP/Clipper2Lib/src/clipper.engine.cpp" 
  "include/CPP/Clipper2Lib/src/clipper.offset.cpp"
  "include/CPP/Clipper2Lib/src/clipper.rectclip.cpp"
)
target_compile_features(clipper2 PUBLIC c_std_99 cxx_std_17)
target_include_directories(clipper2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/CPP/Clipper2Lib/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

add_library(offsetting src/offsetting.cpp)
target_link_libraries(offsetting clipper2)
target_compile_features(offsetting PUBLIC c_std_99 cxx_std_20)
target_include_directories(offsetting PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/CPP/Clipper2Lib/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/VisiLibity1/src>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

add_library(planner_node src/planner.cpp)
ament_target_dependencies(planner_node rclcpp nav_msgs nav2_msgs tf2 rclcpp_action 
  std_srvs rclcpp_components obstacles_msgs geometry_msgs)
target_compile_features(planner_node PUBLIC c_std_99 cxx_std_20)
target_include_directories(planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/CPP/Clipper2Lib/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/VisiLibity1/src>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/MultiDubins>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

add_executable(planner src/main.cpp)
target_link_libraries(planner planner_node tbb visilibity1 multi_dubins offsetting)
target_compile_features(planner PUBLIC c_std_99 cxx_std_20)  
target_include_directories(planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

install(TARGETS planner
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()
