cmake_minimum_required(VERSION 3.8)
project(teleoptocantransformer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sa_msgs REQUIRED)
find_package(Protobuf REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 查找 nlohmann_json（如果找不到，使用简单的字符串解析）
find_package(nlohmann_json QUIET)
if(nlohmann_json_FOUND)
  message(STATUS "Found nlohmann_json")
  add_definitions(-DUSE_NLOHMANN_JSON)
else()
  message(WARNING "nlohmann_json not found, using simple JSON parser")
endif()

# 设置 protobuf 路径（使用 cannode 的 protobuf）
set(CANNODE_PROTOBUF_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../cannode/protobuf/out)
include_directories(${CANNODE_PROTOBUF_DIR})
include_directories(${PROTOBUF_INCLUDE_DIRS})

# 查找所有需要的 protobuf 源文件（递归查找所有 .pb.cc 文件）
file(GLOB_RECURSE PROTOBUF_SOURCES
  "${CANNODE_PROTOBUF_DIR}/*.pb.cc"
)

# 生成自定义消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VehicleCommand.msg"
  DEPENDENCIES std_msgs
)

# 添加可执行文件
add_executable(teleop2can_transformer 
  src/teleop_converter.cpp
  ${PROTOBUF_SOURCES}
)

# 链接库
target_link_libraries(teleop2can_transformer
  ${PROTOBUF_LIBRARIES}
)

# 添加依赖
ament_target_dependencies(teleop2can_transformer
  rclcpp
  std_msgs
  sa_msgs
)

# 链接消息库
rosidl_target_interfaces(teleop2can_transformer
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 安装
install(TARGETS
  teleop2can_transformer
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
