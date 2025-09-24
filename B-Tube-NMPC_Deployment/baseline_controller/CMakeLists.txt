cmake_minimum_required(VERSION 3.8)
project(tube_mpc)

# Absolute paths to generated acados code
get_filename_component(ACADOS_GENERATED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code" ABSOLUTE)
get_filename_component(ACADOS_CORE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../src/acados" ABSOLUTE)

# Check if the generated solver artifacts are present
if(NOT EXISTS "${ACADOS_GENERATED_DIR}/acados_solver_mirte.h")
    message(FATAL_ERROR "Missing generated solver header in ${ACADOS_GENERATED_DIR}")
endif()

if(NOT EXISTS "${ACADOS_GENERATED_DIR}/libacados_ocp_solver_mirte.so")
    message(FATAL_ERROR "Missing generated solver library in ${ACADOS_GENERATED_DIR}")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 & dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED) 
find_package(mirte_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Collect generated acados sources/headers
file(GLOB ACADOS_GENERATED_SRCS
    "${ACADOS_GENERATED_DIR}/*.c"
    "${ACADOS_GENERATED_DIR}/*.h"
)

# Build the acados shared lib inside the generated dir when needed
add_custom_command(
  OUTPUT ${ACADOS_GENERATED_DIR}/libacados_ocp_solver_mirte.so
  COMMAND make -C ${ACADOS_GENERATED_DIR}
  DEPENDS ${ACADOS_GENERATED_SRCS}
  COMMENT "Building acados solver shared library"
)

# Ensure that solver lib exists before building
add_custom_target(acados_solver_lib ALL DEPENDS ${ACADOS_GENERATED_DIR}/libacados_ocp_solver_mirte.so)

add_executable(tube_mpc_node 
    src/tube_nmpc.cpp 
    src/main.cpp
    src/acados_interface.cpp
)

add_dependencies(tube_mpc_node acados_solver_lib)

# Include paths
target_include_directories(tube_mpc_node PUBLIC
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${ACADOS_GENERATED_DIR} 
    ${ACADOS_CORE_DIR}/include
    ${ACADOS_CORE_DIR}/acados/include
    ${ACADOS_CORE_DIR}/external/blasfeo/include
    ${ACADOS_CORE_DIR}/external/hpipm/include
)

target_link_directories(tube_mpc_node PUBLIC
    ${ACADOS_CORE_DIR}/lib
    ${ACADOS_CORE_DIR}/external/blasfeo/lib
    ${ACADOS_CORE_DIR}/external/hpipm/lib
    ${ACADOS_GENERATED_DIR}
)

ament_target_dependencies(tube_mpc_node
  rclcpp
  std_msgs
  Eigen3
  nav_msgs
  mirte_msgs
  tf2
  tf2_geometry_msgs
  geometry_msgs
)

target_link_libraries(tube_mpc_node
    acados_solver_mirte
    acados
    blasfeo
    hpipm
    Eigen3::Eigen
    rclcpp::rclcpp
)

set_target_properties(tube_mpc_node PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "${ACADOS_GENERATED_DIR}"
)

install(TARGETS tube_mpc_node DESTINATION lib/${PROJECT_NAME})
install(FILES ${ACADOS_GENERATED_DIR}/libacados_ocp_solver_mirte.so DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()

