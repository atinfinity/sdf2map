cmake_minimum_required(VERSION 3.16)
project(sdf2map)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(sdformat_vendor REQUIRED)
find_package(sdformat14 REQUIRED)
find_package(gz_math_vendor REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz_common_vendor REQUIRED)
find_package(gz-common5 REQUIRED COMPONENTS graphics geospatial)
find_package(PCL REQUIRED COMPONENTS common io filters registration)

add_library(sdf2map_core STATIC
  src/resource_resolver.cpp
  src/samplers.cpp
  src/world_sampler.cpp
  src/exporter.cpp
)
target_include_directories(sdf2map_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(sdf2map_core PUBLIC
  sdformat14::sdformat14
  gz-math7::gz-math7
  gz-common5::graphics
  gz-common5::geospatial
  ${PCL_LIBRARIES}
)
target_compile_definitions(sdf2map_core PUBLIC ${PCL_DEFINITIONS})

add_executable(sdf2map src/main.cpp)
target_link_libraries(sdf2map sdf2map_core)

install(TARGETS sdf2map DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_samplers test/test_samplers.cpp)
  target_link_libraries(test_samplers sdf2map_core)
  ament_add_gtest(test_world_sampler test/test_world_sampler.cpp)
  target_link_libraries(test_world_sampler sdf2map_core)
  target_compile_definitions(test_world_sampler PRIVATE
    TEST_WORLDS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/worlds")
  ament_add_gtest(test_exporter test/test_exporter.cpp)
  target_link_libraries(test_exporter sdf2map_core)
  ament_add_gtest(test_e2e_geometry test/test_e2e_geometry.cpp)
  target_link_libraries(test_e2e_geometry sdf2map_core)
  target_compile_definitions(test_e2e_geometry PRIVATE
    TEST_WORLDS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/worlds")
  ament_add_gtest(test_ndt_localization test/test_ndt_localization.cpp
    TIMEOUT 120)
  target_link_libraries(test_ndt_localization sdf2map_core)
  target_compile_definitions(test_ndt_localization PRIVATE
    TEST_WORLDS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/worlds")
endif()

ament_package()
