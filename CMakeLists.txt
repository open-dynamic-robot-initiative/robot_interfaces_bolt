cmake_minimum_required(VERSION 3.8)
project(robot_interfaces_solo)

# Specify C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED on)

# libraries need to be position independent for building Python modules
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cli_utils REQUIRED)
find_package(yaml_utils REQUIRED)
find_package(mpi_cmake_modules REQUIRED)
find_package(real_time_tools REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(solo REQUIRED)

find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)

ament_export_dependencies(cli_utils real_time_tools robot_interfaces solo)


add_library(solo12_driver src/solo12_driver.cpp)
target_include_directories(solo12_driver PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(solo12_driver
    real_time_tools::real_time_tools
    robot_interfaces::robot_interfaces
    solo::solo12
    yaml_utils::yaml_utils
    fmt::fmt
    spdlog::spdlog
)


add_executable(demo_solo12_hold demos/demo_solo12_hold.cpp)
target_link_libraries(demo_solo12_hold
    cli_utils::program_options
    solo12_driver
)


# Python Bindings
add_pybind11_module(solo12 srcpy/solo12.cpp
    LINK_LIBRARIES solo12_driver
)


install(DIRECTORY include/ DESTINATION include)

install(
    TARGETS
        solo12_driver
        demo_solo12_hold
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    INCLUDES DESTINATION include
)

install_scripts(
    demos/demo_solo12_sine.py
    scripts/position_ctrl.py

    DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()

    # Python tests
    ament_add_pytest_test(test_solo12_config tests/test_solo12_config.py)
endif()


ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
