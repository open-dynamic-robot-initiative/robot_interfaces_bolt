cmake_minimum_required(VERSION 3.8)
project(robot_interfaces_bolt)

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

find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)
find_package(pybind11 REQUIRED)

ament_export_dependencies(cli_utils real_time_tools robot_interfaces solo)



add_library(solo12_config
    src/solo12_config.cpp
)
target_include_directories(solo12_config PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(solo12_config
    Eigen3::Eigen
    yaml_utils::yaml_utils
    fmt::fmt
)

add_library(solo12_driver
    src/solo12_driver.cpp
    src/solo12_utils.cpp
)
target_include_directories(solo12_driver PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(solo12_driver
    Eigen3::Eigen
    real_time_tools::real_time_tools
    robot_interfaces::robot_interfaces
    solo::solo12
    spdlog::spdlog

    solo12_config
)

add_library(solo12_pybullet_driver
    src/solo12_pybullet_driver.cpp
    src/solo12_utils.cpp
)
target_include_directories(solo12_pybullet_driver PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(solo12_pybullet_driver
    Eigen3::Eigen
    real_time_tools::real_time_tools
    robot_interfaces::robot_interfaces
    spdlog::spdlog
    pybind11::pybind11

    solo12_config
)
# using pybind11 types, therefore visibility needs to be hidden
# https://pybind11.readthedocs.io/en/stable/faq.html#someclass-declared-with-greater-visibility-than-the-type-of-its-field-someclass-member-wattributes
set_target_properties(solo12_pybullet_driver
    PROPERTIES CXX_VISIBILITY_PRESET hidden)



add_executable(demo_solo12_hold demos/demo_solo12_hold.cpp)
target_link_libraries(demo_solo12_hold
    cli_utils::program_options
    solo12_driver
)


## Python Bindings

# TODO: NO_EXTRAS is needed as workaround for an lto-related internal compiler
# error of gcc.  This disables some optimisations and is thus not a good
# permanent solution!  Some proper fix for the gcc issue should be found.
if(CMAKE_COMPILER_IS_GNUCXX)
    message(WARNING
        "Set NO_EXTRAS for Python modules to mitigate an internal error of GCC")
    set(NO_EXTRAS "NO_EXTRAS")
endif()

add_pybind11_module(solo12 srcpy/solo12.cpp
    ${NO_EXTRAS}
    LINK_LIBRARIES 
        solo12_driver
        solo12_pybullet_driver
)


install(DIRECTORY include/ DESTINATION include)

install(
    TARGETS
        solo12_config
        solo12_driver
        solo12_pybullet_driver
        demo_solo12_hold
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    INCLUDES DESTINATION include
)

install_scripts(
    demos/demo_solo12_sine.py
    demos/demo_solo12_simulation.py
    scripts/solo12_show_data.py

    DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)

    # Python tests
    ament_add_pytest_test(test_solo12_config tests/test_solo12_config.py)
endif()


ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
