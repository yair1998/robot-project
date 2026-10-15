cmake_minimum_required(VERSION 3.16)

project(robot_agent)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Protobuf REQUIRED)
find_package(gRPC CONFIG REQUIRED)
find_package(Threads REQUIRED)

set(PROTO_GENERATED_SOURCES
    generated/robot_channel.pb.cc
    generated/robot_channel.grpc.pb.cc
)

# Local perception, navigation and safety core. No gRPC or protobuf in here, so it is unit-testable on its own and
# ThreadSanitizer results on it are meaningful.
set(ROBOT_CORE_SOURCES
    src/core/command_mailbox.cpp
    src/core/control_core.cpp
    src/core/local_navigation.cpp
    src/core/local_perception.cpp
    src/core/local_safety.cpp
    src/core/motion_backend.cpp
    src/core/python_perception_adapter.cpp
    src/core/simulated_sensor_provider.cpp
)

add_library(robot_core STATIC ${ROBOT_CORE_SOURCES})
target_include_directories(robot_core PUBLIC src)
target_compile_options(robot_core PRIVATE -Wall -Wextra)
target_link_libraries(robot_core PUBLIC Threads::Threads)

add_executable(robot-agent
    src/main.cpp
    ${PROTO_GENERATED_SOURCES}
)

target_include_directories(robot-agent PRIVATE
    generated
)

target_link_libraries(robot-agent PRIVATE
    robot_core
    gRPC::grpc++
    protobuf::libprotobuf
)

add_executable(plumbing-tests
    tests/plumbing_test.cpp
    ${PROTO_GENERATED_SOURCES}
)

target_include_directories(plumbing-tests PRIVATE
    generated
)

target_link_libraries(plumbing-tests PRIVATE
    gRPC::grpc++
    protobuf::libprotobuf
)

# Deterministic tests of the core: no network, no sleeps, a ManualClock. Non-zero exit code on any failure.
add_executable(local-nav-tests tests/local_nav_test.cpp)
target_compile_options(local-nav-tests PRIVATE -Wall -Wextra)
target_link_libraries(local-nav-tests PRIVATE robot_core)

enable_testing()
add_test(NAME local_nav_tests COMMAND local-nav-tests)

# The same tests under ThreadSanitizer, with the core compiled in instrumented:  cmake -DROBOT_TSAN=ON ...
option(ROBOT_TSAN "Also build local-nav-tests-tsan (ThreadSanitizer)" OFF)
if(ROBOT_TSAN)
    add_executable(local-nav-tests-tsan tests/local_nav_test.cpp ${ROBOT_CORE_SOURCES})
    target_include_directories(local-nav-tests-tsan PRIVATE src)
    target_compile_options(local-nav-tests-tsan PRIVATE -fsanitize=thread -g -O1)
    target_link_options(local-nav-tests-tsan PRIVATE -fsanitize=thread)
    target_link_libraries(local-nav-tests-tsan PRIVATE Threads::Threads)
    add_test(NAME local_nav_tests_tsan COMMAND local-nav-tests-tsan)
endif()
