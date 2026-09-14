cmake_minimum_required(VERSION 3.16)

project(robot_agent)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Protobuf REQUIRED)
find_package(gRPC CONFIG REQUIRED)

set(PROTO_GENERATED_SOURCES
    generated/robot_channel.pb.cc
    generated/robot_channel.grpc.pb.cc
)

add_executable(robot-agent
    src/main.cpp
    ${PROTO_GENERATED_SOURCES}
)

target_include_directories(robot-agent PRIVATE
    generated
)

target_link_libraries(robot-agent PRIVATE
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