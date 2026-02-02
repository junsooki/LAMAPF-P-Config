cmake_minimum_required(VERSION 3.14)
project(networkflow_mapf LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(pybind11 CONFIG REQUIRED)
find_package(Python3 COMPONENTS Interpreter REQUIRED)

pybind11_add_module(flow_planner_cpp
    src/cpp/bindings.cpp
    src/cpp/flow_planner.cpp
    src/cpp/dinic.cpp
    src/cpp/grid_graph.cpp
)

target_include_directories(flow_planner_cpp PRIVATE src/cpp)

add_custom_command(TARGET flow_planner_cpp POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=$<TARGET_FILE_DIR:flow_planner_cpp> ${Python3_EXECUTABLE} -m pytest -q
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Running pytest after build"
)
