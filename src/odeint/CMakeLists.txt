cmake_minimum_required(VERSION 3.10)
project(odeint)

set(CMAKE_CXX_STANDARD 20)

add_executable(simulator main.cpp simulator_odefun.cpp)
