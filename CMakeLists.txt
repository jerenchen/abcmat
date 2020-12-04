cmake_minimum_required(VERSION 3.1)

project(abcmat)

set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Alembic REQUIRED)

include_directories(abcmat)

add_subdirectory(examples)

