project(octread)

add_executable(octread main.cpp)
target_link_libraries(octread Alembic::Alembic Eigen3::Eigen)
install(TARGETS octread DESTINATION examples)
