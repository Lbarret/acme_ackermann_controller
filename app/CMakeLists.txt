add_executable(shell-app main.cpp Robot.cpp PID.cpp AckermannController.cpp supportLib.cpp pbPlots.cpp)
include_directories(
    ${CMAKE_SOURCE_DIR}/include
)
