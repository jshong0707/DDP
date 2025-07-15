# CMake 프로그램의 최소 버전
cmake_minimum_required(VERSION 3.11)

# 프로젝트 정보
project(
  JinSong
  VERSION 0.1
  DESCRIPTION "Quad simulation"
  LANGUAGES CXX)

  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
  add_compile_options(-std=gnu++17)
  
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(glfw3 REQUIRED)
find_package(crocoddyl REQUIRED)

find_library(MUJOCO_LIB
  NAMES mujoco
  PATHS /usr/lib
  NO_DEFAULT_PATH            # 디폴트 검색 경로(Conda 등)는 절대 보지 않음
)


file(GLOB_RECURSE SOURCES "simulate.cpp" "glfw_adapter.cpp" "glfw_dispatch.cpp" "lodepng.cpp" "platform_ui_adapter.cpp" 
"main.cc" "globals.cc" "filter.cc" "Controller.cc" "Integrate.cc" "Trajectory.cc" "BezierCurve.cc"
 "Actuator.cc" "F_Kinematics.cc" "B_Kinematics.cc" "Body.cc" "MPC.cc" "MPC_util.cc"
 "dataLogging.cc" "FSM.cc" "robot_parameter.cc")


add_executable (main ${SOURCES})
# target_compile_options(main PRIVATE -std=gnu++17 -pthread)

target_include_directories(main PUBLIC
  ${CMAKE_SOURCE_DIR}/robot
  ${CMAKE_SOURCE_DIR}/util
  ${CMAKE_SOURCE_DIR}/../../../include
  ${CMAKE_SOURCE_DIR}/GUI
  ${CMAKE_SOURCE_DIR}/Controller
  ${CMAKE_SOURCE_DIR}/Kinematics
  ${CMAKE_SOURCE_DIR}/LegTrajectory
  ${CMAKE_SOURCE_DIR}/State_Estimation
  # /home/scr
  # /usr/include/eigen-master/unsupported/Eigen
  )

target_link_directories(main PUBLIC           #Library directory
  ${CMAKE_SOURCE_DIR}/../../lib
  /usr/lib
)



target_link_libraries(main
    # lapack
    # blas
    # qpOASES
    # casadi::casadi
    Eigen3::Eigen
    pinocchio::pinocchio
    crocoddyl::crocoddyl
    ${MUJOCO_LIB}
    glfw
    osqp
    
)