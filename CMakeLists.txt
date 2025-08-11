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
find_package(OpenGL REQUIRED)   # Linux면 필요

# MuJoCo 라이브러리/헤더 (경로 환경에 맞게)
find_library(MUJOCO_LIB mujoco REQUIRED)
find_path(MUJOCO_INCLUDE_DIR /../include/mujoco/mujoco.h REQUIRED)
# find_library(MUJOCO_LIB
#   NAMES mujoco
#   PATHS /usr/lib
#   NO_DEFAULT_PATH            # 디폴트 검색 경로(Conda 등)는 절대 보지 않음
# )


# 샘플 유틸 구현파일(정확히 필요 것만!)
set(SIM_SOURCES
  ${CMAKE_SOURCE_DIR}/../simulate/simulate.cc
  ${CMAKE_SOURCE_DIR}/../simulate/glfw_adapter.cc
  ${CMAKE_SOURCE_DIR}/../simulate/glfw_dispatch.cc
  ${CMAKE_SOURCE_DIR}/../simulate/platform_ui_adapter.cc
  # ${CMAKE_SOURCE_DIR}/simulate/array_safety.cc
)

# lodepng (지금 simulate 폴더에 넣었다면 include 경로만 추가)
set(LODEPNG_SOURCES ${CMAKE_SOURCE_DIR}/../simulate/lodepng.cpp)


file(GLOB_RECURSE SOURCES  
"main.cc" 
"globals.cc" 
"filter.cc" 
"Controller.cc" 
"Integrate.cc" 
"Trajectory.cc" 
"BezierCurve.cc"
"Actuator.cc"
"F_Kinematics.cc"
"B_Kinematics.cc"
"Body.cc" 
"MPC.cc" 
"MPC_util.cc"
"dataLogging.cc" 
"FSM.cc" 
"robot_parameter.cc" 
"ModelBuilder.cc" 
"CostBuilder.cc"
"ContactBuilder.cc"
"DDPSolver.cc")


add_executable (main 
${SOURCES}
${SIM_SOURCES}
${LODEPNG_SOURCES}
)
# target_compile_options(main PRIVATE -std=gnu++17 -pthread)

target_include_directories(main PUBLIC
  ${CMAKE_SOURCE_DIR}/../simulate         # simulate/*.h, lodepng.h
  ${MUJOCO_INCLUDE_DIR}                # contains mujoco/mujoco.h
  ${CMAKE_SOURCE_DIR}/robot
  ${CMAKE_SOURCE_DIR}/util
  ${CMAKE_SOURCE_DIR}/../../../include
  ${CMAKE_SOURCE_DIR}/../simulate
  ${CMAKE_SOURCE_DIR}/Controller
  ${CMAKE_SOURCE_DIR}/Controller/DDP
  ${CMAKE_SOURCE_DIR}/Kinematics
  ${CMAKE_SOURCE_DIR}/LegTrajectory
  ${CMAKE_SOURCE_DIR}/State_Estimation
  # /home/scr
  # /usr/include/eigen-master/unsupported/Eigen
  )

# target_link_directories(main PUBLIC           #Library directory
#   ${CMAKE_SOURCE_DIR}/../../lib
#   /usr/lib
# )



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
    # osqp
    
)