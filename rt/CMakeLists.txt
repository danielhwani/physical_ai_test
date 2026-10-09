# Go2 실시간 코어. MuJoCo는 mj_ros 환경의 pip 패키지에 든 libmujoco와 헤더를 쓴다 (Python과 같은 라이브러리 = 같은 물리).
#   cmake -S rt -B build/rt -DMUJOCO_DIR=$(python -c "import mujoco, os; print(os.path.dirname(mujoco.__file__))")
#   cmake --build build/rt
# 또는 python -m sim.rt_link --build
cmake_minimum_required(VERSION 3.16)
project(go2_rt CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(NOT MUJOCO_DIR)
  message(FATAL_ERROR "MUJOCO_DIR (mujoco pip 패키지 폴더)를 지정할 것")
endif()
file(GLOB MUJOCO_LIB "${MUJOCO_DIR}/libmujoco.so.*")
add_executable(rt_core rt_core.cpp)
target_include_directories(rt_core PRIVATE ${MUJOCO_DIR}/include)
target_link_libraries(rt_core PRIVATE ${MUJOCO_LIB} rt pthread)
set_target_properties(rt_core PROPERTIES BUILD_RPATH "${MUJOCO_DIR}")
# Python(numpy)과 같은 부동소수점 연산 순서: FMA 축약 금지
target_compile_options(rt_core PRIVATE -O2 -ffp-contract=off -Wall)
