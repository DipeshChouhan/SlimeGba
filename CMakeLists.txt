cmake_minimum_required(VERSION 3.24)
project(slime)

set(SRC_DIR src)

list(APPEND COMPILE_FILES ${SRC_DIR}/slime.c 
    ${SRC_DIR}/arm/arm.c ${SRC_DIR}/bus/sys_bus.c)

add_executable(${PROJECT_NAME} ${COMPILE_FILES})


