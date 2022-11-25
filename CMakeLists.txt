cmake_minimum_required(VERSION 3.24)
project(slime)

set(SRC_DIR src)
set(CJSON_DIR libs/cJSON)

add_library(cJSON SHARED ${CJSON_DIR}/cJSON.c)

list(APPEND COMPILE_FILES ${SRC_DIR}/slime.c 
  ${SRC_DIR}/arm/arm.c ${SRC_DIR}/arm/thumb.c ${SRC_DIR}/arm/disassembler.c ${SRC_DIR}/memory/memory.c ${SRC_DIR}/gba/gba.c ${SRC_DIR}/utils/file_open.c)

add_executable(${PROJECT_NAME} ${COMPILE_FILES})
target_link_libraries(${PROJECT_NAME} cJSON)

