cmake_minimum_required(VERSION 2.8.3 FATAL_ERROR)

set(version_major 0)
set(version_minor 1)
set(version_patch 0)
set(version "${version_major}.${version_minor}.${version_patch}")
set(package-contact "esolomon1221@gmail.com")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/../cmake_hexagon")
set(BUILD_SHARED_LIBS ON)

include(linux_app)

include_directories("../common")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -O0 -std=gnu11 -fno-strict-aliasing -fdata-sections -fno-zero-initialized-in-bss -Wall -Wextra -Werror -Wno-unused-parameter -Wno-unused-function -Wno-unused-variable -Wno-cast-align -Wno-missing-braces -Wno-strict-aliasing") 

FASTRPC_STUB_GEN(../common/serial_interface.idl)

LINUX_LIB(LIB_NAME simple_autonomy_apps
	IDL_NAME serial_interface 
	SOURCES
		basic_uart.c
)

set(SIMPLE_AUTONOMY_OBJS_DIR
	${CMAKE_BINARY_DIR}/CMakeFiles/simple_autonomy_apps.dir
)

set(SIMPLE_AUTONOMY_SO_FLAGS
	-fPIC -O0 -fno-strict-aliasing -fdata-sections -fno-zero-initialized-in-bss)

add_custom_target(simple_autonomy_apps_shared
	COMMAND
${HEXAGON_SDK_ROOT}/gcc-linaro-4.9-2014.11-x86_64_arm-linux-gnueabihf_linux/bin/arm-linux-gnueabihf-gcc ${FASTRPC_ARM_LIBS} ${SIMPLE_AUTONOMY_SO_FLAGS} -fPIC -shared -o libsimple_autonomy_apps.so ${SIMPLE_AUTONOMY_OBJS_DIR}/basic_uart.c.obj ${SIMPLE_AUTONOMY_OBJS_DIR}/serial_interface_stub.c.obj ${SIMPLE_AUTONOMY_OBJS_DIR}/${HEXAGON_SDK_ROOT}/libs/common/rpcmem/src/rpcmem.c.obj
)
#FASTRPC_ARM_LINUX_INCLUDES
#FASTRPC_ARM_LIBS

add_dependencies(simple_autonomy_apps_shared
	simple_autonomy_apps
)

# vim: set noet fenc=utf-8 ff=unix ft=cmake :
