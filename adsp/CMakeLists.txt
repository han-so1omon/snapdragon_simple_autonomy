cmake_minimum_required(VERSION 2.8.3 FATAL_ERROR)

set(version_major 0)
set(version_minor 1)
set(version_patch 0)
set(version "${version_major}.${version_minor}.${version_patch}")
set(package-contact "esolomon1221@gmail.com")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/../cmake_hexagon")
include(qurt_flags)
include(qurt_lib)

include_directories("$ENV{DSPAL_PATH}/include")
include_directories("../common")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0 -g -std=gnu11 -fno-strict-aliasing -fdata-sections -fno-zero-initialized-in-bss -Wall -Wextra -Werror -Wno-unused-parameter -Wno-unused-function -Wno-unused-variable -Wno-gnu-array-member-paren-init -Wno-cast-align -Wno-missing-braces -Wno-strict-aliasing") 

FASTRPC_STUB_GEN(../common/serial_interface.idl)

QURT_LIB(LIB_NAME simple_autonomy_adsp
	IDL_NAME serial_interface
	SOURCES
		basic_uart.c
)

# vim: set noet fenc=utf-8 ff=unix ft=cmake :
