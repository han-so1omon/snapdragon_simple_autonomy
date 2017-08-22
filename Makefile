CURR_DIR=$(shell pwd)
APPS_OBJS_DIR=${CURR_DIR}/build_apps/CMakeFiles/simple_autonomy_apps.dir
APPS_SO_FLAGS=-fPIC -O0 -fno-strict-aliasing -fdata-sections -fno-zero-initialized-in-bss
.PHONY all:
all: libsimple_autonomy_adsp libsimple_autonomy_apps

QC_SOC_TARGET?="APQ8074"

.PHONY libsimple_autonomy_adsp:
libsimple_autonomy_adsp:
	@mkdir -p build_adsp && cd build_adsp && cmake -Wno-dev ../adsp -DQC_SOC_TARGET=${QC_SOC_TARGET} -DCMAKE_TOOLCHAIN_FILE=../cmake_hexagon/toolchain/Toolchain-qurt.cmake
	@cd build_adsp && make

.PHONY libsimple_autonomy_adsp-debug:
libsimple_autonomy_adsp-debug:
	@mkdir -p build_adsp && cd build_adsp && cmake -Wno-dev ../adsp -DQC_SOC_TARGET=${QC_SOC_TARGET} -DCMAKE_TOOLCHAIN_FILE=../cmake_hexagon/toolchain/Toolchain-qurt.cmake
	@cd build_adsp && make VERBOSE=1

.PHONY libsimple_autonomy_apps:
libsimple_autonomy_apps:
	@mkdir -p build_apps && cd build_apps && cmake -Wno-dev ../apps -DQC_SOC_TARGET=${QC_SOC_TARGET} -DCMAKE_TOOLCHAIN_FILE=../cmake_hexagon/toolchain/Toolchain-arm-linux-gnueabihf.cmake
	@cd build_apps && make 
	@cd build_apps && make simple_autonomy_apps_shared
	@mkdir -p ros/snapdragon_simple_autonomy/include/simple_autonomy && cp apps/*.h ros/snapdragon_simple_autonomy/include/simple_autonomy/ && cp common/*.h ros/snapdragon_simple_autonomy/include/simple_autonomy/ && cp build_apps/*.h ros/snapdragon_simple_autonomy/include/simple_autonomy
	@mkdir -p ros/snapdragon_simple_autonomy/lib && cp build_apps/libsimple_autonomy_apps*.so ros/snapdragon_simple_autonomy/lib/

	#@cd build_apps && ${HEXAGON_SDK_ROOT}/gcc-linaro-4.9-2014.11-x86_64_arm-linux-gnueabihf_linux/bin/arm-linux-gnueabihf-gcc ${APPS_SO_FLAGS} -fPIC -shared -o libsimple_autonomy_apps.so ${APPS_OBJS_DIR}/basic_uart.c.obj ${APPS_OBJS_DIR}/serial_interface_stub.c.obj ${APPS_OBJS_DIR}/${HEXAGON_SDK_ROOT}/libs/common/rpcmem/src/rpcmem.c.obj

.PHONY libsimple_autonomy_apps-debug:
libsimple_autonomy_apps-debug:
	@mkdir -p build_apps && cd build_apps && cmake -Wno-dev ../apps -DQC_SOC_TARGET=${QC_SOC_TARGET} -DCMAKE_TOOLCHAIN_FILE=../cmake_hexagon/toolchain/Toolchain-arm-linux-gnueabihf.cmake
	@cd build_apps && make VERBOSE=1
	@cd build_apps && make simple_autonomy_apps_shared VERBOSE=1
	@mkdir -p ros/snapdragon_simple_autonomy/include/simple_autonomy && cp apps/*.h ros/snapdragon_simple_autonomy/include/simple_autonomy/ && cp common/*.h ros/snapdragon_simple_autonomy/include/simple_autonomy/ && cp build_apps/*.h ros/snapdragon_simple_autonomy/include/simple_autonomy
	@mkdir -p ros/snapdragon_simple_autonomy/lib && cp build_apps/libsimple_autonomy_apps*.so ros/snapdragon_simple_autonomy/lib/

.PHONY clean:
clean:
	@rm -rf build_adsp build_apps

.PHONY ros-simple-load:
ros-simple-load:
ifdef ADB_DEV
	@adb -s $(ADB_DEV) shell rm -rf /home/linaro/ros_ws/src/snapdragon_simple_autonomy/*
	@adb -s $(ADB_DEV) push ros/snapdragon_simple_autonomy/. /home/linaro/ros_ws/src/snapdragon_simple_autonomy
	@adb -s $(ADB_DEV) push ros/snapdragon_simple_autonomy/launch/. /home/linaro/ros_ws/src/snapdragon_simple_autonomy/launch/
	@adb -s $(ADB_DEV) push ros/snapdragon_simple_autonomy/lib/. /home/linaro/ros_ws/src/snapdragon_simple_autonomy/lib/
	@adb -s $(ADB_DEV) push ros/snapdragon_simple_autonomy/include/. /home/linaro/ros_ws/src/snapdragon_simple_autonomy/include/
else
	@adb shell rm -rf /home/linaro/ros_ws/src/snapdragon_simple_autonomy/*
	@adb push ros/snapdragon_simple_autonomy/. /home/linaro/ros_ws/src/snapdragon_simple_autonomy
	@adb push ros/snapdragon_simple_autonomy/launch/. /home/linaro/ros_ws/src/snapdragon_simple_autonomy/launch/
	@adb push ros/snapdragon_simple_autonomy/lib/. /home/linaro/ros_ws/src/snapdragon_simple_autonomy/lib/
	@adb push ros/snapdragon_simple_autonomy/include/. /home/linaro/ros_ws/src/snapdragon_simple_autonomy/include/
endif

.PHONY mavros-cp:
mavros-cp:
	@cp -r ros/snapdragon_mavros_autonomy/* ${CATKIN_HOME}/src/snapdragon_mavros_autonomy

.PHONY mavros-load:
mavros-load:
ifdef ADB_DEV
	@adb -s $(ADB_DEV) shell rm -rf /home/linaro/ros_ws/src/snapdragon_mavros_autonomy/*
	@adb -s $(ADB_DEV) push ros/snapdragon_mavros_autonomy/. /home/linaro/ros_ws/src/snapdragon_mavros_autonomy
	@adb -s $(ADB_DEV) push ros/snapdragon_mavros_autonomy/launch/. /home/linaro/ros_ws/src/snapdragon_mavros_autonomy/launch/
else
	@adb shell rm -rf /home/linaro/ros_ws/src/snapdragon_mavros_autonomy/*
	@adb push ros/snapdragon_mavros_autonomy/. /home/linaro/ros_ws/src/snapdragon_mavros_autonomy
	@adb push ros/snapdragon_mavros_autonomy/launch/. /home/linaro/ros_ws/src/snapdragon_mavros_autonomy/launch/
endif

.PHONY load:
load: libsimple_autonomy_adsp libsimple_autonomy_apps ros-simple-load mavros-load
ifdef ADB_DEV
	@adb -s $(ADB_DEV) shell rm -f /usr/share/data/adsp/libserial_interface_skel.so /usr/share/data/adsp/libsimple_autonomy_adsp.so 
	@cd build_adsp && adb -s $(ADB_DEV) push libserial_interface_skel.so /usr/share/data/adsp/ && adb -s $(ADB_DEV) push libsimple_autonomy_adsp.so /usr/share/data/adsp/
else
	@adb shell rm -f /usr/share/data/adsp/libserial_interface_skel.so /usr/share/data/adsp/libsimple_autonomy_adsp.so 
	@cd build_adsp && make libsimple_autonomy_adsp-load
endif
