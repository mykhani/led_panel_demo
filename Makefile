PROGRAM=light_camera_controller
EXTRA_COMPONENTS = extras/paho_mqtt_c extras/i2c extras/pca9685
CPPFLAGS += -DCONSOLE_UART=0
include ${RTOS_ROOTDIR}/common.mk
