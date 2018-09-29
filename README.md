# Building the code
===================
Run the make command in the code directory and specify the path to ESP open rtos root
directory.

```
$ make -j4 RTOS_ROOTDIR=<path-to-esp-open-rtos-dir>
```

# Flashing the code
===================
To flash the code to esp8266 microcontroller, first hold the flash button and then press
the reset button. Once the esp8266 has booted, release the reset button. After that,
run the below command.

```
$ make flash ESPPORT=<serial-port-for-the-esp8266>

# For quick operation, both build and flash can be combined into a single command
$ make flash -j4 ESPPORT=<serial-port-for-the-esp8266> RTOS_ROOTDIR=<path-to-esp-open-rtos-dir>

```
