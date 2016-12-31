
# Select 'mega' for the 1280 APM1, 'mega2560' otherwise
BOARD = px4fmu-v1_APM

# HAL_BOARD determines default HAL target.
HAL_BOARD ?= HAL_BOARD_APM2

# The communication port used to communicate with the APM.
PORT = /dev/tty.usbmodem1

# uncomment and fill in the path to Arduino if installed in an exotic location
# ARDUINO = /path/to/Arduino

# PX4Firmware tree: fill in the path to PX4Firmware repository from github.com/diydrones:
PX4_ROOT= ~/src/px4/PX4Firmware

# PX4NuttX tree: fill in the path to PX4NuttX repository from github.com/diydrones:
NUTTX_SRC= ~/src/px4/PX4NuttX/nuttx/
# VRBRAIN Firmware tree:
VRBRAIN_ROOT=../VRNuttX

# VRBRAIN NuttX tree:
VRBRAIN_NUTTX_SRC=../VRNuttX/NuttX/nuttx

