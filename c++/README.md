For cross compiling, add the following flag:
cmake -DCMAKE_TOOLCHAIN_FILE=../../aarch64-linux-gnu.cmake ../

Make sure you follow these steps for the jetson nano:
uname -r: 4.9.337-tegra
lsb_release -a: 
No LSB modules are available.
Distributor ID: Ubuntu
Description:    Ubuntu 20.04.6 LTS
Release:        20.04
Codename:       focal

https://dev.intelrealsense.com/docs/nvidia-jetson-tx2-installation section Building from Source using Native Backend

Note: don't need kernel patches


And use these flags:

cmake .. -DBUILD_EXAMPLES=true -DCMAKE_BUILD_TYPE=release -DFORCE_RSUSB_BACKEND=false