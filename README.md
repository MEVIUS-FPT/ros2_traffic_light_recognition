# ROS2 Package for Traffic Light Recognition
This is a ROS2 package that includes HLS-based traffic light recognition circuit. See [MEVIUS-FPT/hls_traffic_light_recognition](https://github.com/MEVIUS-FPT/hls_traffic_light_recognition) for details on the traffic light recognition circuit.

## Supported Environment
- [Kria KV260 Vision AI Starter Kit](https://www.xilinx.com/products/som/kria/kv260-vision-starter-kit.html)
  - [2022.1 Boot FW Update](https://xilinx-wiki.atlassian.net/wiki/spaces/A/pages/1641152513/Kria+K26+SOM#Boot-Firmware-Updates)
  - [Ubuntu 22.04](https://ubuntu.com/download/amd-xilinx)
  - [Kria-PYNQ v3.0](https://github.com/Xilinx/Kria-PYNQ/releases/tag/v3.0)  
  - [ROS2 Humble Hawksbill](https://docs.ros.org/en/humble/Releases/Release-Humble-Hawksbill.html)  
  
  The Bitstream included in this repository is a file generated for [K26](https://www.xilinx.com/products/som/kria/k26c-commercial.html). Please see [MEVIUS-FPT/hls_traffic_light_recognition](https://github.com/MEVIUS-FPT/hls_traffic_light_recognition) for more information. If you want to run another target as a ROS2 node, please regenerate the Bitstream and replace the `.bit` and `.hwh` files.
## Build Workspace
- Custom Message Package
  ```
  colcon build --packages-select traffic_light_recognition_interface
  ```
- ROS2 Node Package
  ```
  colcon build --packages-select traffic_light_recognition_fpga_node
  ```
## Run
### Setup
```
. install/setup.bash
```
### ROS2-FPGA Node (Processing Traffic Light Recognition)
```
ros2 run traffic_light_recognition_fpga_node fpga_node
```
### Talker (Publishing Camera Captured Image)
```
ros2 run traffic_light_recognition_fpga_node talker --ros-args --params-file traffic_light_recognition_param.yaml
```
### Listener (For Debugging)
```
ros2 run traffic_light_recognition_fpga_node draw_circle
```

