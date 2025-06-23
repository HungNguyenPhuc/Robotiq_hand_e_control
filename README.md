*** HƯỚNG DẪN SỬ DỤNG ***

1. Các gói phụ thuộc
   
    - Python3
    - pymodbus >=2.5.3
    - ROS Noetic
    - Ubuntu 20.04
      
3. Cài đặt và xây dựng gói
- Tải về:
  
    ```bash
    cd ~/your_ros_ws/src
    git clone https://github.com/HungNguyenPhuc/Robotiq_hand_e_control.git  
- Xây dựng gói
  
    ```bash
    cd ~/your_ros_ws
    catkin_make 
    source devel/setup.bash
    ```
4. Điều khiển qua ROS service node

- Chạy ROS service node điều khiển gripper:
  
  ```bash
  roslaunch robotiq_hand_e_control gripper_service_node.launch
  ```
- Gửi lệnh qua ROS service:
  
  ```bash
  rosservice call /gripper_control "command: 'activate'"  # dùng để active tool
  rosservice call /gripper_control "command: 'open'"      # dùng để mở hết mức tool
  rosservice call /gripper_control "command: 'close'"     # dùng để đóng hết mức tool
  rosservice call /gripper_control "command: 'reset'"     # dùng để reset lỗi
  rosservice call /gripper_control "command: 'stop'"      # dùng để dừ tool
  
  # Lệnh custom: vị trí, tốc độ, lực
  rosservice call /gripper_control "command: 'custom:128,150,100'"
  ```
4. Điều khiển qua ROS driver node
   
- Chạy ROS driver node điều khiển gripper:
  
  ```bash
  roslaunch robotiq_hand_e_control gripper_driver.launch
  ```
- Gửi lệnh qua ROS topic:
  
  ```bash
  rostopic pub /gripper_command_string std_msgs/String "data: 'open'"
  rostopic pub /gripper_command_string std_msgs/String "data: 'close'"
  rostopic pub /gripper_command_string std_msgs/String "data: 'activate'"
  
  # Lệnh custom: vị trí, tốc độ, lực
  rostopic pub /gripper_command_string std_msgs/String "data: 'custom:120,150,100'"
  ```
- Xem trạng thái gripper:
  
  ```bash
  rostopic echo /gripper_status
  ```
  Kết quả
  ```bash
  gSTA: 3
  gOBJ: 1
  gFLT: 0
  gPOS: 250
  gCUR: 12
  sta_text: "Activated"
  obj_text: "Stopped(outer)"
  flt_text: "No Fault"
  ```
