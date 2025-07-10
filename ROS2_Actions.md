### **Why ROS2 Actions**  
ROS Communication tools
- Topics: Sending data streams
- Services: Client/Server communications
- Actions: Client/Server for longer actions, with cancel, feedback, etc  

### **How to do Actions work**  
![image](https://github.com/user-attachments/assets/4729b1be-6e01-49f5-9bf1-e4c0c6dc43c0)  
Already implmented in action cilent and server classes  
What we going to implement  
![image](https://github.com/user-attachments/assets/1c267627-192b-4b69-b258-2c790d8b7310)  
Cilent can be multiple but only one action server  
![image](https://github.com/user-attachments/assets/b25f736d-5fd7-4e23-a27b-a8d69549f40f)


### **Create an Action definition**  
#### **Create Workspace (ros2_ws)**  
Follow instructions from ROS2 cheatsheet file.  
cd ~/ros2_ws/src  
rm -r include/ src/  
mkdir msg srv action  
cd ..  
code .  

#### **Inside action file**  
Create new file > CountUntil.action

#### **Inside CountUntil.action**  
```yaml
#Goal
int64 target_number
float64 period
---
#Result
int64 reached_number
---
#Feedback
int64 current_number
```

#### **Adding dependencies**  
#### **Inside package.xml**  
Between <buildtool_depend>ament_cmake</buildtool_depend> & <test_depend>ament_lint_auto</test_depend>   
Add the following:  
```xml
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
```

#### **Inside CMakeLists.txt**  
```python
cmake_minimum_required(VERSION 3.8)
project(my_robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_interface_packages(${PROJECT_NAME}
  "action/CountUntil.action"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
```  


#### **Axes**  

