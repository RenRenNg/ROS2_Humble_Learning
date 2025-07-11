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

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CountUntil.action"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
```

#### **Building**  
cd ~/ros2_ws
colcon build --packages-select my_robot_interfaces  
source ~/.bashrc  
ros2 interface show my_robot_interfaces/action/CountUntil  
Check you can see the goal, result and feedback you created  
If you dont repeat and check where it go wrong  

### **Write a Python Server**    
#### **Create Package**  
ros2 pkg create actions_py --build-type ament_python  --dependencies rclpy my_robot_interfaces  
cd ~/ros2_ws/src/actions_py/actions_py  
touch count_until_server.py  
chmod +x count_until_server.py  
#### **Inside count_until_server**  
```python
#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from my_robot_interfaces.action import CountUntil

class CountUntilServerNode(Node):
    def __init__(self):
        super().__init__("count_until_server")
        self.count_until_server_ = ActionServer(self, CountUntil, "count_until", execute_callback=self.execute_callback)
        self.get_logger().info("Action server has been started")

    def execute_callback(self, goal_handle: ServerGoalHandle):
        #Get request from code
        target_number = goal_handle.request.target_number  
        period = goal_handle.request.period

        #Execute the action
        self.get_logger().info("Executing the goal")
        counter = 0
        for i in range(target_number):
            counter += 1
            time.sleep(period)

        # Once done, set goal final state
        goal_handle.succeed()

        # and send the result
        result = CountUntil.Result()
        result.reached_number = counter
        return result

def main(args=None):
    rclpy.init(args=args)
    node = CountUntilServerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

#### **Inside setup.py**  
Under console_scriptes  
Add "count_until_server = actions_py.count_until_server:main"  
Save  

#### **building**
cd ~/ros2_ws  
colcon build --packages-select actions_py  
source install/setup.bash  
ros2 run actions_py count_until_server  

### **Write a Python Action Cilent**  
#### **Create file**  
cd ~/ros2_ws/src/actions_py/actions_py  
touch count_until_client.py
chmod +x count_until_client.py


