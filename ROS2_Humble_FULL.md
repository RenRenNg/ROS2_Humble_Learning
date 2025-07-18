# **ROS2 Humble Learning**
## **Useful Tools**  
cd #moving directories  
mkdir <folder_name> #Make folder  
touch <file_name> #create a file with format (.py, .yaml)  
chmod +x <file_name> #change file permissions  
ls #Show list of files in current directory  
ls -la #Show hidden files  
gedit #edit certain file code  
code . #launch vs code from current directory of the terminal  
## **ROS2 Tools**  
ros2 <command_line> -h # show arguments/commands of the command  
colcon build # Must be inside the workspace first level (~/ros2_ws/)   
colcon build --packages-select <package_name_1> <package_name_2> ...  #Build selected packages  
colcon build --packages-select <package_name_1> <package_name_2> -symlink-install # constant  building python packages only (less stable & buggy)   
rm -r build/ install/ log/ #if build in the wrong cd instead of ~/ros2_ws  
ros2 pkg create <pkg_name> --build-type ament_python --dependencies rclpy #Create a pkg with py version and dependencies  
ros2 pkg create <pkg_name> --build-type ament_cmake --dependencies rclcpp #Create a pkg with cpp version and dependencies  
source ~/.bashrc  
source install/setup.bash  
ros2 run <package_name> <executable_node_name>  
ros2 node list # show list of nodes running
ros2 node info <node_name> # Show info of the node
## **OOP Template for Your Nodes**  
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyCustomNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("node_name") # MODIFY NAME

def main(args=None):
    rclpy.init(args=args)
    node = MyCustomNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()
     
     
if __name__ == "__main__":
    main()
```

## **Source**  
https://www.udemy.com/course/ros2-for-beginners/  

## **Install ROS2 and Setup your enivronment**
In this tutorial, I will be using Humble instead of Jazzy as I am using Ubuntu 22.04 Native   
Follow the instructions of installation in https://docs.ros.org/en/humble/Installation.html  
### **Launch ROS2 Program**
ros2 run demo_nodes_cpp talker  
ros2 run demo_nodes_cpp listener  

## **Write your first ROS 2 Program**  
### **Create a ROS2 Workspace**
cd ~  
mkdir ros2_ws  
cd ros2_ws/  
mkdir src  
#### **Building**  
cd ~/ros2_ws  
colcon build  
ls > should see build install and log folders  
#### **Sourcing WorkSpace**
cd ~/ros2_ws/install  
ls > there is a setup.bash file that need sourcing  
cd ~  
gedit .bashrc  
Add > source cd ~/ros2_ws/install/setup.bash at the last line > SAVE   

### **Create a Python Package**  
<img width="714" height="386" alt="image" src="https://github.com/user-attachments/assets/2563831f-8a31-4ef6-8a4d-df69dcf46eaf" />   

Packages will allow you to separate your code into reusable blocks.  
Each package is an independent unit.  
For example, in one application you could have one package that will handle a camera, another package that will run the wheels of your robot, and yet another one that will handle motion planning for the robot in the environment.  
cd ~/ros2_ws/src/  
ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy  #py verison
#### **Inside my_py_pkg**  
Inside my_py_pkg > my_py_pkg: This is where you write the py codes.  
Inside package.xml: FIll up license if you publishing the code, dependencies, build type  
Inside setup.py: Use when installing node under console scripts  

### **What is a ROS2 Node**  
<img width="665" height="391" alt="image" src="https://github.com/user-attachments/assets/8b5a15f2-b632-47cf-930a-8bd638a811f0" />  

- Subprograms in your application, responsible for only one thing
- Combined into a graph
- Communincate with each other through topics, service and parameters
Benfits:
- Reduce code complexity
- Fault tolerance
- Can be written in Py, C++, ...

### **Write a Python Node** - with OOP
cd ~/ros2_ws/src/my_py_pkg/my_py_pkg  
touch my_first_node.py  
#### **Inside my_first_node.py**  
```python
#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__("py_test")
        self.counter_ = 0
        self.get_logger().info("Hello world")
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("Hello " + str(self.counter_))
        self.counter_ += 1

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node) # run forever until ctrl c is used
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```  
chmod +x my_first_node.py  
./my_first_node.py  #run file or the python3 way  
#### **Install node**
Inside setup.py file  
Under console scripts  
"py_node = my_py_pkg.my_first_node:main" #executable node name  
#### **Building**  
cd ~/ros2_ws  
colcon build --packages-select my_py_pkg  
source ~/.bashrc  
ros2 run my_py_pkg py_node   
### **ROS2 Client Libraries (RCL)**  

<img width="804" height="340" alt="image" src="https://github.com/user-attachments/assets/f6a97755-05fd-4540-b7e1-d433fdd60110" />  

## **Intro to ROS2 Tools**  
### **Rename a Node at Runtime**
When a node needs to be run multiple times.  
ros2 run my_py_pkg py_node --ros-args -r __node:=abc  #rename node when running
### **Colcon**  
