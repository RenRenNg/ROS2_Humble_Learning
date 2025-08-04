# **ROS2 Humble Learning**
## **Useful Tools**  
cd #Moving directories  
mkdir <folder_name> #Make folder  
touch <file_name> #Create a file with format (.py, .yaml)  
chmod +x <file_name> #Change file permissions  
ls #Show list of files in current directory  
ls -la #Show hidden files  
gedit #Edit certain file code  
code . #Launch vs code from current directory of the terminal  
## **ROS2 Tools**   
#### **Building & sourcing**  
ros2 <command_line> -h #Show arguments/commands of the command  
colcon build #Must be inside the workspace first level (~/ros2_ws/)   
colcon build --packages-select <package_name_1> <package_name_2> ...  #Build selected packages  
colcon build --packages-select <package_name_1> <package_name_2> -symlink-install #Constant building python packages only (less stable & buggy)   
rm -r build/ install/ log/ #If build in the wrong cd instead of ~/ros2_ws  
ros2 pkg create <pkg_name> --build-type ament_python --dependencies rclpy #Create a pkg with py version and dependencies  
ros2 pkg create <pkg_name> --build-type ament_cmake --dependencies rclcpp #Create a pkg with cpp version and dependencies  
source ~/.bashrc  
source install/setup.bash  
#### **Running**  
ros2 run <package_name> <executable_node_name>  
#### **Introspecting & Troublshooting **  
ros2 node list #Show list of nodes running  
ros2 node info <node_name> #Show info of the node    
ros2 topic list #Show list of topics running     
ros2 topic echo <topic_name> #Show output of topic    
ros2 topic info <topic_name> #Show msg type, publisher and subscription count    
ros2 interface show <msg_type, srv_type, action_type> #Show what is inside the msg      
ros2 topic hz <topic_name> #Show rate of msg publish    
ros2 topic bw <topic_name> #Show how much data is sent    
ros2 interface list # List all of the interfaces  
ros2 interface package <package_name> #Show all interface of that package   
rqt_graph # Graphical representation of the nodes, topics and networks    
#### **Recording & Playing**   
ros2 bag record <topic_name> #Record data for that topic until ctrl + c  
ros2 bag record <topic_name1> <topic_name2> #Record multiple data for that topic until ctrl + c  
ros2 bag record -a #Record all data for that topic until ctrl + c  
ros2 bag record -o <file_name> <topic_name1> #Record data and output file will be named according to user  
ros2 bag record -o <file_name> <topic_name1> <topic_name2> #Record multiple data and output file will be named according to user  
ros2 bag info <file_name> #Info of the bag file  
ros2 bag play <file_name> #Play recorded file to see data topic needs to be subscribed  
#### **Service**
ros2 service list # Show list of services   
ros2 service type <service_name> # Show interface  
ros2 interface show <interface_name> #Show request and response   
ros2 service call <service_name> <interface_name> <request> # Provide request for a service ONLY WORK FOR SIMPLE REQUEST  
rqt > plugins > service caller to send simple request   
#### **Parameters**  
ros2 param list #Show list of parameters  
ros2 <package_name> <file_name> --ros-args -p <parameter_name1>:=<value_> -p <parameter_name2>:=<value_> #Set new parameter value at run time   
ros2 param get /<node_name> <parameter_name>   
ros2 run my_py_pkg <file_name> --ros-args -r __node:=<node_name> --params-file ~/yaml_params/number_params.yaml #Make the node name use the params of the params file    

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
ros2 run <pkg_name> <executable_node_name> --ros-args -r __node:=<new_executeable_node_name>  # Rename node when running  
ros2 run my_py_pkg py_node --ros-args -r __node:=abc  # Rename node when running  
### **TurtleSim**   
ros2 run turtlesim turtlesim_node   
ros2 run turtlesim turtle_teleop_key # Keyboard control   

## **ROS2 Topics**  
### **What is ROS2 Topic**  
A topic is a named bus over which nodes exchange messages
- Unidirectional data stream (Publisher/Subscriber)
- Anonymous
- A topic has a message type
- Can be written in Python, C++, ... directly inside ROS nodes
- A node can have many publishers/subscribers for any different topics  
### **Write a Python Publisher**
cd ~/ros2_ws/src/my_py_pkg/my_py_pkg  
touch robot_news_station.py  
chmod +x robot_news_station.py   
#### **Inside robot_news_station**  
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class RobotNewsStationNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("robot_news_station") # MODIFY NAME
        self.publisher_ = self.create_publisher(String, "robot_news", 10) #msg type, topic/node name, queue size
        self.timer_ = self.create_timer(0.5,self.pusblish_news)
    
    def pusblish_news(self):
        msg = String()
        msg.data = "Hello"
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotNewsStationNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()
     
     
if __name__ == "__main__":
    main()
```
#### **Getting the msg to publish**
ros2 topic list > look for /turtle1\_cmd_vel  
ros2 topic info /turtle1/cmd_vel > look at Type section > geometry_msgs/msg/Twist  
Import the msg to draw\_circle file  

#### **Adding dependencies**
In package.xml file > below `<depend>rclpy<depend>` > add `<depend>example_interfaces</depend>`  

#### **What input do i need**
ros2 interface show example_interface/msg/String > see the inputs of the msg > string data

#### **Adding console script**
In setup.py file > under entry_points > add "," behind an exising node if there is > new line > add "robot_news_station = my_py_pkg.robot_news_station:main"

### **Write a Python Subscriber**
#### **Inside smartphone**  
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class SmartPhoneNode(Node):
    def __init__(self):
        super().__init__("smartphone")
        self.subcriber_ = self.create_subscription(String, "robot_news", self.callback_robot_news, 10) #Same name the publisher topic name
        self.get_logger().info("Smartphone has been started")

    def callback_robot_news(self, msg: String):
        self.get_logger().info(msg.data) # from the publisher msg type



def main(args=None):
    rclpy.init(args=args)
    node = SmartPhoneNode()
    rclpy.spin(node)
    rclpy.shutdown()
     
     
if __name__ == "__main__":
    main()
```  
#### **Adding console script**
In setup.py file > under entry_points > add "," behind an exising node if there is > new line > add "smartphone = my_py_pkg.smartphone:main"  

### **Remap a topic at Runtime**  
ros2 run my_py_pkg robot_news_station --ros-args -r __node:=my_station -r robot_news:=abc  
ros2 run <pkg_name> <file_name> --ros-args -r __node:=<new_node_name> -r robot_news:=<new_topic_name>  
**NOTE: Subscriber will also need to rename the topic name to get the messages**  
ros2 run my_py_pkg smartphone --ros-args -r robot_news:=abc  

### **Experiment on Topics with Turtlesim**  
ros2 run turtlesim turtlesim_node   
rqt_graph  
ros2 node info <node_name> #To see pub and sub  
ros2 topic info <topic_name> #See Pub and Sub count and msg type  
ros2 show interface <msg_type> #See what is inside the msg 
#### **Creating a publisher from the terminal**  
ros2 topic pub -r 2 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 1.0}}"  

## **ROS2 Services - Client/Server Comms between Nodes**  
### **What is ROS2 Service?**  
- Client/Server system
- Synchronous or asynchronous
- One message type for Request, one message type for Response
- Can be writtten in Python, C++, .. directly inside ROS nodes
- A service server can only exist once, bit can have many clients
### **Write a Python Service Server**  
ros2 interface show example_interfaces/srv/AddTwoInts #see what is inside the message (request and response)  
cd ~/ros2_ws/src/my_py_pkg/my_py_pkg  
touch add_two_ints_server.py  
chmod +x add_two_ints_server.py  
#### **Inside add_two_ints_server.py**  
```Python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsServerNode(Node): 
    def __init__(self):
        super().__init__("add_two_ints_server")
        self.server_ = self.create_service(AddTwoInts, "add_two_ints", self.callback_add_two_ints) #Create a service server
        self.get_logger().info("Add Twos Int started")

    def callback_add_two_ints(self, request: AddTwoInts.Request, response: AddTwoInts.Response):
        response.sum = request.a + request.b 
        self.get_logger().info(f"{request.a} + {request.b} = {response.sum}")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServerNode() 
    rclpy.spin(node)
    rclpy.shutdown()
     
     
if __name__ == "__main__":
    main()
```
cd ~/ros2_ws
colcon build --packages-select my_py_pkg  
source ~/.bashrc  
ros2 run my_py_pkg add_two_ints_server  
### **Write a Python Service Client no OOP**  
Purpose for introspecting service server when ros2 service call is not sufficient.  
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

def main(args=None):
    rclpy.init(args=args)
    node = Node("add_two_ints_client_no_oop")

    client = node.create_client(AddTwoInts, "add_two_ints") # create client
    while not client.wait_for_service(1.0): #Check and Wait for service server
        node.get_logger().warn("Waiting for Add Two Ints Server...")

    request = AddTwoInts.Request() #Request
    request.a = 3
    request.b = 8

    future = client.call_async(request) # Send request
    rclpy.spin_until_future_complete(node, future) # Spin until future is complete

    response = future.result() # Get response from the future object
    node.get_logger().info(f"{request.a} + {request.b} = {response.sum}")

    rclpy.shutdown()
     
     
if __name__ == "__main__":
    main()
```
### **Write a Python Service Client with OOP**  
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
from functools import partial

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__("add_two_ints_client")
        self.client_ = self.create_client(AddTwoInts, "add_two_ints")

    def call_add_two_ints(self, a, b):
        while not self.client_.wait_for_service(1.0): #Check and Wait for service server
            self.get_logger().warn("Waiting for Add Two Ints Server...")

        request = AddTwoInts.Request() #Create Request
        request.a = a
        request.b = b

        future = self.client_.call_async(request) # Send Request 
        future.add_done_callback(partial(self.callback_call_add_two_ints, request=request)) # Register the callback

    def callback_call_add_two_ints(self, future, request): # Get Reponse from callback
        response = future.result()
        self.get_logger().info(f"{request.a} + {request.b} = {response.sum}")

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClient()
    node.call_add_two_ints(2, 7)
    rclpy.spin(node)
    rclpy.shutdown()
     
     
if __name__ == "__main__":
    main()
```  
### **Remap a Service at Runtime**
ros2 run my_py_pkg <server/client_file_name> --ros-args -r <service_name>:=<new_service_name> 

## **Create Custom ROS2 Interfaces (Msg and Srv)**  
### **What is ROS2 Interfaces**
- Use primitive types to create a message definition
- You can create a message definition using other message definitions
- Existing interfaces: https://docs.ros.org/en/humble/Concepts/Basic/About-Interfaces.html, https://github.com/ros2/example_interfaces, https://github.com/ros2/common_interfaces  
### **Create and Build Your first custom Msg**  
cd ~/ros2_ws/src  
ros2 pkg create my_robot_interfaces  
https://roboticsbackend.com/ros2-create-custom-message/  
#### **Inside package.xml**  
add the following:  
```xml
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
```
<img width="774" height="388" alt="image" src="https://github.com/user-attachments/assets/1f938388-e6f3-422f-bc68-d80c18a54e04" />

#### **Inside CMakeList.txt**  
Add the following  
```txt
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "your custom interfaces will be here"
  "one per line"
  "no comma for separating lines"
 )
ament_export_dependencies(rosidl_default_runtime)
```
<img width="774" height="402" alt="image" src="https://github.com/user-attachments/assets/0fa79794-ffd0-43bd-9a8e-4f13b9b155bd" />

cd ~/ros2_ws/src/my_robot_interfaces  
mkdir msg  
cd ~/ros2_ws/src/my_robot_interfaces/msg  
touch HardwareStatus.msg
#### **Inside HardwareStatus.msg**  
float64 temperature  
bool are_motors_ready  
string debug_message  
colcon build --packages-select my_robot_interfaces  
source install/setup.bash  
ros2 interface show my_robot_interfaces/msg/HardwareStatus   
### **Use Your Custom Msg in a Python Node**  
cd ~/ros2_ws/src/my_py_pkg/my_py_pkg  
touch hardware_status_publisher.py  
chmod +x hardware_status_publisher.py  
#### **Inside hardware_status_publisher.py**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import HardwareStatus

class HardwareStatusPublisherNode(Node): 
    def __init__(self):
        super().__init__("hardware_status_publisher") 
        self.hw_status_pub_ = self.create_publisher(HardwareStatus, "hardware_status", 10)
        self.timer_ = self.create_timer(1.0, self.publish_hw_status)
        self.get_logger().info("HW status publisher started")

    def publish_hw_status(self):
        msg = HardwareStatus()
        msg.temperature = 43.7
        msg.are_motors_ready = True
        msg.debug_message = "Nothing special"
        self.hw_status_pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HardwareStatusPublisherNode() #
    rclpy.spin(node)
    rclpy.shutdown()
     
     
if __name__ == "__main__":
    main()
```  
If the import code is not recongnise (grey colouring) > close vs code > source > cd ~/ros2_ws/src > code .  
#### **Adding dependencies**
package.xml > `<depend>my_robot_interfaces</depend>`
#### **Setup.py**  
"hw_status_publisher = my_py_pkg.hardware_status_publisher:main"  
Build and run  
### **Create and Build your first Custom Srv**  
cd ~/ros2_ws/src/my_robot_interfaces/srv  
touch ComputeRectangleArea.srv  
#### **Inside ComputeRectangeArea.srv**  
cd ~/ros2_ws/src/my_robot_interfaces/srv  
touch ComputeRectangleArea.srv  
```srv
float64 length
float64 width
---
float64 area
```
Into CMakeList.txt > under rosidl_generate_interfaces(${PROJECT_NAME} > Add "srv/ComputeRectangleArea.srv"  
cd ~/ros2_ws  
colcon build --packages-select my_robot_interfaces   
source ~/.bashrc  
ros2 interface show my_robot_interfaces/srv/ComputeRectangleArea  

## **Change node settings at runtime with ROS2 Parameters**   
### **What is ROS2 Parameters?**  
- Settings for your nodes, value set at run time
- A parameter is specific to a node  
- ROS2 Parameter types
- Boolean
- Int
- Double
- String
- Lists
- ...
### **Using Parameters in Your Python Nodes**   
Parameters are usually added before the publisher/subscriber etc.    
Two steps process:  
- Declare parameter  
- Get parameter back  
### **YAML Parameter files**  
cd ~  
mkdir yaml_params  
cd ~/yaml_params  
touch number_params.yaml  
gedit number_params.yaml  
#### **Inside number_params.yaml**  
```yaml
/number_publisher2:
  ros__parameters:
    number2: 5
    timer_period: 2.0
    
/number_publisher3:
  ros__parameters:
    number2: 3
    timer_period: 3.0
```
node names must make to get the specific parameters   
node name obtained from ros2 node list   
params names obtained from ros2 param list   
ros2 run my_py_pkg number_publisher2 --ros-args --params-file ~/yaml_params/number_params.yaml   
ros2 run my_py_pkg number_publisher2 --ros-args -r __node:=number_publisher3 --params-file ~/yaml_params/number_params.yaml  #get the parameters of number_publisher3  

