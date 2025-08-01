### **Tools**

rqt\_graph # network of ros2 communications running (will not show services)

ros2 node list # show list of nodes

ros2 node info \[node name] # show info of the node (like subscriber and publisher)

ros2 topic list # show list of topics

ros2 topic info \[topic name] # show info of the topic like the msg, publisher and subscriber

ros2 interface show \[msg type] # show what the msg contains (e.g. linear x.y.z)

ros2 topic echo \[topic name] # show the print result of the topic 

ros2 service list # show list of services

ros2 node list # show list of nodes

ros2 service type \[service name] # show type of service node

ros2 interface show \[service type] # show info the type of service request and reponse 

ros2 run \[package name] \[py file] # running file in ros2 command

ros2 service call \[service name] \[service type] # calling to input request

ros2 topic hz \{topic name] # get the hz of the topic (how messages per second)

### **Create and setup a workspace**

#### Install colcon (1 time only)

sudo apt update  
sudo apt install python3-colcon-common-extensions  

#### Source colcon (1 time only)

gedit ~/.bashrc  
add > source /usr/share/colcon\_argcomplete/hook/colcon-argcomplete.bash (below source ros2) > save  

#### Making workspace

cd ~  
mkdir ros2\_ws  
cd ros2\_ws/  
mkdir src  
In ros2 ws directory > colcon build  
ls > should have build install log src  
cd install  
ls > look for setup.bash  

#### Source setup.bash

gedit ~/.bashrc  
add > source ~/ros2\_ws/install/setup.bash (below colcon source)> save

### **Create a ROS2 Python Package**

cd ros2\_ws/src  
ros2 pkg create my\_robot\_controller --build-type ament\_python --dependencies rclpy  
cd ~/ros2\_ws/src code . (open folder in vs studio)  
cd ~/ros2\_ws > colcon build  
cd ~/ros2\_ws/install > ls > look for my\_robot\_controller (Able to use ros2 commandline tools like ros2 run)

### **Create a ROS2 Node with Python and OOP**

cd ~/ros2\_ws/src/my\_robot\_controller/my\_robot\_controller > touch my\_first\_node > chmod +x my\_first\_node

#### **Inside my\_first\_node.py**

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node):

    def __init__(self):
        super().__init__("first_node") #node name
        #self.get_logger().info("Hello from ROS2!!!")
        self.counter_ = 0 # Adds attribute
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("Hello " + str(self.counter_))
        self.counter_ += 1


def main(args=None):
    #start ros2 comms
    rclpy.init(args=args)
    
    node = MyNode()
    
    rclpy.spin(node) #kept alive until it is killed
    
    #shutdown ros2 comms
    rclpy.shutdown

if __name__ == '__main__': #directly execute file from the terminal
    main()
```
cd ~/ros2\_ws/src/my\_robot\_controller/my\_robot\_controller > python3 my\_first\_node.py

#### **How to use with ros2 commands**

setup.py > under entry\_points > under console\_scripts array > "test\_node = my\_robot\_controller.my\_first\_node:main" (\[executable name] = \[file name])> save  
cd ~/ros2\_ws > colcon build > source ~/.bashrc > ros2 run my\_robot\_controller test\_node

#### **How to skip the colcon build if i update the file**

cd ~/ros2\_ws > colcon build --symlink install > source ~/.bashrc  
ros2 run my\_robot\_controller test\_node  
Edit my\_first\_node.py string text > save > ros2 run my\_robot\_controller\_node test\_node > look for the change made

### **Write a ROS2 Publisher with Python**

#### **Create a py file as a publisher**

cd ~/ros2\_ws/src/my\_robot\_controller/my\_robot\_controller > touch draw_circle.py > chmod +x draw_circle.py

#### **Inside draw\_circle.py**

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
class DrawCircleNode(Node):

    def __init__(self):
        super().__init__("draw_circle")
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10) # (msg_type, topic name, queue size)
        self.timer_ = self.create_timer(0.5, self.send_velocity_command)
        self.get_logger().info("Draw circle node has been started")

    def send_velocity_command(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        self.cmd_vel_pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DrawCircleNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__': #directly execute file from the terminal
    main()
```

#### **Getting the msg to publish**
ros2 topic list > look for /turtle1\_cmd_vel  
ros2 topic info /turtle1/cmd_vel > look at Type section > geometry_msgs/msg/Twist  
Import the msg to draw\_circle file  

#### **Adding dependencies**
In package.xml file > below `<depend>rclpy<depend>` > add `<depend>geometry_msgs<depend>`

#### **What input do i need**
ros2 interface show geometry_msgs/msg/Twist > see the inputs of the msg > put the desired linear and angular velocities to the 'send_velocity_command' function

#### **Adding console script**
In setup.py file > under entry_points > add "," behind an exising node if there is > new line > add "draw_circle = my_robot_controller.draw_circle:main"

#### **Building and running the publisher**
cd ~/ros_ws > colcon build --symlink-install  
ros2 run turtlesim turtlesim_node  
ros2 run my_robot_controller draw_circle  

### **Write a ROS2 Subscriber with Python**
#### **Finding the msg type, topic name and what is inside the msg** 
rqtgraph > find the \[topic name] published of interest  
ros2 topic echo /turtle\_pose # see what are we getting  
ros2 topic info /turtle\_pose % get msg type  
ros2 interface show /turtle\_pose # get what is inside the msg  

#### **Create a py file as a subscriber**  
cd ~/ros2\_ws/src/my\_robot\_controller/my\_robot\_controller > touch pose_subscriber.py > chmod +x pose_subsriber.py  

#### **Inside pose/subscriber.py**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class PoseSubscriberNode(Node):

    def __init__(self):
        super().__init__("pose_subscriber")
        self.pose_subscriber_ = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10)

    def pose_callback(self, msg: Pose): # make msg default to Pose
        #self.get_logger().info(str(msg)) # get all the msg
        self.get_logger().info("("+ str(msg.x) + "," + str(msg.y) + ")") # access certain coordinates

def main(args=None):
    rclpy.init(args=args)
    node = PoseSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__': #directly execute file from the terminal
    main()
```

#### **Adding dependencies**
In package.xml file > below `<depend>geometry_msgs<depend>` > add  `<depend>turtlesim<depend>`

#### **Adding console script**
In setup.py file > under entry_points > add "," behind an exising node if there is > new line > add "pose_subscriber = my_robot_controller.pose_subscriber:main"

#### **Building and running the publisher**
cd ~/ros_ws > colcon build --symlink-install  
ros2 run turtlesim turtlesim_node  
ros2 run my_robot_controller pose_subscriber  

### **Create a Closed Loop System with a Publisher and a Subscriber**
Create a node that subscribe to /turtle1\_pose and publish to /turle1\_cmd_vel  
cd ~/ros2\_ws/src/my\_robot\_controller/my\_robot\_controller > touch turtle_controller.py > chmod +x turtle_controller.py  

#### **Inside pose_subscriber.py**  
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TurtleControllerNode(Node):

    def __init__(self):
        super().__init__("turtle_controller")
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10)
        self.get_logger().info("Turle controller has started.")

    def pose_callback(self, pose: Pose): # when you receive a pose we send a cmd to go basically get subscribe and then publish
        cmd = Twist()
        if pose.x > 9.0 or pose.x < 2.0 or pose.y > 9.0 or pose.y < 2.0: # if turtle close to wall
            cmd.linear.x = 1.0
            cmd.angular.z = 0.9
        else: 
            cmd.linear.x = 5.0
            cmd.angular.z = 0.0
        self.cmd_vel_publisher_.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__': #directly execute file from the terminal
    main()
```  
#### **Adding console script**
In setup.py file > under entry_points > add "," behind an exising node if there is > new line > add "turtle_controller = my_robot_controller.turtle_controller:main"

#### **Building and running the publisher**
cd ~/ros_ws > colcon build --symlink-install  
ros2 run turtlesim turtlesim_node  
ros2 run my_robot_controller turtle_controller  

### **What is ROS2 Service**  
Can only have one service but as many cilents as you want

#### **Changing colour of the turtle's trail**  
ros2 run turtlesim turtlesim+_node  
ros2 service list  #show list of service 
ros2 service type /turtle1\_set_pen  # show service type
ros2 interface show turtlesim/srv\_SetPen # show requests and responses  
ros2 service call /turtle1\_set_pen turtlesim/srv\_SetPen "{'r': 255, 'g': 0, 'b': 0, 'width': 3, 'off': 0}"

### **Write a ROS2 Service Client with Python**  
Going to change the colour of the turtle's trail when it red on the right and green on the left

#### **Inside turtle_controller.py**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
from functools import partial

class TurtleControllerNode(Node):

    def __init__(self):
        super().__init__("turtle_controller")
        self.previous_x_ = 0.0
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10)
        self.get_logger().info("Turle controller has started.")

    def pose_callback(self, pose: Pose): # when you receive a pose we send a cmd to go basically get subscribe and then publish
        cmd = Twist()
        if pose.x > 9.0 or pose.x < 2.0 or pose.y > 9.0 or pose.y < 2.0: # if turtle close to wall
            cmd.linear.x = 1.0
            cmd.angular.z = 0.9
        else: 
            cmd.linear.x = 5.0
            cmd.angular.z = 0.0
        self.cmd_vel_publisher_.publish(cmd)

        #Problem with this method: will be called so many times that the application will lag
        #Another way to only call the service when needed and not all the time to reduce load
        # if pose.x > 5.5:
        #     self.get_logger().info("Set colour to red")
        #     self.call_set_pen_service(255,0,0,3,0)
        # else:
        #     self.get_logger().info("Set colour to green")
        #     self.call_set_pen_service(0,255,0,3,0)

        #add more condition to reduce the need to call it constantly and only when required
        if pose.x > 5.5 and self.previous_x_ <= 5.5:
            self.previous_x_ = pose.x
            self.get_logger().info("Set colour to red")
            self.call_set_pen_service(255,0,0,3,0)
        elif pose.x <= 5.5 and self.previous_x_ > 5.5:
            self.previous_x_ = pose.x
            self.get_logger().info("Set colour to green")
            self.call_set_pen_service(0,255,0,3,0)

    #Template for service cilent 
    def call_set_pen_service(self, r, g, b, width, off):
        cilent = self.create_client(SetPen, "/turtle1/set_pen")
        while not cilent.wait_for_service(1.0): # if service call not availabe for 1 second
            self.get_logger().warn("Waiting for service to be available...")
        
        request = SetPen.Request()
        request.r = r
        request.b = b
        request.g = g
        request.width = width
        request.off = off

        future = cilent.call_async(request) # send request to service and create a future object
        future.add_done_callback(partial(self.callback_set_pen)) 

    def callback_set_pen(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed: %r" % (e,))
    #-------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__': #directly execute file from the terminal
    main()
```
#### **Running turtle_controller with turtlesim_node**  
ros2 run turtlesim turtlesim_node
ros2 run my_robot_controller turtle_controller
