### **Tools**

rqt\_graph # network of ros2 communications running

ros2 node list # show list of nodes

ros2 node info \[node name] # show info of the node (like subscriber and publisher)

ros2 topic list # show list of topics

ros2 topic info \[topic name] # show info of the topic like the msg, publisher and subscriber

ros2 interface show \[msg type] # show what the msg contains (e.g. linear x.y.z)

ros2 topic echo \[topic name] # show the print result of the topic 

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
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
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
ros2 topic info /turtle1\_cmd_vel > look at Type section > geometry_msgs\_msg\_Twist
Import the msg to draw\_circle file

#### **Adding dependencies**
package.xml file > below "`<depend>rclpy<depend>`" >add "`<depend>geometry_msgs<depend>`"
