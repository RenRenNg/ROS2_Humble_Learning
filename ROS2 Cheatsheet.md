### **Tools**

rqt\_graph # network of ros2 communications running

ros2 node list # show list of nodes

ros2 node info \[node name] # show info of the node (like subscriber and publisher)



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

\#!/usr/bin/env python3

import rclpy

from rclpy.node import Node



class MyNode(Node):





    def \_\_init\_\_(self):

        super().\_\_init\_\_("first\_node") #node name

        self.get\_logger().info("Hello from ROS2")





def main(args=None):

    #start ros2 comms

    rclpy.init(args=args)

 

    node = MyNode()

 

    rclpy.spin(node) #kept alive until it is killed

 

    #shutdown ros2 comms

    rclpy.shutdown



if \_\_name\_\_ == '\_\_main\_\_': #directly execute file from the terminal

    main()

---

cd ~/ros2\_ws/src/my\_robot\_controller/my\_robot\_controller > python3 my\_first\_node.py

#### **How to use with ros2 commands**

setup.py > under entry\_points > under console\_scripts array > "test\_node = my\_robot\_controller.my\_first\_node:main" (\[executable name] = \[file name])> save

cd ~/ros2\_ws > colcon build > source ~/.bashrc > ros2 run my\_robot\_controller test\_node

#### **How to skip the colcon build if i update the file**

cd ~/ros2\_ws > colcon build --symlink install > source ~/.bashrc

ros2 run my\_robot\_controller test\_node

Edit my\_first\_node.py string text > save > ros2 run my\_robot\_controller\_node test\_node > look for the change made



### **What is a ROS2 topic**

asd

