# **ROS2 Humble Learning**
## **Useful Tools**  
cd #moving directories  
mkdir #Make folder  
ls #Show list of files in current directory  
gedit #edit certain file code  

## **Source**  
https://www.youtube.com/redirect?event=video_description&redir_token=QUFFLUhqbkE1a2FONnlCNVNhZThXTjlkVGNCRDk0YURMd3xBQ3Jtc0trMzNFT21SUHdGRG1wZVVPVGxfZjNMcU5aS09BNGllR2JZdEp0TVlDalRvbjdXcTJtSF9BMzR0TktQWnRkem5VYjM4ZmpRUTBlWmtRZjhFNUtNTk1SRTdQUlVncS1MWGRDM1ZXdE5Hdm01MVBsWERROA&q=https%3A%2F%2Frbcknd.com%2Fros2-for-beginners&v=Gg25GfA456o

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
#### **Create a Python Package**  
Packages will allow you to separate your code into reusable blocks.  
Each package is an independent unit.  
For example, in one application you could have one package that will handle a camera, another package that will run the wheels of your robot, and yet another one that will handle motion planning for the robot in the environment.  
cd ~/ros2_ws/src/
ros2 pkg create
