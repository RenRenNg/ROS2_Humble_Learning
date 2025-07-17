# **ROS2 Learning**
## **Source**  
https://www.youtube.com/redirect?event=video_description&redir_token=QUFFLUhqbkE1a2FONnlCNVNhZThXTjlkVGNCRDk0YURMd3xBQ3Jtc0trMzNFT21SUHdGRG1wZVVPVGxfZjNMcU5aS09BNGllR2JZdEp0TVlDalRvbjdXcTJtSF9BMzR0TktQWnRkem5VYjM4ZmpRUTBlWmtRZjhFNUtNTk1SRTdQUlVncS1MWGRDM1ZXdE5Hdm01MVBsWERROA&q=https%3A%2F%2Frbcknd.com%2Fros2-for-beginners&v=Gg25GfA456o

## **Install ROS2 and Setup your enivronment**
In this tutorial, I will be using Humble instead of Jazzy as I am using Ubuntu 22.04 Native   
Follow the instructions of installation in https://docs.ros.org/en/humble/Installation.html  

### **Launch ROS2 Program**
ros2 run  demo_nodes_cpp talker  
ros2 run demo_nodes_cpp listener  

## **Write your first ROS 2 program**  
### **Create a ROS2 Workspace**
cd ~  
mkdir ros2_ws  
cd ros2_ws/  
mkdir src
#### **Building**  
cd ~/ros2_ws
colcon build
ls > should see build install and log folders  
#### **Sourcing workspace**
cd ~/ros2_ws/install  
ls > there is a setup.bash file that need sourcing
cd ~  
gedit .bashrc
Add > source cd ~/ros2_ws/install/setup.bash at the last line > SAVE  


