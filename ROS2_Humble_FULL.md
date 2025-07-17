# **ROS2 Humble Learning**
## **Useful Tools**  
cd #moving directories  
mkdir #Make folder  
touch #create a file with format (.py, .yaml)  
chmod +x #change file permissions  
ls #Show list of files in current directory  
ls -la #Show hidden files  
gedit #edit certain file code  
code . #launch vs code from current directory of the terminal  
### **ROS2 Tools**  
colcon build #cpp  
colcon build --packages-select <package_name_1> <package_name_2> ...  #Build selected packages
ros2 pkg create [pkg_name] --build-type ament_python --dependencies rclpy #Create a pkg with py version and dependcies  
ros2 run  

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
ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy  
##### **Inside my_py_pkg**  
Inside my_py_pkg > my_py_pkg: This is where you write the py codes.  
Inside package.xml: FIll up license if you publishing the code, dependencies, build type  
Inside setup.py: Use when installing node under console scripts  
