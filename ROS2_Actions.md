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
```plaintext
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


#### **Axes**  

