# ROS package for pocketsphinx  
Original repository: https://github.com/mikeferguson/pocketsphinx  
  
Also used repo: https://github.com/gorinars/ros_voice_control  
  
You can know more about pocketpshinx here: https://cmusphinx.github.io/  

## Dependencies  
1) pyaudio  
    ```
    sudo pip install pyaudio
    ```  
    If this does not work, follow instructions below:
    ```
    sudo apt-get install libasound-dev
    sudo apt-get install python-pyaudio
    ```
2) pocketsphinx: You will need to have pip preinstalled for this to work
    ```
    sudo pip install pocketsphinx
    ```
    There are many dependencies which need to be met before installation of pocketsphinx through pip works.
    Use Synaptics package manager to install the unmet dependencies which would be mentioned as error messages on the terminal window in case installation fails. Some of them include:  
    libpulse-dev  
    swig

## Getting Started
Clone this repository into the src folder of your catkin workspace using:  
```  
cd ~/catkin_ws/src
git clone https://github.com/Pankaj-Baranwal/pocketsphinx
```
To know more about catkin workspace and ROS, follow instructions at: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment  
After everything is setup, open a terminal from your catkin workspace and type the following command:  
``` 
catkin_make
```
Now, you are all setup. To test kws(keyword spotting) mode:  
1) You can use the default test files including a dictionary file and a kws list.  
2) Launch roscore in a termincal window:  
```
roscore
```
2) Use the following command to start the required launch file from demo folder inside this package:  
``` 
roslaunch pocketsphinx kws.launch input:=goforward.raw dict:=voice_cmd.dic kws:=voice_cmd.kwlist
```  
Please note that the files provided as input need to be present in the **demo** folder withing this package.  
Remove the input parameter to enable continuous input from your system's microphone:
```
roslaunch pocketsphinx kws.launch dict:=voice_cmd.dic kws:=voice_cmd.kwlist
```
3) You can see the output as a message on the topic "kws_data"
```
rostopic echo /kws_data
```
Voila! You are good to go! You can read more about the different features and their usages in the wiki.  
For running this node with turtlebot_gazebo simulation, follow these instructions:  
4) Follow the above 3 steps.  
5) Install turtlebot simulation packages. Instructions can be found on its ROS wiki.  
If they do not work, you can use these instructions: https://docs.google.com/document/d/1gRBlLUdsWePjek5WvIHpK88wv1jqKmL901IvruDaM_I/edit?usp=sharing  
6) Launch turtlebot_gazebo:  
```  
roslaunch turtlebot_gazebo turtlebot_world.launch
```
7) Launch voice_control_example.py:  
```
rosrun pocketsphinx voice_control_example.py
```
And now, your turtlebot simulation will follow your voice commands!
Words in the dictionary include: "forward", "back", "stop", "half speed", "full speed", "left", "right" etc  
  
Further details can be found in the wiki!
