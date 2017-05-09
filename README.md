# Modified ROS wrapper for pocketsphinx  
Original repository: https://github.com/mikeferguson/pocketsphinx  
  
Also used repo: https://github.com/gorinars/ros_voice_control  
The script shows how to control ROS turtlebot with English keywords using pocketsphinx.  
It used up-to-date pocketsphinx features and is independent of most external dependencies.  
  
Current repository is a ROS wrapper which incorporates those features.  
  
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
    ###libpulse-dev  
    ###swig

## Getting Started
Clone this repository into the src folder of your catkin workspace using:
``` 
git clone https://github.com/Pankaj-Baranwal/pocketsphinx
```
To know more about catkin workspace and ROS, follow instructions at: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment  
After everything is setup, open a terminal from your catkin workspace and type the following command:  
``` 
catkin_make
```
Now, you are all setup. To test the package:  
1) You should have a dictionary file (with extension ".dict") and a kws list (with extension ".kwlist") ready.  
2) Please ensure that roscore is running.
3) Use the following command to start the required node: (Also add the 2 arguments) 
``` 
rosrun pocketsphinx voice_control_updated.py
```
In case you just want to test the package out, a sample dictionary and kws list have already been placed in the demo folder within the package. Just use this command from within the folder:  
``` 
rosrun pocketsphinx voice_control_updated.py _dict:=voice_cmd.dic _kws:=voice_cmd.kwlist
```
For instructions on how to run this node with turlebot simulation, please read the instructions at:  
https://github.com/gorinars/ros_voice_control
