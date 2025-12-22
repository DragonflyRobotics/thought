---
title: Beginner’s Guide to ROS — Part 2 “Installation and First Run” Tutorial
description: Alright, be honest. We all were like, “ROS sounds cool.” Let me install it and see how it works, plus how hard can installing it be(famous last words). 
slug: ros-beginners-guide-part-2
date: 2021-11-08 00:00:00+0000
image: cover.jpg
categories:
    - ROS 
    - Robotics
    - Tutorial
tags:
    - Robotics
    - ROS
    - Tutorial
weight: 1       # You can add weight to some posts to override the default sorting (date descending)
---

Alright, be honest. We all were like, “ROS sounds cool.” Let me install it and see how it works, plus how hard can installing it be(famous last words). Then you search up “ROS install Ubuntu”, and you find 50 guides all saying something different. Some require 4 hours of compiling while others simply use ATP. What is the deal with all of this?

## Method 1: Easy way out!

[https://imgr.search.brave.com/HrMbmi1kSMEnJflnt8rSZnsbBd8q3LKt_4gurC8rsos/fit/602/606/ce/1/aHR0cHM6Ly9pLnBp/bmltZy5jb20vb3Jp/Z2luYWxzL2Q5LzJm/L2RkL2Q5MmZkZGQ1/NGFkOTA3NTQ5NWY0/ODdjMTRhM2ZjODQ1/LmpwZw](1.webp)
Now how this one works is simple. ROS only supports LTS versions of Ubuntu. That is 20.04, 18.04, 16.04, 14.04, etc. Now for each LTS version, there is a version of ROS. For 20.04, it is Noetic Ninjemys, for 18.04, it was Melodic Morenia, etc. You must be wondering, “Can you communicate between other versions of ROS?” Absolutely! They will communicate flawlessly with each other. They communicate over an IP Protocol that is common and reverse compatible. It is a little difficult to communicate between ROS1 and ROS2 since you must use ros_bridge. But that’s for another day!

### For Ubuntu 20.04 (Noetic Ninjemys)

Now, unless you have some crazy dependency, the latest ROS1 version is the way to go for beginners. The latest as of December 12, 2021, is ROS Noetic. Installing previous versions will be a little harder. 1 more thing, I will show installation on Ubuntu 20.04 but other operating systems like Arch, Windows, and Debian are available. Here is how you can install Noetic Ninjemys:

Link to other OS installations: http://wiki.ros.org/noetic/Installation

```bash
(Usually done by default) Make sure "restricted," "universe," and "multiverse" are enabled. Check https://help.ubuntu.com/community/Repositories/Ubuntu
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' # Add their repository to your machine
sudo apt install curl # Usually comes preinstalled
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - # Add repository key
sudo apt update # Update repository listing
sudo apt install ros-noetic-desktop-full # Contains all essential packages like perception and simulation. Perfect for beginers. Others are "ros-noetic-ros-base" which is the bare-bones. The middle version is "ros-noetic-desktop" which had some debug tools but none of the perception and simulation packages. You must source the ROS environment everytime so you can run "source /opt/ros/noetic/setup.bash" but if you don't want to run it everytime you spawn a terminal
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
# Close all terminal instances and reopen terminal.
# Update ROS Core Dependencies:
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential # Install tools
sudo apt install python3-rosdep # Install rosdep(tool)
sudo rosdep init # Initialize rosdep
rosdep update # Update rosdep
```

That’s it!

![https://imgr.search.brave.com/axjiqL7zpc8jGQb8kFbQhr1OBrQGgVcOUdtLqtccFZg/fit/1000/1000/ce/1/aHR0cHM6Ly9paDAu/cmVkYnViYmxlLm5l/dC9pbWFnZS4yOTYy/NjMwMTkuMDUzMi9m/bGF0LDEwMDB4MTAw/MCwwNzUsZi51My5q/cGc](2.webp)

## Method 2: From Source (Advanced Users)
![https://imgr.search.brave.com/sejDYrgkO1uLxFatbZfl_mUjwZsSjn6fVZjSQtNMab8/fit/800/1200/ce/1/aHR0cHM6Ly9pLnBp/bmltZy5jb20vb3Jp/Z2luYWxzL2NkL2Rm/LzdlL2NkZGY3ZWE0/OGFkM2Q1NDA3ZTJk/ZjY3NDU4Nzc4N2Jh/LmdpZg.gif](3.gif)
There is a separate way where you can install and compile all the ~150 packages from source. This is for the professional developers or the confused beginners(get it?). Please let me know in the comments if you want me to make a separate article on this!

--- 

## Setup Development Environment

Alright, the installation is finally complete. Not so fast! We still have to make the development environment. Don’t worry, it is SUPER easy!

```bash 
mkdir WORKSPACE_NAME # Go to the place you want your code, environment, and packages to reside. Replace WORKSPACE with a name of your choice. 
cd WORKSPACE_NAME # Go into directory
mkdir src # DO NOT change this name or it won't compile. Also, DO NOT cd into it
catkin_make # compile your blank project!
```

---

## Take if for a Spin!
Finally, the moment you have all been waiting for…your very first ROS1 command!

![https://imgr.search.brave.com/JXshFVJjUjk5DWOlW7lsKhJQ0m6a_1uJMGcl68P-qk8/fit/750/1000/ce/1/aHR0cHM6Ly9paDEu/cmVkYnViYmxlLm5l/dC9pbWFnZS44NzUx/MTE5MDUuNDc5OC9m/bGF0LDc1MHgsMDc1/LGYtcGFkLDc1MHgx/MDAwLGY4ZjhmOC5q/cGc](4.webp)

### Note:
Please note: this article shows you how to set up ROS on 1 computer. Meaning that the Master and Nodes are on the same computer. TO communicate across many computers, there is a painful setup procedure that I won’t make you endure just yet(brace yourself for the future).

### ROS Core
ROS1(not ROS2) works on the following system:
![https://upload.wikimedia.org/wikipedia/commons/e/e7/ROS-master-node-topic.png](5.png)
ROS Core manages the master node. It manages all the communication happening in the ROS network. You cannot run any ROS nodes without it.

```bash
roscore # Spawns a ROS master node.
```

### Turtle Sim
![](7.webp)

The first thing you are going to do is make a little turtle draw things on a screen using your arrows.

If you are short on time, install it using APT:

```bash
sudo apt-get install ros-$(rosversion -d)-turtlesim # install the turtle sim package
```

If you want to learn ROS properly, do this:

```bash 
cd WORKSPACE_NAME/src # Go to the source directory
sudo apt install git # Install git if you don't have it.
git clone https://github.com/ros/ros_tutorials.git # Download the turtle_sim and lesson packages
cd .. # Go to main directory
catkin_make # Compile the project
source devel/setup.bash # Re-Source compiled files.
```

Now, let us run it! Make sure roscore is running on a different terminal

```bash 
rosrun turtlesim turtlesim_node
# command   node       script
```

This should show a box where you can move a turtle around. But more importantly, is the things happening behind the scene.

```bash
rosnode list
```

What do you see? Can you find our node?

```bash
rostopic list
```

Where is the turtle location topic? Did you see it?

```bash
rostopic <TURTLE_POSE_TOPIC_NAME> echo
```

Replace the first argument with the topic name you previously found. Can you see the coordinates move as the turtle runs around(or should I say crawl around)?

### Things to note:

**cmd_vel**: This is what we call a Twist message. It contains target velocity values for 3 axes of translation and 3 axes of rotation. You can imagine how useful this is for a remote-controlled robot.

**Command structure**: Did you notice the pattern in the commands. It is the object you are accessing(rosnode, rosservice, rostopic, rosrun, etc), then the action you want to perform(echo, list, hz, etc), then the arguments. We will dive into the depth of these commands in another article.
