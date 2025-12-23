---
title: Beginner’s Guide to ROS — Part 3 “Topics In-Depth” Tutorial
description: Let's dive deeper into ROS topics and understand how they work under the hood.
slug: ros-beginners-guide-part-3
date: 2022-03-18 00:00:00+0000
image: cover.png
categories:
    - ROS 
    - Robotics
    - Tutorial
tags:
    - ROS 
    - Robotics
    - Tutorial
weight: 1       # You can add weight to some posts to override the default sorting (date descending)
---

## Recap: What is a Topic

Remember from the [first part]({{< relref "post/ros-part-1" >}}) of this series, topics are a channel(a pipe), through which communication between nodes(sets of code) occurs. If you want to send sensor data from a microcontroller on a robot to another node on another machine(note how a machine can contain SEVERAL nodes), this would be the way to go.

Also, recall how a node can publish a topic and push data to it. Any other node on the network can anonymously subscribe to the topic and digest the information it can provide.

All good? Right then, let's get out hands dirty with some code and some errors as well!

---

## Format of this and future articles:

It is important to understand how this article will go. We will go through an example together. I believe that knowledge can only be attained through practical examples.

---

## Task 1
We have a gyroscope on the robot that is publishing data in this format:

```plaintext
Data:    135.6,    13.4,   3.14159
Format: FLOAT64  FLOAT64  FLOAT64
Axis:    roll      pitch    yaw
```

**NOTE**: The Format and Axis are NOT part of the data. They are just extra information I added.

### Data Formatting

We must first consider the format and structure of the data. This is not multidimensional data or a specialized format like an Image or Audio sample. This makes our life very easy.

![](2.webp)

Next, we must consider the amount of data and its relationship. The roll, pitch, and yaw axes correspond to each other so it is impractical to send them individually like this:

```plaintext
135.6
13.4
3.14159
```

![](3.webp)

ROS has the functionality to make a custom topic type that incorporates 3 `Float64` in a single message. But, that is way more complicated and beyond the scope of this article(there may be an article on this soon though).

We can also make this a comma-separated string. This way, we can send a single String message that contains the following information:

```plaintext
"135.6, 13.4, 3.14159"
```

The subscriber can then easily splice the string and extract their information. That won’t be too intensive and still maintain the time-dependent structure of the data.

## Programming

### Making the ROS Package
First, we have to make a ROS package which will be our node. That node will contain the code to make a topic and publish the message. There is a built-in command to do this using catkin.

First, though, navigate to the workspace:

```bash
cd catkin_ws/src
```

*NOTE*: You MUST be in src.

Then run this:

```bash
catkin_create_pkg basic_ros_topic std_msgs rospy roscpp --pkg_version 0.0.1 --description "Basic ROS Topic" --license MIT --author "Krishna" --maintainer "Krishna"
```

Yikes! That is a lot so let me break it down.

 * First, we have `catkin_create_pkg` which is the base command.
 * Next comes the name of the package, in this case `basic_ros_topic` . This can be anything.
 * Then, we add all the dependencies of the package. Here, we have 3: `std_msgs` `rospy` `roscpp`
   * `stg_msgs` is a ROS package that contains a bunch of data types like `Float64`, `String`, `Int64`, and more!
   * `rospy` is a library that contains methods for using ROS with Python. This is what we will use today.
   * `roscpp` is a library that contains methods for using ROS with C++. We will NOT use this today.
   * On a side note: ROS also supports Java, Lisp, and some other languages too!

Everything that is mandatory to install has been done. Now on to the optional stuff.

    These parts are really self-explanatory. It includes the license, description, author, maintainer, and version.
    You don’t need this but it is a good practice to keep.

Now, list the files in the directory. You will see this:

```plaintext
basic_ros_topic  CMakeLists.txt
```

Go to `basic_ros_topic` and list again.

```plaintext
CMakeLists.txt  include  package.xml  src
```

 * `CMakeLists.txt` — contains all the info catkin needs to compile the project. It contains a list of the libraries, files, packages, and other stuff it has to compile. We will come back to it later.
 * `package.xml` — contains info like package owner, description, version, maintainer, license, etc. We filled some of it earlier. The only important thing in it is the dependencies definitions. Our command already did this previously.

It looks like this:
```xml 
***A BUNCH OF OTHER STUFF***
<!-- The *depend tags are used to specify dependencies -->
  <!-- Dependencies can be catkin packages or system dependencies -->
  <!-- Examples: -->
  <!-- Use depend as a shortcut for packages that are both build and exec dependencies -->
  <!--   <depend>roscpp</depend> -->
  <!--   Note that this is equivalent to the following: -->
  <!--   <build_depend>roscpp</build_depend> -->
  <!--   <exec_depend>roscpp</exec_depend> -->
  <!-- Use build_depend for packages you need at compile time: -->
  <!--   <build_depend>message_generation</build_depend> -->
  <!-- Use build_export_depend for packages you need in order to build against this package: -->
  <!--   <build_export_depend>message_generation</build_export_depend> -->
  <!-- Use buildtool_depend for build tool packages: -->
  <!--   <buildtool_depend>catkin</buildtool_depend> -->
  <!-- Use exec_depend for packages you need at runtime: -->
  <!--   <exec_depend>message_runtime</exec_depend> -->
  <!-- Use test_depend for packages you need only for testing: -->
  <!--   <test_depend>gtest</test_depend> -->
  <!-- Use doc_depend for packages you need only for building documentation: -->
  <!--   <doc_depend>doxygen</doc_depend> -->
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>

  <!-- The export tag contains other, unspecified, tags -->
  <export>
    <!-- Other tools can request additional information be placed here -->
  </export>
</package>
```

That bottom part with the `buildtool_depend` and `build_depend` statements define the important stuff.

Navigate to the `src` within that. Now you should be in `USERNAME/catkin_ws/src/PACKAGE_NAME/src`.

Here is where we put all of our code! Make a file called `GyroPublisher.py`. Again, names aren’t super important.

Here is your file structure right now:

```plaintext
catkin_ws
  - build
  - devel
  - src
    - basic_ros_topic
      - CMakeLists.txt
      - package.xml
      - include {This directory contains nothing}
      - CMakeLists.txt
      - src
        - GyroPublisher.py
```

Now let’s start on the code itself. First, you have to import the `rospy` and `std_msgs`.

```python3
#!/usr/bin/env python3import rospy
from std_msgs.msg import String
```

Now we make the function that publishes the actual messages.

```python3
def publisher():
    publisher = rospy.Publisher('gyro_rpy', String, queue_size=10) # Make a publisher with a topic called "gyro_rpy" with the type String. queue_size is the max amount of messages that can be queued.
    rospy.init_node('GyroData', anonymous=True) # Make a node called "GyroData". Anonymous means that the name will be "GyroData" with a lot of random numbers.
    rate = rospy.Rate(10) # 10Hz
    while not rospy.is_shutdown(): # Run forever until exit.
        roll = 135.6 # set values
        pitch = 13.4 # set values
        yaw = 3.14159 # set values
        gyro_mock_data = f"{roll}, {pitch}, {yaw}," # Combine the strings together
        rospy.loginfo(gyro_mock_data) # log the Publisher data
        publisher.publish(gyro_mock_data) # Publish
        rate.sleep() # Slow down the publishing to not choke CPU.
```

And then we can run it with this:

```python3
if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        print("Publisher Interrupted...\nExiting cleanly...")
        exit()
```

**IMPORTANT**: You much make sure that your Python file is executable. `sudo chmod +x GyroPublisher.py`

Go the `CMakeLists.txt` and add our python script to the file. This will make sure that catkin will compile it.

```plaintext
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  src/GyroPublisher.py
)
```

Now go back to `catkin_ws`. Run `catkin_make`. Source the directory `source devel/setup.bash`.

### Running the Node

Start roscore: `roscore`. Now, start publishing: `rosrun basic_ros_topic GyroPublisher.py`.

Check the node using `rosnode list`. We will see our `GyroData` node with a bunch of numbers since it is anonymous.

Check the topic using `rostopic list`. We will see our `gyro_rpy` topic. We can actually see the data using `rostopic echo /gyro_rpy`:

```plaintext
data: "135.6, 13.4, 3.14159,"
---
data: "135.6, 13.4, 3.14159,"
---
data: "135.6, 13.4, 3.14159,"
---
data: "135.6, 13.4, 3.14159,"
---
data: "135.6, 13.4, 3.14159,"
```

### Receive Data Programmatically

Import libs:

```python3
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
```

Define callback to log:

```python3
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + f"Recieved Data: {data.data}") # Log data
```

Make the subscriber node:

```python3
def sub():
    rospy.init_node('sub', anonymous=True) # Initialize the noderospy.Subscriber("gyro_rpy", String, callback) # Start a subscriber. The name here is the name of the topic we want to subscribe to.rospy.spin() # Keep the node running until program exits.
```

Run the script:

```python3
if __name__ == '__main__':
    sub()
```

Again, remember to change the permissions of the file. And run it the same way as the publisher.

## Conclusion
In this article, you learned how to make Publishers and Subscribers in ROS. If you encounter any errors or have any questions, leave a comment. I will try my best to help you. I hope you enjoyed it. If there is a particular topic in ROS that you want me to cover, let me know in the comments as well!
