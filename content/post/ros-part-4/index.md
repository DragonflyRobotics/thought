---
title: Beginner’s Guide to ROS — Part 4 “Services In-Depth” Tutorial
description: Let's dive deeper into ROS topics and understand how they work under the hood.
slug: ros-beginners-guide-part-4
date: 2022-04-01 00:00:00+0000
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

![](1.gif)

## Recap: What is a Service

Remember from the [first part]({{< relref "post/ros-part-1" >}}) of this series, services are a node that allows you to give and input to get the desired output. If we want to add values, we can make a service that takes two numbers, sums them, and responds with the sum. Remember, they don’t publish continuously. They only respond when the service is called.

All good? Time to serve some nodes!

---

### Format of this and future articles:

It is important to understand how this article will go. We will go through an example together. I believe that knowledge can only be attained through practical examples.

---

## Task 1

Take in 2 numbers and respond with the product of those numbers.

### Process

A node will programmatically call the service and we must do the following:
 * Get inputs
 * Compute product
 * Respond via topic the product

## Programming
### Making a ROS Package

First, we have to make a ROS package which will be our node. That node will contain the code to make a topic and publish the message. There is a built-in command to do this using catkin.

First, though, navigate to the workspace:

```bash
cd catkin_ws/src
```

**NOTE**: You MUST be in `src`.

Then run this:

```bash
catkin_create_pkg basic_ros_service std_msgs rospy roscpp --pkg_version 0.0.1 --description "Basic ROS Service" --license MIT --author "Krishna" --maintainer "Krishna" --rosdistro "noetic"
```

Yikes! That is a lot so let me break it down.
 * First, we have `catkin_create_pkg` which is the base command.
 * Next comes the name of the package, in this case `basic_ros_service`. This can be anything.
 * Then, we add all the dependencies of the package. Here, we have 3: `std_msgs` `rospy` `roscpp`
   * `std_msgs` is a ROS package that contains a bunch of data types like `Float64`, `String`, `Int64`, and more!
   * `rospy` is a library that contains methods for using ROS with Python. This is what we will use today.
   * `roscpp` is a library that contains methods for using ROS with C++. We will NOT use this today.
   * `rosdistro` states the distribution of ROS we are using. In this case, it is ROS *“noetic”*. (This is optional as long as you have sourced your ROS environment)
   * **Note**: Services have additional requirements that we will manually add later!


Everything that is mandatory to install has been done. Now on to the optional stuff.
 * These parts are really self-explanatory. It includes the license, description, author, maintainer, and version.
 * You don’t need this but it is a good practice to keep.

Now, list the files in the directory. You will see this:

```plaintext
basic_ros_service  CMakeLists.txt
```

Go to `basic_ros_service` and list again.

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


That bottom part with the `buildtool_depend` and `build_depend` statements define the important libraries.

First, we must make a folder in our package called `srv/`. This will hold our custom service. In it make a file called `Product.srv`. Put this in it:

```plaintext
float64 a
float64 b
---
float64 product
```

The first 2 float statements are the input. Then we put `---` to show ROS that we are now defining the output. Lastly, we have the output. This allows us to define custom services with custom inputs and outputs.

Navigate to the src within that. Now you should be in `USERNAME/catkin_ws/src/PACKAGE_NAME/src`.

Here is where we put all of our code! Make a file called `ProductServer.py`. Again, names aren’t super important.

Here is your file structure right now:

```plaintext
catkin_ws
  - build
  - devel
  - src
    - basic_ros_service
      - CMakeLists.txt
      - package.xml
      - include {This directory contains nothing}
      - CMakeLists.txt
      - src
        - ProductServer.py
      - srv
        - Product.srv
```

Now let’s start on the code itself. First, you have to import the `rospy` and `std_msgs`.

```python3
#!/usr/bin/env python3
from __future__ import print_function
from basic_ros_service.srv import Product, ProductResponse # Import the compiled version of our service. There aren't files called ProductResponse but ROS will understand that. 
import rospy
import math
```

Now we make the function that serves the clients.

```python3
def product_func(a, b): # Actual multiplication function
    return a*b
def compute(req): # Find the product from the inputs req.a and req.b and return the product.
    product = product_func(req.a, req.b)
    print("Returning [%s]" % (product))
    return ProductResponse(product)
def product_server(): # Start the node and server.
    rospy.init_node('basic_ros_service')
    s = rospy.Service('basic_ros_service', Product, compute)
    print("Ready to compute.")
    rospy.spin()
```

And then we can run it with this:

```python3
if __name__ == "__main__":
    product_server()
```

**IMPORTANT**: You much make sure that your Python file is executable. sudo chmod +x ProductServer.py

Go the `CMakeLists.txt` and add our python script to the file. This will make sure that catkin will compile it.

```plaintext
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  src/ProductServer.py
)
```

Then, we need to make our SRV file that also needs to be compiled. Add the following lines to your `CMakeLists.txt`:

```plaintext
## Generate services in the 'srv' folder
 add_service_files(
   FILES
   Product.srv
)## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )
```

Also, add a dependency to the `find_package` section:

```plaintext
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)
```

We also need to add this dependency to our package.xml. Add the following lines next to all your other depend statements:

```xml
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
```

Now go back to `catkin_ws`. Run `catkin_make`. Source the directory `source devel/setup.bash`.

If it compiles, the hardest part is over!


### Running it

Start roscore: `roscore`. Now, start serving: `rosrun basic_ros_service ProductServer.py`.

Check the node using `rosnode list`. We will see our `basic_ros_service` service.

Check the service using `rosservice list`. We will see our `basic_ros_service` service and some logger services. Let us try calling our service using CLI.

```bash
rosservice call /basic_ros_service "a: 2.0
b: 2.0"
```

*Hint*: If you forget the format of the command hit TAB twice and it will auto-populate the command.

We will see our result of 4.0.

### Receive Data Programmatically

Import libraries:

```python3
#!/usr/bin/env python3from __future__ import print_functionimport sys
import rospy
from basic_ros_service.srv import *
```

Define client function and make the appropriate call:

```python3
def product_client(a, b):
    rospy.wait_for_service('basic_ros_service') # Find our service
    try:
        solver = rospy.ServiceProxy('basic_ros_service', Product) # Connect to the service
        resp1 = solver(a, b) # Make a request
        return resp1.product # Return the result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
```

Make the usage helper in case we forget how to use it:

```python3
def usage():
    return "%s [a b]"%sys.argv[0]
```

Run the functions from the CLI arguments:

```python3
if __name__ == "__main__":
    if len(sys.argv) == 3:
        a = float(sys.argv[1])
        b = float(sys.argv[2])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting %s*%s"%(a, b))
    print("%s * %s = %s"%(a, b, product_client(a, b)))
```

Again, remember to change the permissions of the file. And run it the same way as the publisher.

To run it, do `rosrun basic_ros_service ProductClient.py 2 2`.

## Conclusion

In this article, you learned how to make a Service Server and Client. You also learned how to make custom messages in the form of service. Lastly, you dived a little deeper into the `CMakeLists` and the `message_generation` library. If you encounter any errors or have any questions, leave a comment. I will try my best to help you. I hope you enjoyed it. If there is a particular topic in ROS that you want me to cover, let me know in the comments as well!



