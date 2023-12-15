# INSTALLATION 
## What is ROS2 and why shoul I use it? 
Between a Middleware and a Framework whcih provides ways to separate code along with communication tools. Multiple chunks of code communicate using ROS2 communications. IT provides tools and libraries (e.g., path planning) without reinventing the wheel. ROS is language agnostic (MATLAB, Python, C++, etc., can be used to write code). ROS2 is open source with an active community. 

## Which distribution?
Check the [ROS website](https://docs.ros.org) to consult all the distributions. In green there are the one that are still supported. Example: use ROS2 ```humble``` if you have Ubuntu 22.04 and ROS2 ```foxy``` if you use Ubuntu 20.04.

## Installation 
Follow the instuction in the dedicated section of the desired distribution. Also installe argcomplete for autocompletion with the command
```
nikolas@nikolas-PC:~$ pip3 install argcomplete
```
Reboot the computer/virtual machine.

## Usage
Every time a new terminal is opened, remember to source the ```.bash``` to enable ROS in that terminal. Example 
```
nikolas@nikolas-PC:~$ source /opt/ros/ROS_DISTRIBUTION_NAME/setup.bash
```
**NOTE**: it is possible to add it to ```~/.bashrc``` to make it sourced automatically. 

## Install the build tool 
In ROS2 the build tool used is ```colcon```. To install it
```
nikolas@nikolas-PC:~$ sudo apt install python3-colcon-common-extensions
```
Then it is possible to enable the autocompletion feature ein every terminal in which is required
```
nikolas@nikolas-PC:~$ source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
```
**NOTE**: it is possible to add it to ```~/.bashrc``` to make it sourced automatically. 


# ROS2 WORKSPACE 
## Workspace creation
The workspace is the place in which the code for the ROS2 application is located. IT is also the place where the code is compiled. To create it 
```
nikolas@nikolas-PC:~$ mkdir ros2_ws
nikolas@nikolas-PC:~$ cd ros2_ws
```
Now create the folder which contains the code and packages that we are going to create
```
nikolas@nikolas-PC:~/ros2_ws$ mkdir src

```
## Workspace build
Build the workspace using ```colcon build``` command
```
nikolas@nikolas-PC:~/ros2_ws$ colcon build 
```

This creates the folders ```build```, ```install``` and ```log``` inside the workspace. 
## Workspace enable
The ```install``` folder contains a ```setup.bash``` and a ```local_setup.bash``` scripts. The second sources only the workspace, while the first one sources the workspace and the global ros installation. 

**NOTE**:  It is important to source one of the two in every terminal which requires functionalities (e.g., packages) implemented in the workspace. 

# ROS2 PACKAGES
## Package creation
Allows to separate code into reusable blocks (e.g., camera pkg, motion planning pkg, and hardware control pkg). To create a package in the workspace, first navigate in the ```src``` folder of the workspace, then use ```ros pkg create ``` command 
```
nikolas@nikolas-PC:~/ros2_ws/src$ ros2 pkg create PACKAGE_NAME ARGUMENTS DEPENDENCIES
```
Examples of argument: 

- if I want a python package, then use ```--build-type ament_python```
- if I want a C++ package, then use ```--build-type ament_cmake```
  
Example of dependency: 
- if I want a python package, I need ```rclpy```, therefore ```--dependencies rclpy```
- if I want a C++ package, I need ```rclcpp```, therefore ```--dependencies rclcpp```

## Package structure 
Once created, the ```src``` will contain a folder called ```package_name```, structured more or less as follows (depends on the value of the ```--build-type``` argument)

Python package structure 
```
src
|
|- py_package_name
|  |- py_package name   // Code goes in this folder
|  |- resource
|  |- test
|  |- package.xml   // Contains package info (e.g., author, dependencies, etc.)
|  |- setup.cfg     // Useful to install nodes
|  |- setup.py      // USeful to install nodes 
```

C++ package structure 
```
src
|
|- cpp_package_name
|  |- src           //cpp files in this folder
|  |- include       // Headers in this folder
|  |- package.xml   // Contains package info (e.g., author, dependencies, etc.)
|  |- CMakeLists.txt     
```

## Build the package
To build the package, move in the main workspace directory and use ```colcon build ``` command, followed by the name of the packages 
```
nikolas@nikolas-PC:~/ros2_ws$ colcon build --packages-select PACKAGE_NAME 
```
***NOTE:*** it is good practice to source the workspace after creating a new package 

# ROS2 NODES
Subprograms in the application. Each node is responsible for only one thing (e.g., image processing node, camera driver node, etc.). Nodes are combined into a graph and each node can communicate with any other node (even if is in another package) using ROS2 communicaton tools (topics, services and parameters). Each node can be executed individually. 

Benefits:
- Reduce code complexity
- Provide great fault tolerance
- Nodes can be written using different programming languages

**NOTE:** it is fundamental that each node has a unique name

## How to create a Python node?
There are some steps 
1. CREATE: Move into ```ros2_ws/src/py_package_name/py_package_name``` and create the ```.py``` file which contain the source code of the node. See the created files in the package ```my_py_pkg``` to see how to structure the code. **NOTE**: The name of the node is NOT the name of the file. 
2. INSTALL: open the ```setup.py``` file in the package and modify the ```entry_points``` field, adding in the ```console_scripts``` list the string 
    ```
   "EXECUTABLE_NAME: PACKAGE_NAME.NODE_FILE_NAME:FUNCTION_TO_CALL"
    ```
    Example: 
     ```
    "py_node = my_py_pkg.my_first_node:main"
    ```
3. BUILD PACKAGE: move in the main workspace directory and use the ```colcon_build``` command, specifying the desired package. The node will be installed in ``` ros2_ws/install/PACKAGE_NAME/lib/PACKAGE_NAME/ ```.

***NOTE:*** it is good practice to source the workspace after creating a new node.
    
## How to start a ROS2 node?
To start the node, use the command 
```
nikolas@nikolas-PC:~/ros2_ws$ ros2 run PACKAGE_NAME EXECUTABLE_NAME
```

## How to create a C++ node?
There are some steps 
1. CREATE: Move into ```ros2_ws/src/cpp_package_name/src``` and create the ```.cpp``` file which contain the source code of the node. See the created files in the package ```my_cpp_pkg``` to see how to structure the code. 
   
    ***NOTE:*** Remember to modify ```c_cpp_properties.json``` in VSCode in order to solve the import error. In particular add the path ``` /opt/ros/DISTRIBUTION_NAME/include``` in the ```includePath``` list.
2. COMPILE AND INSTALL: open ```CMAkeLists.txt``` and specify the name of the executable and the associated source file adding the line 
   
    ```
    add_executable(EXECUTABLE_NAME PATH/TO/.CPP)
    ```
    example
    ```
    add_executable(cpp_node src/my_first_node.cpp)
    ```
    Then link the dependencies using
    ```
    ament_target_dependencies(EXECUTABLE_NAME DEPENDENCY)
    ```
    example
    ```
    ament_target_dependencies(cpp_node rclcpp)
    ```
    Finally, add the following to specify the destination
    ```
    install(TARGETS
        EXECUTABLE_NAME
        DESTINATION lib/${PROJECT_NAME}
    )
    ``` 
3.  BUILD PACKAGE: move in the main workspace directory and use the ```colcon_build``` command, specifying the desired package. The node will be installed in ``` ros2_ws/install/PACKAGE_NAME/lib/PACKAGE_NAME/ ```.

***NOTE:*** it is good practice to source the workspace after creating a new node.

# SOME ROS2 TOOLS
It is possible to check the list of active nodes using ```ros2 node list```. To have info about a particular node called ```NODE_NAME```, use the command ```ros2 node info NODE_NAME```.
## Change node name at runtime
It is not possible to have two active nodes with the same name: no error will be raised, only a warning. However, communication will have problems. What if there is the need to launch the same node multiple times (e.g., 5 temperature sensors)? Use the ```--remap``` (or ```-r```) argument:
```
ros2 run PACKAGE_NAME EXECUTABLE --ros-args --remap __node:=NEW_NODE_NAME
```

## Using RQT
There exists a collection of plugins called ```rqt```. One ofthe most important plugins is the ```rqt_graph```, which shows the active nodes in a gui, along the way in which nodes are connected each other.

# ROS2 TOPICS
## What is a topic?
A topic is a named bus over which nodes exchange messages. The data stream is unidirectional: a **publisher** sends messages on the topic, while a **subscriber** reads messages from the topic. A topic is **anonymous**, i.e., publishers are not aware of subscribers and vice-versa. Each topic is characterised by a **message type**. A node can be written directly inside a node using any ROS supported language (C++, Python, etc.). A node can have many publishers/subscribers for many different topics. 

## Create a Python Publisher
To create a python publisher, a ```publisher``` object, which can be created with the ```Node``` class method ```create_publisher()``` must be istantiated. To publish a message, the method ```publish()``` is used. If a message must be published periodically, then a ```timer``` is created in order to call a desired function (callback) with a fixed frequency. Example of publisher node

```
from example_interfaces.msg import String 

...

class RobotNewsStationNode(Node):
    def __init__(self):
        super().__init__("robot_news_station")

        self.robot_name = "C3PO"
        self.publisher = self.create_publisher(msg_type=String, # msg type
                                               topic="robot_news", # Topic name
                                               qos_profile=10) # Queue size

        self.timer = self.create_timer(0.5, callback=self.publish_news)
        self.get_logger().info("Robot News Station has been started")

    def publish_news(self):
        msg = String()
        msg.data = "Hi, this is " + str(self.robot_name)."
        self.publisher.publish(msg)
```

## Create a Python subscriber
To create a ```subscriber``` object, the method ```create_subscription()``` can be used. Such a method wants 4 parameters: ```msg_type``` (message type), ```topic``` (topic name), ```callback```(function called each time a message is received) and ```qos_profile``` (queue size). An example of node

```
from example_interfaces.msg import String 

...

class SmartphoneNode(Node):
    def __init__(self):
        super().__init__("smartphone")
        self.subscriber = self.create_subscription(msg_type=String, topic="robot_news", callback=self.callback_robot_news, qos_profile=10)

        self.get_logger().info("Smartphone has been started.")

    def callback_robot_news(self, msg): 
        self.get_logger().info(msg.data)
```

## Command line tools for topics
The base command is ```ros2 topic COMMAND```, the value of ```COMMAND``` can be
  
  - ```list```: list the active topics
  - ```info /topic_name```: gives info about a specific topic
  - ```echo /topic_name```: print the messages of the topic on the terminal
  - ```hz /topic_name```: print the average frequency with which messages are published on a specific topic
  - ```bw /topic_name```: show the bandwidth occupied by a topic
  - ```pub -r FREQ /topic_name MSG_TYPE "{data: "example_mesage"}"``` publish a message over a topic with a certain frequency

## Remap a Topic at Runtime
It is possible to change the topic name, originally defined in the code when starting the publisher node. Example 
```
ros2 run my_py_pkg robot_news_station --ros-args -r ORIGINAL_TOPIC_NAME:=NEW_TOPIC_NAME
```
**NOTE**: rememeber to remap the topic both in publisher and subscriber

# ROS2 SERVICES
A ROS2 Service is a client/server system, which can be synchronous or asynchronous. The server receives a _request_ from the client, checks if it is compliant and then sends back a _response_. Requests and Replies are ROS2 messages. For each service, there is ne message type for the request and one for the reply. Any ROS2 service can be written directly inside ROS2 nodes.  

A service is defined by a name and a type. As example, the service type which sums two integers can be seen using the command
```
ros2 interface show example_interfaces/srv/AddTwoInts
``` 
## Service server implementation in Python
A service can be created using the ```Node``` class method ```create_service()``` which wants 3 paramaters: ```srv_type```(service type), ```srv_name```(service name), and ```callaback``` (callback function). Here there is part of the implementation of the service which sums two integers 
```
...
from example_interfaces.srv import AddTwoInts

class AddTwoIntsServerNode(Node):
    def __init__(self):
        super().__init__("add_two_ints_server")
        self.server = self.create_service(
            srv_type=AddTwoInts, srv_name="add_two_ints",
            callback=self.callback_add_two_ints)
        self.get_logger().info("Add Two Ints cserver has been started")
    
    def callback_add_two_ints(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(str(request.a) + " + " + 
            str(request.b) + " = " + str(response.sum))
        return response
...
```
## Testing a server using Command Line
To test a service without writing the code for the client, it is possble to use 
```
ros2 service call /SERVICE_NAME SERVICE_TYPE REQUEST_DATA
```
Example
```
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 3, b: 6}"
```

## Service client implementation in Python
Below there is a commented example of an Client which requests the sum of two number 
```
...
from functools import partial # Allows to add more argument to a callback
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClientNode(Node):
    def __init__(self):
        super().__init__("add_two_ints_client")
        self.call_add_two_ints_server(6,7)
    
    def call_add_two_ints_server(self, a, b):
        # Create the client
        client = self.create_client(
            srv_type=AddTwoInts, srv_name="add_two_ints")
        
        # Wait for the service Server to be active
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for server Add Two Ints...")

        # Create the request object 
        request = AddTwoInts.Request()
        request.a = a
        request.b = b   

        # Call the server asynchronously
        future = client.call_async(request=request) 
        # Send the request in a non-blocking way and create the future object (i.e., the response)
        future.add_done_callback(callback=partial(self.callback_call_add_two_ints_server, a=a, b=b)) # Add a callback when the futue object is populated (i.e., a response is received)
    
    def callback_call_add_two_ints_server(self, future, a, b):
        # Process the request and does something with it
        try:
            response = future.result()
            self.get_logger().info(str(a) + " + " + 
                str(b) + " = " + str(response.sum))
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
...

```
## Command Line tools for Services 
There are fiew useful commands to debug ROS2 services. The base command is ```ros2 service COMMAND```, the value of ```COMMAND``` can be
  
  - ```list```: list the active services
  - ```call /service_name```: calls directly the service
  - ```find /service_type```: find the services of a specific type
  - ```type /service_name```: gives the service type

Moreover, ```rqt``` can be use to test services (see ```Plugins > Services``` in the GUI).  
## Remap Services at Runtime
To remap a service at runtime, use 
```
ros2 run PACKAGE_NAME NODE_NAME --ros-args -r OLD_SERVICE_NAME:=NEW_SERVICE_NAME
```

# ROS2 Interfaces
Messages and Services in ROS2 are also called **Interfaces**. Commonly used interfaces can be seen at ```https://github.com/ros2/common_interfaces```.
## Creating a custom Msg
It is convenient to create all the custom message definitions in a single package dedicated to the custom interfaces.
```
nikolas@nikolas:~/ros2_ws/src$ ros2 pkg create PACKAGE_NAME
```
**Note:** when creating packages for interfaces, it is possible to remove the ```include``` and ```src``` directories. Instead, create a ```msg``` folder. 
Then create the ```.msg``` file in the ```msg``` folder.

The package must be then configured.

1. In ```package.xml```, add the dependencies 
   ```
    <build_depend> rosidl_default_generators</build_depend>
    <exec_depend> rosidl_default_runtime</exec_depend>
    <member_of_group>rosidl_interface_packages </member_of_group>
   ``` 
2. In ```CMakeLists.txt```, add the dependency
   ```
   find_package(rosidl_default_generators REQUIRED)
   ```
    and add the command to generate to compile the message 
    ```
    rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/CustomMessageName.msg"
    )
    ```
Then, compile the package and source the workspace to be able to use the custam message.

## Create a Custom Srv
First, create the ```srv``` folder inside the interface package. This folder will contain all the ```.srv``` files. Perform the two steps indicated in the previous subsection, adding the command to generate the service. As an example 
```
rosidl_generate_interfaces(${PROJECT_NAME}
    ...
    "srv/ComputeRectangleArea.srv"
)
```

# Change Node settings at runtime with ROS2 Parameters
It is convenient to pass arguments to ROS2 node, in order to avoid hardcoding settings in the node. Parameters must be declared in the node, then when starting the node, a value to is assigned to those paramters, without the need to compile the node again.
To declare a parameter in a node, in the constructo call the ```Node``` method ```declare_parameter()```, which wants some paramters: ```name``` (name of the parameter), ```value``` (default value of the parameter). Example:
```
class HWStatusPublisherNode(Node):
    def __init__(self):
        super().__init__("hardware_status_publisher")

        # Define the parameters
        self.declare_parameter(name="motors_ready", value=True)
        ...
```
**NOTE:** the type of the parameter is set dynamically depending on the value assigned!
To set the parameter, 
```
ros2 run PACKAGE_NAME NODE_NAME --ros-args -p PARAM1_NAME:=value1 -p PARAM2_NAME:=value2
```
To access the parameter velue in the code, use the ```Node``` method ```get_parameter()``` which takes as argument the ```name``` of the desired parameter.

# ROS2 Launch Files 
Launch files allows to run several nodes without executing the ```ros2 run``` command several times. Launch files are like scripts which allow to start nodes simultaneously. 
## Creating lunch file
First, create a new package for launch files. From the package, remove ```src``` and ```include``` folders and create ```launch``` folder. In ```CMAkeLists.txt```, provide the instructions to install the launch folder, adding
```
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
```
In ```package.xml```, all the packages of the nodes launched by the launch files must be included as dependencies, e.g.,
```
<exec_depend> my_py_pkg </exec_depend>
<exec_depend> my_cpp_pkg </exec_depend>
```
Create the launch file in the ```launch``` folder. Launch files are Python files, so it will be named like ```app_name.launch.py```, structured as follows
```
from launch import LaunchDescription

def generate_launch_description():
    ld = LaunchDescription()
    ...
    return ld
```

A full example:
```
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    robot_news_station_node = Node(
        package="my_py_pkg",
        executable="robot_news_station", # The one in setup.py
        name="my_news_station", # Change default node name
        remappings=[
            ("robot_news", "new_robot_news") # Topic name remap
        ],
        parameters=[    # Assign parameters value
            {"robot_name": "CIRO"},
            {"publish_period": 2}
        ]
    )

    smartphone_node = Node(
        package = "my_py_pkg",
        executable = "smartphone", 
        name="galaxy_note",
        remappings=[
            ("robot_news", "new_robot_news") 
        ]
        
    )


    ld.add_action(robot_news_station_node)
    ld.add_action(smartphone_node)

    return ld
```
Since launch files are python files, it is possble to define the nodes start logic freely and easily. 
