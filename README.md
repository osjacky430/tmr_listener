# TM Robot Listener

A package that handles TM robot listen node, this package takes care of the TCP connection and message generation/parsing. The application is left for the end user to implement, making it simple, flexible and robust.

## Table of Contents

- [About the Project](#about-the-project)
  - [Built With](#built-with)
- [Getting Started](#getting-started)
  - [Prerequisite](#Prerequisite)
  - [Instal and run](#Instal-and-run)
- [Creating your own listener handle](#Creating-your-own-listener-handle)
- [Generate tm external script language](#Generate-tm-external-script-language)
- [Verify your handler works](#Verifiy-your-handler-works)
- [Using listen service](#Using-Listen-Service)
- [TODO](#TODO)
- [Contact](#contact)
- [Reference](#Reference)
- [Notes](#Notes)

## About the Project

TM robot provides listen node in TMFlow where user can control the robot arm via sending TM external script when entering the node. This package provides user not only the connection between TM robot, but also message generation/parsing. By providing your own handler, this package will take care of the rest of the stuffs,.

### Built with

- [ROS Kinetic Kame](http://wiki.ros.org/kinetic) under [Ubuntu 16.04.6 LTS (Xenial Xerus)](http://releases.ubuntu.com/16.04/)
- [ROS Melodic Morenia](http://wiki.ros.org/melodic) under [Ubuntu 18.04.5 LTS (Bionic Beaver)](https://releases.ubuntu.com/18.04.5/)

## Getting Started

These instructions will get you a copy of the package up and running on your machine.

### Prerequisite

- Boost: 1.58 at least, this is the default version shipped with ROS Kinetic

### Install and run

1. clone the repo to desire directory:

```sh
git clone https://git-codecommit.us-east-2.amazonaws.com/v1/repos/tm_robot_listener
```

2. build the package:

```sh
catkin build tm_robot_listener
```

3. run tm_robot_listener:

```sh
roslaunch tm_robot_listener tmr_listener.launch ip:=<your ip>
```

### Creating your own listener handle

This package requires user to implement their own handlers. This section will give you a basic idea of how to create your own listener handle, including brief introduction to the interface.

First and foremost, this package requires c++14, simply add the line(s) in `CMakeLists.txt`:

```cmake
# add this...
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# OR this (not recommended, ROS tutorial is not a good reference for writing CMakelist)
# In general, prefer to avoid the directory property commands, see reference (4)
add_compile_options(-std=c++14)

# and remember to find these packages
find_package(catkin REQUIRED COMPONENTS roscpp pluginlib tm_robot_listener)
```

To create your own listener handle, inherit `tm_robot_listener::ListenerHandle` in `tmr_listener_handle.hpp`:

```cpp

// in MyTMListenerHandle.hpp:
class MyTMListenerHandle final : tm_robot_listener::ListenerHandle {  // final keyword for devirtualization, see #3 in reference section
  // the rest of the implementation, will be explained in the ensuing section
};

// in MyTMListenerHandle.cpp:
PLUGINLIB_EXPORT_CLASS(MyTMListenerHandleNamespace::MyTMListenerHandle, tm_robot_listener::ListenerHandle)

```

For the rest of the setup, [see pluginlib tutorial (pretty outdated IMO)](http://wiki.ros.org/pluginlib/Tutorials/Writing%20and%20Using%20a%20Simple%20Plugin). `tm_robot_listener::ListenerHandle` provides several functions that can/must be overriden:

#### 1. motion_function::BaseHeaderProductPtr generate_cmd (MessageStatus const t_prev_response)

This function is the only way for the handler to talk to the TM robot, and therefore it must be overriden by the user. `t_prev_response` indicates whether TM robot responded to the message sent previously. Most of the time, we would like to generate command only after TM robot responded to our previous message. The responded message will be passed to the handler via [`response_msg`](<#3.-response_msg-(...)>).

```cpp
struct YourHandler final : public tm_robot_listener::ListenerHandle {
  protected:
    motion_function::BaseHeaderProductPtr generate_cmd(MessageStatus const t_prev_response) override {
      if (t_prev_response == MessageStatus::Responded) {
        // generate command
      }

      return empty_command_list();  // not responded yet, send empty message
    }
};

```

#### 2. tm_robot_listener::Decision start_task (std::vector\<std::string> const& t_data)

`start_task` takes data sent from TM robot on entering the listen node, and checks whether the listen node entered is the one it wants to handle. The messages passed are user-defined (see [tm expression editor and listen node](#Reference)), meaning there are various ways to do so. However, bear in mind that current tm_robot_listener only choose **one handler** when listen node is entered, the order of the plugin is decided by the ros param `listener_handles`:

```cpp
// The simplest way of implementation
struct YourHandler final : public tm_robot_listener::ListenerHandle {
  protected:
    tm_robot_listener::Decision start_task(std::vector<std::string> const& t_data) override {
      auto const start_handle = t_data[0] == "Listen1";
      if (start_handle) {
        // do some initialization
        return tm_robot_listener::Decision::Accept;  // start handling if the message sent is "Listen1"
      }

      return tm_robot_listener::Decision::Ignore;
    }
};
```

#### 3. response_msg (...)

The overload set `response_msg` allows user to response to certain message from header, override the header that you need, the rest of the header will be ignored.

```cpp
// this example overrides TMSCTResponse and the one with no argument
struct YourHandler final : public tm_robot_listener::ListenerHandle {
  private:
    motion_function::BaseHeaderProductPtr next_cmd_;
    int msg_count_ = 0;
  protected:
    // Exit script if error happened
    void response_msg(tm_robot_listener::TMSCTResponse const& t_resp) override {
      if (not t_resp.script_result_) {
        this->next_cmd_ = TMSCT << ID{"1"} << ScriptExit();
      }
    }

    // Count how many messages TM responded, response_msg with no argument will be called everytime TM robot respond,
    // no matter what header it is
    void response_msg() override {
      ++this->msg_count_;
    }
};

```

### Generate tm external script language

TM external message is complicated for end user to generate, and can easily screw things up. Therefore, tm_robot_listener provides some handy ways to generate the message. `tm_robot_listener` creates two global `Header` instances, i.e., `TMSCT`, and `TMSTA`. Also, for all motion functions, tm_robot_listener creates a `FunctionSet` instance for each of them. See reference manual for all motion functions in listen mode. By doing so, we can avoid syntax error or typo, since the interface acts like you are writing c++ code, typo simply indicates compile error.

With tm_robot_listener, user can generate listen node command relatively easy, by fluent interface:

```cpp
using namespace tm_robot_listener::motion_function;

auto const cmd = TMSCT << ID{"Whatever_you_like"} << QueueTag(1, 1) << End();
// this generates "TMSCT,XX,Whatever_you_like,QueueTag(1,1),*XX", XX means handled by tm_robot_listener
```

Instead of typing: `"$TMSCT,XX,Whatever_you_like,QueueTag(1,1),XX\r\n"`. A typo, e.g., `TMSCT` to `TMSTA`, or `QueueTag(1,1)` to `QueuTag(1,1)`, or wrong length, or checksum error, you name it (while reading this line, you might not notice that a `*` is missing in the checksum part, gotcha!), may ruin one's day.

The generation of external script message is composed of three parts: `Header`, `Command`, and `End` signal. See reference manual for all the `Header` and its corresponding `Command`. For the last part, end signal, `End()` and `ScriptExit()` is used, command cannot be appended after `End()` and `ScriptExit()`:

```cpp
TMSCT << ID{"1"} << QueueTag(1, 1) << End() << QueueTag(1, 1); // compile error (no known conversion)
```

Commands without `End()` or `ScriptExit()` will also result in compile error:

```cpp
TMSCT << ID{"1"} << QueueTag(1, 1); // well, this line alone won't generate compile error. However, this cannot be used as return value for generate_command()
```

Also, TM listen node commands are tagged, meaning that it is impossible to misuse:

```cpp
TMSTA << QueueTag(1, 1) << End(); // compile error: Command is not usable for this header
TMSTA << QueueTagDone(1) << ScriptExit(); // compile error: Script exit can only be used in TMSCT
```

For more detail, see `src/test/CMakeLists.txt`. It contains a couple of examples of correct and wrong syntax.

### Verify your handler works

To verify whether your handler works or not, first, make sure tm_robot_listener is aware of your plugin:

```sh
rospack plugins --attrib=plugin tm_robot_listener # this command should list your plugin if you configure it correctly
```

Secondly, add the handler to the tmr_listener.launch: (@todo: think of a better way to add plugin):

```xml
<launch>
    <arg name="ip"/>
    <node pkg="tm_robot_listener" type="tm_robot_listener_node" name="tm_robot_listener" output="screen" args="--ip $(arg ip)">
        <rosparam param="listener_handles">["tm_error_handler::TMErrorHandler", ...]</rosparam>
    </node>
</launch>
```

Lastly, to make sure your handler generate the message at the right time, launch tmr_listener using local ip: (@todo: need more convinient way!)

```sh
roslaunch tm_robot_listener tmr_listener.launch ip:=127.0.0.1
```

The command above create a socket at `127.0.0.1:5890`, where `5890` is the port of the TM listen node. Next, we are going to create a fake endpoint to simulate TM robot, open another window and enter the following command:

```
nc  -lC 5890  # l: listen, C: carriage return and line feed at the end of the messages
```

This will establish the connection between netcat and tmr_listener, the only thing left is to simulate TM robot and generate TM messages. (@todo: I will improve this in the future)

### Using Listen Service

Under construction...

### Unit Test

To run unit test, copy paste the following line to the terminal:

```sh
catkin build -v tm_robot_listener --catkin-make-args CTEST_OUTPUT_ON_FAILURE=1 test
```

Notice the option `-v`, **this is needed** since tm_robot_listener will determine whether the test is success by verbose output (for normal unit test, this is not needed, but because we are testing code that can't even compile, the only thing we can depend on is the result output by the compiler).

### TODO

- Better ROS interface
- Inject tm logic to boost asio instead of heavy coupling
- Type conversion operator, TM has some "unique" type conversion rules, which is totally BS to me
- consider function accepting types that can be implicitly converted to the desired type
- Implement some services
- TM functions, and project variables
- More Unit test
  - Connection

### Contact

Jacky Tseng (master branch) - jacky.tseng@gyro.com.tw

### Reference

1. [TM expression editor and listen node reference manual](https://assets.omron.com/m/1d1932319ce3e3b3/original/TM-Expression-Editor-Manual.pdf)
2. [pluginlib tutorial](http://wiki.ros.org/pluginlib/Tutorials/Writing%20and%20Using%20a%20Simple%20Plugin)
3. [the performance benefits of final classes](https://devblogs.microsoft.com/cppblog/the-performance-benefits-of-final-classes/)
4. [Professional CMake - A practical guide, P122. ~ P126.](https://crascit.com/professional-cmake/)
5. [boost asio for tcp socket programming](https://www.boost.org/doc/libs/1_58_0/doc/html/boost_asio.html)

### Notes

- [Memory leak issue due to plugin lib](https://github.com/ros/class_loader/issues/131)
