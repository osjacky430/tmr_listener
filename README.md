# TM Robot Listener <!-- omit in toc -->

[![ubuntu](https://github.com/osjacky430/tmr_listener/actions/workflows/ubuntu.yml/badge.svg)](https://github.com/osjacky430/tmr_listener/actions/workflows/ubuntu.yml) [![sanitizer](https://github.com/osjacky430/tmr_listener/actions/workflows/sanitizer.yml/badge.svg)](https://github.com/osjacky430/tmr_listener/actions/workflows/sanitizer.yml) [![windows](https://github.com/osjacky430/tmr_listener/actions/workflows/windows.yml/badge.svg)](https://github.com/osjacky430/tmr_listener/actions/workflows/windows.yml) [![Build Status](https://app.travis-ci.com/osjacky430/tmr_listener.svg?branch=master)](https://app.travis-ci.com/osjacky430/tmr_listener) [![codecov](https://codecov.io/gh/osjacky430/tmr_listener/branch/master/graph/badge.svg?token=WVAY02N0WD)](https://codecov.io/gh/osjacky430/tmr_listener) [![Codacy Badge](https://app.codacy.com/project/badge/Coverage/96a45c63f83d43eb8e5a178594b0d8f2)](https://www.codacy.com/gh/osjacky430/tmr_listener/dashboard?utm_source=github.com&utm_medium=referral&utm_content=osjacky430/tmr_listener&utm_campaign=Badge_Coverage) [![CodeFactor](https://www.codefactor.io/repository/github/osjacky430/tmr_listener/badge)](https://www.codefactor.io/repository/github/osjacky430/tmr_listener)

A package that handles TM robot listen node and TM Ethernet Slave functionality. This project strives to reduce the amount of knowledge needed in order to use listen node and ethernet slave by providing easy to use, hard to misuse interface, and still remains maximum flexibility at the same time. As the result, library users only need to create the plugins in order to use listen node. Meanwhile, ethernet slave functionality is reduced to single service, and two published topics.

## Table of Contents

  - [Table of Contents](#table-of-contents)
  - [About the Project](#about-the-project)
    - [Built with](#built-with)
  - [Getting Started](#getting-started)
    - [Prerequisite](#prerequisite)
    - [Install and run](#install-and-run)
  - [TM robot listener](#tm-robot-listener-1)
    - [Custom plugin manager](#custom-plugin-manager)
    - [Creating your own listener handle](#creating-your-own-listener-handle)
      - [1. tmr_listener::MessagePtr generate_cmd (tmr_listener::MessageStatus const t_prev_response)](#1-tmr_listenermessageptr-generate_cmd-tmr_listenermessagestatus-const-t_prev_response)
      - [2. tmr_listener:: Decision start_task (std::string const& t_data)](#2-tmr_listener-decision-start_task-stdstring-const-t_data)
      - [3. response_msg (...)](#3-response_msg-)
    - [Generate tm external script language](#generate-tm-external-script-language)
    - [Verify your handler works](#verify-your-handler-works)
    - [Using Listen Service](#using-listen-service)
  - [TM Robot Ethernet Slave](#tm-robot-ethernet-slave)
    - [Communication Mode](#communication-mode)
    - [Data table](#data-table)
    - [Read / Write Request](#read--write-request)
    - [Exported Data Table XML](#exported-data-table-xml)
  - [Unit Test](#unit-test)
  - [TODO](#todo)
  - [Contact](#contact)
  - [Reference](#reference)
  - [Notes](#notes)

## About the Project

TM robot provides listen node in TMFlow where user can control the robot arm via sending TM external script when the control flow enters it. This node provides better scalability due to the fact that one can't copy TMFlow project from one robot arm to the other without doing furthur adjustment.

However, the implementation of [the official ros package](https://github.com/TechmanRobotInc/tmr_ros1) has several drawbacks:

1.  End user can never know whether the control flow enters listen node or not without polling the service.

2.  Command can't be stacked for TMSCT since some of them are implemented as individual services, you would never want to stack command using service `send_script`, unless you are fine with this:

    ```cpp
    std::string const cmd = "float[ targetP1= {0,0,90,0,90,0}\r\n"  // wrong here, missing right bracket
                            "PTP(”JPP”,targetP,10,200,0,false)\r\n"   // wrong here, I accidentally enter the wrong name
                            "QueueTag(1)\r" // wrong here, no line feed
                            "foat[] targetP2 = { 0, 90, 0, 90, 0, 0 }\r\n"  // wrong here, float not foat
                            "PTP(”JPP”, targetP2, 10, 200, false)\r\n"  // wrong here, I accidentally remove 4-th parameter
                            "QueueTag(2)\n" // wrong here, no carriage return
    ```

3.  No Parameterized Objects, no Variable, those can only be sent via string, which is error-prone

4.  No compile time check, i.e. it is extremely easy for user to make such mistake:

    ```cpp
    std::string cmd = "PTP(\"JPP\",0,0,90,0,90,0,200,0,false)"; // wrong here, I accidentally remove 8-th parameter, 35
    // enjoy your happy debug time
    ```

5.  Script sending and response recieving are implemented separately, this means that if there are different nodes sending script command and subscribing to the response topic, user might get confused cause they will recieve responses that they didn't send, which makes it relatively difficult to debug (ID is of no help under such circumstances). The only way to mitigate the situation is to limit the use to single ros package, or to ensure only one node is communicating to TM listen node at a time.

To overcome the drawbacks mentioned above while offering more functionality, we need to implement it in a different way. See README.md under `tmr_listener/src` for more detail. The tutorial will guide you through this library, both listen node and ethernet service, and also showcase how the drawbacks are overcomed.

### Built with

-   [ROS Kinetic Kame](http://wiki.ros.org/kinetic) under [Ubuntu 16.04.6 LTS (Xenial Xerus)](http://releases.ubuntu.com/16.04/)
-   [ROS Melodic Morenia](http://wiki.ros.org/melodic) under [Ubuntu 18.04.5 LTS (Bionic Beaver)](https://releases.ubuntu.com/18.04.5/)
-   [ROS Noetic Ninjemys](http://wiki.ros.org/noetic) under [Windows 10](http://wiki.ros.org/Installation/Windows)

## Getting Started

These instructions will get you a copy of the package up and running on your machine.

### Prerequisite

-   Boost: 1.58 at least, this is the default version shipped with ROS Kinetic

### Install and run

1.  clone the repo to desire directory:

```sh
git clone https://git-codecommit.us-east-2.amazonaws.com/v1/repos/tmr_listener # for gyrobot developer
git clone https://github.com/osjacky430/tmr_listener # for github user
```

2.  build the package:

```sh
# use catkin build:
catkin build tmr_listener (Additional build options)
# or catkin_make: (which is the only way to build ROS1 package on windows)
catkin_make
```

Build options for `tmr_listener`:

-   `-DTMR_ENABLE_TESTING`: to enable unit test code building, set `-DTMR_ENABLE_TESTING=ON`, default to `OFF`
-   `-DENABLE_IPO`: to enable link-time optimization, a.k.a Inter-Procedural Optization, set `-DENABLE_IPO=ON`, default to `OFF`
-   `-DTMR_TMFLOW_VERSION`: to compile `tmr_listener` that support specific version of TMFlow, set `-DTMR_TMFLOW_VERSION=<TMFlow version>`, default to `1.82.0000`, i.e., the code is built as if explicitly pass `-DTMR_TMFLOW_VERSION=1.82.0000` to `catkin build`
-   `-DTM_ETHERNET_SLAVE_XML`: see section [Exported Data Table XML](#exported-data-table-xml) for more detail
-   `-DENABLE_CACHE`: enable [`ccache`](https://ccache.dev/) to speed up compilation, default to `ON`, but will only warn if `ccache` is not found

1.  run tmr_listener:

```sh
roslaunch tmr_listener tmr_listener.launch  # for listen node
roslaucnh tmr_listener tmr_eth_slave.launch # for ethernet slave
```

-   Required Arguments:
    -   ip: set this arg to your desire ip
-   Optional Arguments:
    -   mock_tmr (default "false"): this is equivalent to launching tmr_listener with `ip:=127.0.0.1`

## TM robot listener

In order to use `tmr_listener` , the running TMFlow project must contain the listen node, and can be reached by the control flow.

The implementation of `tmr_listener` is composed of three parts: (1) TCP/IP comm (via `boost::asio` ) (2) plugin manager (via `pluginlib` ), and (3) message generation (self implementation) and parsing (via `boost::Spirit` ).

Normally, one would only need to implement plugin for specific task (see [creating your own listener handle](#Creating-your-own-listener-handle)). `tmr_listener` also allows user to implement their own plugin manager for finer configuration (see next section).

### Custom plugin manager

Plugin manager is default implemented via pluginlib. However, one would like to skip the use of it (e.g. You only have one plugin, using pluginlib may be an overkill) and provide their own plugin management implementation. To do so, you need to inherit from `TMRPluginManagerBase` , the example below shows the default implementation of the TM Robot plugin manager:

```cpp
// In header file:

// Runtime library plugin manager
class RTLibPluginManager final : public TMRPluginManagerBase {
  ...
 public:
  // these two functions must be overriden
  TMTaskHandlerArray_t get_all_plugins() const noexcept override { return this->all_plugins_; }
  TMTaskHandler find_task_handler(std::string const &t_input) const noexcept override {
    auto const predicate = [&t_input](auto const &t_handler) {
      return t_handler->start_task_handling(t_input) == Decision::Accept;
    };

    auto const matched = std::find_if(this->all_plugins_.begin(), this->all_plugins_.end(), predicate);
    if (matched == this->all_plugins_.end()) {
      return this->default_handler_;
    }

    return *matched;
  }
};

```

Beware: `tmr_listener::TMRobotListener` will not handle any of the exception raised inside `find_task_handler(std::string const&) const`. Therefore, **it is forbidden to throw exception without catching it inside the function.** Exception will be thrown if detected nullptr returned.

In order to use your own plugin manager implementation, pass it to the constructor of `tmr_listener:: TMRobotListener` :

```cpp
#include "tmr_listener/tmr_listener.hpp"
... // other includes

int main(int argc, char** argv) {
  ... // ros related initialization

  tmr_listener::TMRobotListener listener_node{ip, boost::make_shared<RTLibPluginManager>()};
  listener_node.start();

  ... // the rest of the cleanup, if needed
}

```

### Creating your own listener handle

The default plugin manager used by the library loads plugins via DLL. Therefore, users are required to build their implementations into a shared object. This section will give you a basic idea of how to create your own listener handle, including brief introduction to the interface.

First and foremost, this package requires c++14, simply add the line(s) in `CMakeLists.txt` :

```cmake
# Three option to compile project with c++14
# (1) add this...
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# (2) OR this (not recommended, ROS tutorial is not a good reference for writing CMakelist)
# In general, prefer to avoid the directory property commands, see reference (4)
add_compile_options(-std=c++14)

# (3) or for more finer configuration
add_library(project_cpp_ver INTERFACE)
target_compile_options(project_cpp_ver INTERFACE cxx_std_14)
# then link this library to other target, just as you did normally

# and remember to find these packages
find_package(catkin REQUIRED COMPONENTS roscpp pluginlib tmr_listener)
```

To create your own listener handle, inherit `tmr_listener::ListenerHandle` in `tmr_listener_handle.hpp` :

```cpp

// in MyTMListenerHandle.hpp:
class MyTMListenerHandle final : tmr_listener::ListenerHandle {  // final keyword for devirtualization, see #3 in reference section
  // the rest of the implementation, will be explained in the ensuing section
};

// in MyTMListenerHandle.cpp:
PLUGINLIB_EXPORT_CLASS(MyTMListenerHandleNamespace::MyTMListenerHandle, tmr_listener::ListenerHandle)

```

For the rest of the setup, see the [pluginlib tutorial](http://wiki.ros.org/pluginlib/Tutorials/Writing%20and%20Using%20a%20Simple%20Plugin). One thing worth mentioning is that, in `package.xml`, one needs to set `<depend>tmr_listener</depend> `, not `<build_depend>tmr_listener</build_depend>`, which is not mentioned in the tutorial.

`tmr_listener::ListenerHandle` provides several functions that can/must be overriden:

#### 1. tmr_listener::MessagePtr generate_cmd (tmr_listener::MessageStatus const t_prev_response)

This function is the only way for the handler to talk to the TM robot, and therefore it must be overriden by the user. `t_prev_response` indicates whether TM robot responded to the message sent previously. Most of the time, we would like to generate command only after TM robot responded to our previous message.

```cpp
struct YourHandler final : public tmr_listener::ListenerHandle {
  protected:
    tmr_listener::MessagePtr generate_cmd(tmr_listener::MessageStatus const t_prev_response) override {
      if (t_prev_response == tmr_listener::MessageStatus::Responded) {
        // generate command
      }

      return empty_command_list();  // not responded yet, send empty message
    }
};

```

#### 2. tmr_listener::Decision start_task (std::string const& t_data)

`start_task` takes data sent from TM robot on entering the listen node, and checks whether the listen node entered is the one it wants to handle. The messages passed are user-defined (see [tm expression editor and listen node](#Reference)), meaning there are various ways to do so. However, bear in mind that current tmr_listener only choose **one handler** when listen node is entered, the order of the plugin is decided by the ros param `listener_handles` :

```cpp
// The simplest way of implementation
struct YourHandler final : public tmr_listener::ListenerHandle {
  protected:
    tmr_listener::Decision start_task(std::string const& t_data) override {
      auto const start_handle = t_data == "Listen1";
      if (start_handle) {
        // do some initialization
        return tmr_listener::Decision::Accept;  // start handling if the message sent is "Listen1"
      }

      return tmr_listener::Decision::Ignore;
    }
};
```

#### 3. response_msg (...)

The overload set `response_msg` allows user to respond to certain header packet, you only need to override those that you need, those that are not overriden will be ignored. Currently available overloads:

```cpp
void response_msg(tmr_listener::TMSCTResponse const&);            //  get called when user send correct TMSCT command
void response_msg(tmr_listener::TMSTAResponse::Subcmd00 const&);  //  get called when user send correct TMSTA,XX,00, ...
void response_msg(tmr_listener::TMSTAResponse::Subcmd01 const&);  //  get called when user send correct TMSTA,XX,01, ...
void response_msg(tmr_listener::TMSTAResponse::DataMsg const&);   //  get called when TM send TMSTA,XX,9X,...
void response_msg(tmr_listener::CPERRResponse const&);            //  get called when user send incorrect packet, or other error
void response_msg();                                              //  get called every time packet is received
```

The following example overrides TMSCTResponse and the one with no argument:

```cpp
struct YourHandler final : public tmr_listener::ListenerHandle {
  private:
    MessagePtr next_cmd_;
    int msg_count_ = 0;

    // Exit script if error happened
    void response_msg(tmr_listener::TMSCTResponse const& t_resp) override {
      if (not t_resp.script_result_) {
        this->next_cmd_ = TMSCT << ID{"1"} << ScriptExit();
      }
    }

    // Count how many messages TM responded, response_msg with no argument will be called everytime TM robot responded,
    // no matter what header it is
    void response_msg() override {
      ++this->msg_count_;
    }
};

```

Note: `response_msg(TMSTAResponse::DataMsg const&)` is a bit different from other overloads,**`tmr_listener` will call this function for each plugin** whenever TM robot sends Subcmd 90 ~ 99. For those that doesn't implement a plugin, but would also want to receive data from Subcmd 90 ~ 99, subscribe to topic `/tmr_listener/subcmd_90_99`

#### 4. void handle_disconnect()

This function should be easy to understand, user must make sure that all variables are set to reasonable states if disconnect happened, since your handler might be called again next time the project entered listen node:

```cpp
// This example analyzes the quality of vision job by executing it repeatedly at fixed pose.
// The listen node executes vision job named "some_vision", then exit script, and expects the
// result to be sent via TMSTA 90-99, e.g.
//
//      dummy_var = ListenSend("192.168.1.186", 91, GetString(Base["some_vision"].Value, 10)).
//
// The project flow would look like this (note: we don't need listen node in order to execute
// vision job, this is just an example):
//
//        ________                 _________          ______
//       |        |               |         |        |      |
//  ...  | Listen | ----------->  |   SET   | -----> | GOTO |
//       |________|     pass      |_________|        |______|
//           |         ________   (ListenSend)       (Listen)
//           |        |        |
//           -------> |  STOP  |
//             fail   |________|
//
struct YourHandler final : public tmr_listener::ListenerHandle {
  std::vector<std::array<float, 6>> vision_bases_;
  std::size_t recorded_count_ = 0;

  static constexpr std::size_t MAX_RECORDING = 150;

  tmr_listener::Decision start_task(std::string const& t_data) override {
    auto const start_handle = t_data == "Listen1";
    if (start_handle) {
      return tmr_listener::Decision::Accept;  // start handling if the message sent is "Listen1"
    }

    return tmr_listener::Decision::Ignore;
  }

  tmr_listener::MessagePtr generate_cmd(tmr_listener::ListenerHandle::MessageStatus t_prev_response) {
    using namespace tmr_listener;
    using namespace tmr_listener::motion_function;
    if (t_prev_response == ListenerHandle::MessageStatus::Responded) {
      auto ret_val = TMSCT << ID{"random"s};
      if (++this->recorded_count_ > MAX_RECORDING) {
        this->recorded_count_ = 0;
        return ret_val << ScriptExit::WithPriority{ScriptExit::Result::ScriptFail}; // fail on purpose, in order to stop the project
      }

      return ret_val << Vision_DoJob("some_vision"s) << ScriptExit{};
    }

    return dummy_command_list("waiting"s);
  }

  void response(TMSTAResponse::DataMsg const& t_msg) override {
    if (t_msg.cmd_ == 91) { // assuming 91 is only used in this situation
      this->vision_bases_.emplace_back(parse_as<float, 6>(t_msg.data_));
    }
  }

  void handle_disconnect() override {
    // if disconnected, reset these variables, we are not sure the robot will be at the same position next time
    this->recorded_count = 0;
    this->vision_bases.clear();
  }
};

```

### Generate tm external script language

TM external message is complicated for end user to use, not only because there are some undocumented or unclear behavior, but also because there are a lot of rules to keep in mind. As a result, it is extremely easy to screw things up. `tmr_listener` knows your pain, and it strives to simplify the process of message generation by making the code fail as early as possible.

The generation of external script message is composed of three parts: `Header` , `Command` , and `End` signal. See [reference manual](https://www.tm-robot.com/en/wpdmdownload/expression-editor-manual_sw1-82/) for all the `Header` and `Command` (functions) that can be used.

First of all, `tmr_listener` creates four global `Header` instances, i.e., `TMSCT` , `TMSTA` , `TMSVR` , and `CPERR`. These headers are used as the starting of the message generation / parsing. Secondly, for all functions (motion functions, math functions, etc.) and their corresponding overload functions, tmr_listener creates a `FunctionSet` instance for them. By doing so, we can avoid syntax error or typo, since the interface acts as if you were writing c++ code, typo simply indicates compile error.

With `tmr_listener`, command generation becomes pretty intuitive, by fluent interface:

```cpp
using namespace tmr_listener; // for TMSCT and ID, End
using namespace tmr_listener::motion_function; // for QueueTag

auto const cmd = TMSCT << ID{"Whatever_you_like"} << QueueTag(1, 1) << End();
// this generates "TMSCT,XX,Whatever_you_like,QueueTag(1,1),XX", XX means handled by tmr_listener
```

Instead of typing: `"$TMSCT, XX, Whatever_you_like, QueueTag(1, 1), XX\r\n"` . A typo, e.g., `TMSCT` to `TMSTA` , or `QueueTag(1,1)` to `QueuTag(1, 1)` , or wrong length, or checksum error, you name it (while reading this line, you might not notice that a `*` is missing in the checksum part, gotcha!), may ruin one's day.

Note: There is one more mistake in the example, that is, the name of the ID is invalid! In order to deal with this problem, it is recommended to use macro `TMR_ID` or UDL `_id` if ID is initialized with string literal, e.g, use `TMR_ID("SomeString")` or `"SomeString"_id` instead of `ID{"SomeString"}`, initialization via `TMR_ID` check validity of the string literal at compile time, i.e., `TMR_ID("Whatever_you_like")` will not compile, and `"SomeString"_id` will not compile if evaluated at compile time (e.g. `constexpr auto my_id = "SomeString"_id;`), and will throw if evaluated at runtime, whereas `ID{"Whatever_you_like"}` will not do any runtime check so that users don't need to pay for the overhead of it. **USER NEED TO MAKE SURE THE ID IS VALID ON THEIR OWN, OTHERWISE TM WILL REPLY WiTH CPERR 04**.

As for the last part, end signal, `End()` and `ScriptExit()` is used, command cannot be appended after `End()` and `ScriptExit()` :

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

For more detail, see `test/CMakeLists.txt` . It contains a couple of examples of correct and wrong syntax.

### Verify your handler works

To verify whether your handler works or not, first, make sure `tmr_listener` is aware of your plugin:

```sh
rospack plugins --attrib=plugin tmr_listener # this command should list your plugin if you configure it correctly
```

Secondly, make sure rosparam `/tmr_listener/listener_handles` contains the array of plugins you wanted to load:

```sh
rosparam get /tmr_listener/listener_handles # expected result example: ['tmr_listener_plugins::CoordinateManager']
```

To do so, one can either set `/tmr_listener/listener_handles` via `rosparam set /tmr_listener/listener_plugin "[...]"`, or create a launch file that load rosparam from yaml file, and include `tmr_listener.launch`:

```xml
<launch>
  <arg name="robot_ip" default="" doc="set this arg to your desire ip"/>

  <rosparam command="load" ns="tmr_listener" file="$(find tmr_listener_plugins)/param/plugin_to_load.yml"/>
  <include file="$(find tmr_listener)/launch/tmr_listener.launch">
    <arg name="ip" value="$(arg robot_ip)"/>
  </include>
</launch>
```

Lastly, to make sure your handler generate the message at the right time, launch tmr_listener using local ip: (@todo: need more convinient way!)

```sh
roslaunch tmr_listener tmr_listener.launch ip:=127.0.0.1

# or your own launch file that includes tmr_listener.launch (take the launch file above as an example)
roslaucnh tmr_listener_plugins tmr_listener_plugin.launch robot_ip:=127.0.0.1
```

The command above create a socket at `127.0.0.1:5890` , where `5890` is the port of the TM listen node. Next, we are going to create a fake endpoint to simulate TM robot, open another window and enter the following command:

    nc  -lC 5890  # l: listen, C: carriage return and line feed at the end of the messages

This will establish the connection between netcat and tmr_listener, the only thing left is to simulate TM robot and generate TM messages. (@todo: I will improve this in the future)

### Using Listen Service

Current `tmr_listener` implementation makes the listen service kind of redundant:

1.  For `TMSTA SubCmd 00`, `tmr_listener` can take on this responsibility, as the library behaves similarly to listen node once entered (one listen node to one listen node handler), this is relatively easy, and can be implemented readily once there is a need.
2.  For `TMSTA SubCmd 01`, this is totally unusable because user not aware of listen node handler doesn't have any single information about motion tag.
3.  Last but not least, `TMSTA SubCmd 90...99`, since the user is in full passive mode. Service is definitely not the ideal way of implementing it.

Due to the reasons above, I am not intended to implement any listen services now, feel free to give me ideas if you find services useful in your case.

## TM Robot Ethernet Slave

TM robot ethernet slave provides us a convinient way to read/write variables (global, predfined, and user defined variables). This package has full supports for read/write request and periodically updated data table. Users only need to make sure the item listed in data table is configured correctly and run the following command:

```sh
roslaunch tmr_listener tmr_eth_slave.launch # this will run tmr_eth_slave_node and tmr_eth_exported_dt_node
```

### Communication Mode

Currently, **only JSON mode is implemented**, therefore, to let the library function properlly, please make sure the communication mode is set to JSON. (@todo add support for String, and possibly, Binary)

### Data table

TM robot sends data table periodically after power cycling if it was previously set to Enable. Having no prior knowledge regarding the listed item in the data table, this package decides to provide two topics, `/tm_ethernet_slave/raw_data_table` and `/tm_ethernet_slave/parsed_data_table` , to receive the contents. Users can choose either to parse data received from the former with their favorite 3rd party library, e.g., [nlohmann-json](https://github.com/nlohmann/json):

```cpp
#include <nlohmann/json.hpp>
#include <std_msgs/String.h>

void raw_data_cb(std_msgs::String::ConstPtr& t_msg) {
  // possible content of t_msg->data: [{"Item":"Robot_Link","Value":0}, {"Item":"Ctrl_DO1","Value":0},{"Item":"g_ss","Value":["Hello","TM","Robot"]}]
  // which is an array, first element: {"Item":"Robot_Link","Value":0}
  auto const json_obj = nlohmann::json::parse(t_msg->data);

  // assuming the first element in the array is: {"Item": "Robot_Link","Value":0}
  auto const robot_link = json_obj[0]["Value"].get<int>();
}

```

, or the latter by some handy function this package offers:

```cpp
#include "tmr_listener/JsonDataArray.h"
#include "tmr_utility/tmr_parser.hpp"

void parsed_data_cb(tmr_listener::JsonDataArray::ConstPtr& t_msg) {
  // assuming data[0] contains Item: "rng", Value: 0
  auto const v = tmr_listener::parse_as<int>(t_msg->data[0].value_);  // the type of v is int

  // assuming data[1] contains Item: "Joint_Angle", Value: [89.4597,-35.00033,125.000435,-0.000126898289,90.0,0.0005951971]
  auto const array_v = tmr_listener::parse_as<double, 6>(t_msg->data[1].value_);  // the type of array_v is std::array<double, 6>
}
```

### Read / Write Request

This package provides service, `/tm_ethernet_slave/tmsvr_cmd` , to read/write variables. For read operation, the `EthernetSlaveCmdRequest::value_list` must be left empty, ~~since the read result is stored in it~~ (**this is absurdly wrong, I'm fooled by the `MReq& req` in roscpp service call**); as for write operation, `value_list.size() == item_list.size()` must hold, and the value of `item_list[i]` is `value_list[i]` . (@todo I forgot the reason to not use an array of msg with item/value string instead of two separate string array)

```cpp
// write operation
tmr_listener::EthernetSlaveCmd cmd;
cmd.request.id = "StartProject";
cmd.request.item_list.emplace_back("Stick_PlayPause");
cmd.request.value_list.emplace_back("true");

if (not ros::service::call("/tmr_eth_slave/tmsvr_cmd", cmd)) {
  ROS_ERROR_STREAM("service fail");
} else if (cmd.response.res != "00,OK") {
  ROS_ERROR_STREAM(cmd.response.res);
}

// read operation
tmr_listener::EthernetSlaveCmd cmd;
cmd.request.id = "StartProject";
cmd.request.item_list.emplace_back("Stick_PlayPause");

if (not ros::service::call("/tmr_eth_slave/tmsvr_cmd", cmd)) {
  ROS_ERROR_STREAM("service fail");
} else {
  auto const result = tmr_listener::parse_as<bool>(cmd.value_list[0]);
}

```

### Exported Data Table XML

**(Not tested yet, worked on my machine though)**

If the data table you are going to use to receive data periodically is not going to change that frequently, you can let tmr_listener generate the message file for you. All you need to do is to export the data table, see the **Operation Interface** - **System Setting** - **Import/Export** in Software-Manual-TMFlow. The exported file is in XML format, then build tmr_listener, specifying the path to the exported xml:

```cmake
catkin build tmr_listener -DTM_ETHERNET_SLAVE_XML=path/to/your/xml/file
```

This command will generate `TMREthernet.msg` under `msg` directory, to recieve the data, subscribe to this topic `/tmr_eth_slave/exported_data_table`, that is all!

(**Note: Everytime the data table is changed and you still want to use this feature, remember to rebuild the package, otherwise the node will end immediately once it detect mismatched message**)

## Unit Test

To run unit test, copy paste the following lines to the terminal:

### Linux

```sh
catkin build tmr_listener -DTMR_ENABLE_TESTING=ON
catkin run_tests tmr_listener
ctest --test-dir $(catkin locate -b tmr_listener) -E _ctest # --test-dir is available since CMake 3.20, for older version, just `cd` to the one with test sets and run ctest
```

### Windows

Before running unit test, **you must clone `googletest` to the workspace**, despite that LKG build did provide both gmock and gtest. (catkin can't find gmock since the directory layout doesn't satisfy the assumption it made, even if the directory is manually specified, tests still fail to compile because of some unresolved references) (2021/09/23).

``` sh
catkin_make -DTMR_ENABLE_TESTING=ON
catkin_make run_tests -j1

cd <catkin_ws/build> && ctest -E _ctest
```

The first two command should be easy to understand, the third one is used to run compile time test, by checking if the program can be compiled or not. Test created via `catkin_add_gtest`, `catkin_add_gmock`, `add_rostest_gtest`, `add_rostest_gmock`, and `add_rostest` are also added to `ctest` with test name started with `_ctest`, and are thus excluded here.

## TODO

-   General
    -   More Unit test
    -   Consider the case where user would like to run these two in one executable?
-   Parser object
    -   Maybe I should adapt ros msg object as spirit parse data storage class
-   ROS interface
    -   More services
        -   load plugin dynamically
-   TM external script language
    -   Type conversion operator, TM has some "unique" type conversion rules, which is totally BS to me
    -   consider function accepting types that can be implicitly converted to the desired type
    -   TM functions, and project variables
    -   MUST disable user construct Expression from string, only internally usable
    -   Reply if tm message not yet respond is bugged since the response is not queued, fix it in the future

## Contact

Jacky Tseng - jacky.tseng@gyro.com.tw

## Reference

1.  [TM expression editor and listen node reference manual](https://www.tm-robot.com/en/wpdmdownload/expression-editor-manual_sw1-82/)
2.  [pluginlib tutorial](http://wiki.ros.org/pluginlib/Tutorials/Writing%20and%20Using%20a%20Simple%20Plugin)
3.  [the performance benefits of final classes](https://devblogs.microsoft.com/cppblog/the-performance-benefits-of-final-classes/)
4.  [Professional CMake - A practical guide](https://crascit.com/professional-cmake/)
5.  [boost asio for tcp socket programming](https://www.boost.org/doc/libs/1_58_0/doc/html/boost_asio.html)
6.  [cpp starter project](https://github.com/lefticus/cpp_starter_project)

## Notes

-   [Memory leak issue due to plugin lib](https://github.com/ros/class_loader/issues/131)
-   [undefined behavior in boost::format](https://svn.boost.org/trac10/ticket/11632)
-   [undefined behavior in boost::xpressive](https://github.com/boostorg/xpressive/issues/14)
-   [gcc ubsan + asan log_path bug](https://gcc.gnu.org/bugzilla/show_bug.cgi?id=94328)
-   [asan comes with lsan](https://github.com/google/sanitizers/wiki/AddressSanitizerLeakSanitizer)
