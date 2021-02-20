## Design consideration

The overall design consideration can be summarized into the picture down below:

```
                                              ----------> Listen node handler 1
 _____________               ______________  /    .
|             |   TX, RX    |              |     (2)
|    TM14     |  <------>   | tmr_listener | <----------> Listen node handler m   (only one handler is active when entered listen node)
|_____________|     (1)     |______________|      .
                                             \----------> Listen node handler n

                              tmr_listener handles communication (1) and task dispatch (2)
```

Doing so not only adhere to OCP (open-closed principle), but also hide the connection implementation from the end user. User has one and only one responsibility -- to implement the ListenerHandle class -- then everything is good to go.

Secondly, even though it is user's responsibility to generate/parse the command, user can still screw things up. Fortunately, tmr_listener also provides user friendly command generater.

With this in mind, the design constraint will be:

- Provide uniform interface (ListenerHandle)
  - Furthermore, to avoid end user messes up with the library, e.g. returning random string, the result is wrapped in a struct `Command`. This struct can only be initialized by the command builder (@todo implement)
- Create an easy to use, hard to misuse command generator. To me, this means that:
  1.  the interface is simple, (syntax that can be understanded intuitively, **AND** similar)
      - Builder pattern + fluent interface using `operator<<` is adopted, considering the command can easily scale up since there are quite a few motion functions, not to mention their possible combinations, doing so makes the syntax of the message generator intuitive.
  2.  misusing it will issue compile error instead of runtime error
      - Misusing = compile error: Meta programming, strong type for strong interface

## Why Pluginlib?

Seeing the picture of the design consideration above, one may realize the reason behind this already. Briefly speaking, ROS provides three ways (at least I could think of) of message exchanging: 1. `Publisher/Subscriber` 2. `Service` 3. `Actionlib`. But they have some common drawbacks, e.g., they can't tell the end user when did TM robot enter listen node mode, at least not in the easy way, might involve multiple connections to same port, which is impossible to manage well for mulituple listen node handler case.

As for `Pluginlib`, even though we implement the task handlers in different packages, the execution part is controlled by one node, i.e., no multiple communication needed, easy to implement listen mode entered notification.

## TM external script language

The external script language of TM robot is quite difficult to implement, due to the fact that it has the concept of named variable. This library decomposed the external script language into several parts:

- Variable: namely named variable, declaration, and its operators.
- Function call, e.g. Queue(1, 1), PTP("CPP", 161.f, 241.f, 470.f, -179.f, 0.f, 175.f, 60, 200, 0, false), etc.
- Combination of two things above.

### Variables

Under construction...

### Function call

Under construction...

### Message generation

There are two commands in TM external script language, i.e., TMSCT and TMSTA. TMSTA is relatively simple (by simple, I mean limited amount of combinations), whereas TMSCT is complicated. One way to implement message generation is to wrap the hard coded command of TMSTA into function, this is possible due to its simplicity, and fluent interface with builder pattern for TMSCT:

```cpp
namespace tm_robot_listener {

auto queue_tag_done(int const t_tag_number) {
  return TMSTACommand{"$TMSTA,01," + std::to_string(t_tag_number) + ...};
}

auto is_in_ext_script_ctl() {
  return TMSTACommand{"$TMSTA,00," ...};
}

/* the rest of the TMSTA command */

struct TMSCTHeader {
  decltype(auto) operator<<(TMSCTCommand const& t_cmd) const noexcept {
    /* implementation */
    return *this;
  }
};

static constexpr TMSCTHeader TMSCT{};

TMR_MOTION_FUNC(QueueTag, RETURN_TYPE(bool), SIGNATURE(Int), SIGNATURE(Int, Int));

}

using namespace tm_robot_listener;
// to generate TMSCT message
auto const cmd_tmsct = TMSCT << QueueTag(1, 1) << End();

// to generate TMSTA message
auto const cmd_tmsta = queue_tag_done();
```

Even though this can prevent user from misusing the library, the process of message generation is no longer similar (not both fluent interface). To make the process similar, we can create two classes that handles each command generations (using fluent interface, of course), as long as they return the same thing. However, we have another disadvantage: the error can't be related to business logic, for example:

```cpp
namespace tm_robot_listener {

struct TMSCTCommand;
struct TMSTACommand;

struct TMSCTHeader {
  static constexpr auto HEADER() { return "$TMSCT"; }
  auto operator<<(TMSCTCommand const& t_cmd) const noexcept { /* implementation */ }
};

struct TMSTAHeader {
  static constexpr auto HEADER() { return "$TMSTA"; }
  auto operator<<(TMSTACommand const& t_cmd) const noexcept { /* implementation */ }
};

static constexpr TMSCTHeader TMSCT{};
static constexpr TMSTAHeader TMSTA{};

TMR_MOTION_FUNC(QueueTag, RETURN_TYPE(bool), SIGNATURE(Int), SIGNATURE(Int, Int));
}

// QueueTag is motion function, can be used in TMSCT only (reference: tm_expression_editor_and_listen_node_reference_manual_en.pdf P209)
using namespace tm_robot_lisetner;
TMSTA << QueueTag(1, 1) << End(); // This will generate compile error: no known conversion from TMSCTCommand to TMSTACommand
                                  // One may wonder: why and what is this error?
```

If we want to create our own error messages, we must use `static_assert`, or something `concept`-ish. We have no choice but to template the class, thanks to lazy evaluation, the error will only show up unless we use it:

```cpp
namespace tm_robot_listener {

struct TMSTATag {
  static constexpr auto HEADER() { return "$TMSTA"; }
};

struct TMSCTTag {
  static constexpr auto HEADER() { return "$TMSCT"; }
};

template <typename HeaderTag>
struct Header {
  template <typename CommandTag>
  auto operator<<(Command<CommandTag> const& t_cmd) const noexcept {
    static_assert(std::is_same<CommandTag, HeaderTag>::value, "This command cannot be used by this header");

    /* implementation */
  }
};

static constexpr Header<TMSCTTag> TMSCT{};
static constexpr Header<TMSTATag> TMSTA{};

TMR_MOTION_FUNC(QueueTag, RETURN_TYPE(bool), SIGNATURE(Int), SIGNATURE(Int, Int));
}

using namespace tm_robot_listener;
TMSTA << QueueTag(1, 1) << End(); // This will generate compile error: This command cannot be used by this header
```

### Notes

1. ScriptExit will return OK
2. Variable can't be declared and used at the same line
3. One line can only have one command, TMSTA accept only one command at a time
