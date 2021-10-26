#ifndef TMR_MOTION_FUNCTION_HPP_
#define TMR_MOTION_FUNCTION_HPP_

#include <boost/make_shared.hpp>

#include <string>
#include <utility>
#include <vector>

#include "tmr_ext_script/detail/tmr_function.hpp"
#include "tmr_ext_script/detail/tmr_header_tag.hpp"
#include "tmr_prototype/tmr_header.hpp"
#include "tmr_variable.hpp"
#include "version.hpp"

namespace tmr_listener {

using MessagePtr = boost::shared_ptr<prototype::MessageBase>;

using TMSCTHeader    = prototype::Header<detail::TMSCTTag>;
using TMSCTPacket    = TMSCTHeader::Packet;
constexpr auto TMSCT = TMSCTHeader{};

using TMSTAHeader    = prototype::Header<detail::TMSTATag>;
using TMSTAPacket    = TMSTAHeader::Packet;
constexpr auto TMSTA = TMSTAHeader{};

using CPERRHeader    = prototype::Header<detail::CPERRTag>;
using CPERRPacket    = CPERRHeader::Packet;
constexpr auto CPERR = CPERRHeader{};

namespace motion_function {

// clang-format off
#define TMR_MOTION_FUNC(name, ret_type, ...)  constexpr tmr_listener::detail::TMSTCFuncSet<ret_type, __VA_ARGS__> name { #name }
#define TMR_SUBCMD(name, subcmd, ...)         constexpr tmr_listener::detail::TMSTAFuncSet<__VA_ARGS__> name { #subcmd }
#define SIGNATURE(...)                        tmr_listener::detail::Function<__VA_ARGS__>
#define RETURN_TYPE(type)                     type
#define TMR_VOID
// clang-format on

/**
 * @brief Set robot motions with Queue Tag Numbers to denote the current robot motion in process. The status of each
 *        queue tag can be monitored using TMSTA SubCmd 01.
 *
 * @details bool QueueTag(int t_tag_number, int t_wait)     (1)
 *          bool QueueTag(int t_tag_number)                 (2)
 *
 *          1) int: The tag number. Valid for integers between 1 and 15.
 *             int: Wait for the tagging to continue processing or not.
 *
 *                  0 Not wait (default)
 *                  1 Wait
 *
 *                  When the value is set to 1, the process stays in the function and waits for the tagging to complete
 *                  and continue processing.
 *
 *          2) The syntax is the same as syntax (1) The default is to not wait for the tagging to continue processing.
 */
TMR_MOTION_FUNC(QueueTag, RETURN_TYPE(bool), SIGNATURE(int), SIGNATURE(int, int));

/**
 * @brief Wait for the Queue Tag Number of the robot motion to complete.
 *
 * @details int WaitQueueTag(int tag_number_to_wait, int timeout)   (1)
 *          int WaitQueueTag(int tag_number_to_wait)                (2)
 *
 *          1) int: The tag number waiting in queue.
 *             int: Set the time to timeout
 *
 *          2) The syntax is the same as syntax 1. The default is no timeout and required to wait for the tagging to
 *             complete (or not existed)
 *
 * @note There is a slight difference between different software versions, please refer to the corresponding
 *       version of "Expression Editor and Listen Node" for more detail
 */
TMR_MOTION_FUNC(WaitQueueTag, RETURN_TYPE(int), SIGNATURE(int), SIGNATURE(int, int));

/**
 * @brief Stop the motion of the robot and clear existing commands of the robot in the buffer.
 *
 * @details bool StopAndClearBuffer()
 */
TMR_MOTION_FUNC(StopAndClearBuffer, RETURN_TYPE(bool), SIGNATURE(TMR_VOID));

/**
 * @brief Pause the project and the motion of the robot other than non-paused threads and external script. Use Resume()
 *        or press the Play button on the robot stick to resume.
 *
 * @details bool Pause()
 */
TMR_MOTION_FUNC(Pause, RETURN_TYPE(bool), SIGNATURE(TMR_VOID));

/**
 * @brief Resume the project and the motion of the robot.
 *
 * @details bool Resume()
 */
TMR_MOTION_FUNC(Resume, RETURN_TYPE(bool), SIGNATURE(TMR_VOID));

/**
 * @brief Define and send PTP motion command into buffer for execution.
 *
 * @details bool PTP(string t_data_format, float[] t_motion_target, int t_speed_percentage, int t_time_to_top_speed,
 *                   int t_blending_percentage, bool t_precise_positioning)                                          (1)
 *          bool PTP(string t_data_format, float[] t_motion_target, int t_speed_percentage, int t_time_to_top_speed,
 *                   int t_blending_percentage, bool t_precise_positioning, int[] t_robot_pose)                      (2)
 *          bool PTP(string t_data_format, float, float, float, float, float, float, int t_speed_percentage,
 *                   int t_time_to_top_speed, int t_blending_percentage, bool t_precise_positioning)                 (3)
 *          bool PTP(string t_data_format, floa at, float, float, float, int t_speed_percentage,
 *                   int t_time_to_top_speed, int t_blending_percentage, bool t_precise_positioning, int, int, int)  (4)
 *
 *             string: Definition of data format, it is composed of three letters
 *
 *                     #1 - Motion target format: "J" for joint angles, and "C" for Cartesian coordinate.
 *                                                This can only be "C" for syntax 2.
 *                     #2 - Speed format: "P" for percentage
 *                     #3 - Blending format: "P" for percentage
 *
 *             Motion target data: float[] (syntax 1, 2), or float, float, float, float, float, float (syntax 3, 4)
 *
 *                      If motion target == "J", then this represent the angle of the joint 1 - 6
 *                      If motion target == "C", then this represent the coordinate of TCP (XYZ)
 *
 *             int: The speed setting in percentage
 *             int: The time interval to accelerate to top speed in ms
 *             int: Blending value in percentage
 *             bool: precise position disabled, set to true to disable precise positioning
 *
 *             The pose of the robot: int[] (syntax 2), or int, int, int (syntax 4)
 *
 * @note Refer to the Expression Editor and Listen Node for the definition of the pose of the robot
 */
TMR_MOTION_FUNC(PTP, RETURN_TYPE(bool), SIGNATURE(std::string, std::array<float, 6>, int, int, int, bool),
                SIGNATURE(std::string, std::array<float, 6>, int, int, int, bool, std::array<int, 3>),
                SIGNATURE(std::string, float, float, float, float, float, float, int, int, int, bool),
                SIGNATURE(std::string, float, float, float, float, float, float, int, int, int, bool, int, int, int));

/**
 * @brief Define and send Line motion command into buffer for execution.
 *
 * @details bool Line(string t_data_format, float[] t_motion_target, int t_speed_setting, int t_time_to_top_speed,
 *                    int t_blending, bool t_precise_positioning)                                           (1)
 *          bool Line(string t_data_format, float, float, float, float, float, float, int t_speed_setting,
 *                    int t_time_to_top_speed, int t_blending, bool t_precise_positioning)                  (2)
 *
 *            string: Definition of the data format, it is composed of three letters
 *
 *                     #1 - Motion target format: "C" for Cartesian coordinate.
 *                     #2 - Speed format: "P" for percentage, or "A" for velocity (mm/s)
 *                     #3 - Blending format: "P" for percentage, or "R" for radius (mm)
 *
 *            Motion target data: float[] (syntax 1), or float, float, float, float, float, float (syntax 2), which
 *                                stands for X (mm), Y (mm), Z (mm), Rx, Ry, Rz of TCP
 *
 *            int: The speed setting in percentage or in velocity (mm/s)
 *            int: Blending value in percentage or in radius (mm)
 *            bool: precise position disabled, set to true to disable precise positioning
 */
TMR_MOTION_FUNC(Line, RETURN_TYPE(bool), SIGNATURE(std::string, std::array<float, 6>, int, int, int, bool),
                SIGNATURE(std::string, float, float, float, float, float, float, int, int, int, bool));

/**
 * @brief Define and send Circle motion command into buffer for execution.
 *
 * @details bool Circle(string t_data_format, float[] t_point_on_arc, float[] t_arc_endpoint, int t_speed_setting,
 *                      int t_time_to_top_speed, int t_blending_percentage,
 *                      int t_arc_angle, bool t_precise_positioning)                                                (1)
 *          bool Circle(string, t_data_format, float, float, float, float, float, float,
 *                      float, float, float, float, float, float, int t_speed_setting, int t_time_to_top_speed,
 *                      int t_blending_percentage, int t_arc_angle, bool t_precise_positioning)                     (2)
 *
 *            string: Definition of the data format, it is composed of three letters
 *
 *                     #1 - Motion target format: "C" for Cartesian coordinate.
 *                     #2 - Speed format: "P" for percentage, or "A" for velocity (mm/s)
 *                     #3 - Blending format: "P" for percentage
 *
 *            A point on arc: float[] (syntax 1), or float, float, float, float, float, float (syntax 2), which
 *                            stands for X (mm), Y (mm), Z (mm), Rx, Ry, Rz of TCP
 *
 *            Endpoint of arc: float[] (syntax 1), or float, float, float, float, float, float (syntax 2), which
 *                             stands for X (mm), Y (mm), Z (mm), Rx, Ry, Rz of TCP
 *
 *            int: The speed setting in percentage or in velocity (mm/s)
 *            int: Blending value in percentage or in radius (mm)
 *            int: Arc angle. If non-zero value is given, the TCP will keep the same pose. Otherwise, the TCP will
 *                 move
 *            bool: precise position disabled, set to true to disable precise positioning
 */
TMR_MOTION_FUNC(Circle, RETURN_TYPE(bool),
                SIGNATURE(std::string, std::array<float, 6>, std::array<float, 6>, int, int, int, bool),
                SIGNATURE(std::string, float, float, float, float, float, float, float, float, float, float, float,
                          float, int, int, int, bool));

/**
 * @brief Define and send PLine motion command into buffer for execution.
 *
 * @details bool PLine(string t_data_format, float[] t_motion_target, int t_speed_setting,
 *                     int t_time_to_top_speed, int t_blending_percentage)                        (1)
 *          bool PLine(string t_data_format, float, float, float, float, float, float,
 *                     int t_speed_setting, int t_time_to_top_speed, int t_blending_percentage)   (2)
 *
 *            string: Definition of the data format, it is composed of three letters
 *
 *                     #1 - Motion target format: "J" for joint angles, or "C" for Cartesian coordinate.
 *                     #2 - Speed format: "A" for velocity (mm/s)
 *                     #3 - Blending format: "P" for percentage
 *
 *            Motion target: float[] (syntax 1), or float, float, float, float, float, float (syntax 2), which
 *                           stands for joint angle 1 ~ 6 for motion target format == "J", and X (mm), Y (mm), Z (mm),
 *                           Rx, Ry, Rz of TCP for motion target format == "C"
 *
 *            int: The speed setting in velocity (mm/s)
 *            int: The time interval to accelerate to top speed in ms
 *            int: Blending value in percentage or in radius (mm)
 */
TMR_MOTION_FUNC(PLine, RETURN_TYPE(bool), SIGNATURE(std::string, std::array<float, 6>, int, int, int),
                SIGNATURE(std::string, float, float, float, float, float, float, int, int, int));

/**
 * @brief Define and send PTP relative motion commands for execution.
 *
 * @details bool Move_PTP(string t_data_format, float[] t_relative_motion_param, int t_speed_percentage,
 *                        int t_time_to_top_speed, int t_blending_percentage, bool t_precise_positioning)         (1)
 *          bool Move_PTP(string t_data_format, float, float, float, float, float, float, int t_speed_percentage,
 *                        int t_time_to_top_speed, int t_blending_percentage, bool t_precise_positioning)         (2)
 *
 *            string: Definition of the data format, it is composed of three letters
 *
 *                     #1 - Data format: "C" for current base, or "T" for tool coordinate, or "J" for joint angles.
 *                     #2 - Speed format: "P" for percentage
 *                     #3 - Blending format: "P" for percentage
 *
 *            Relative motion parameters: float[] (syntax 1), or float, float, float, float, float, float (syntax 2),
 *                                        which stands for joint angle 1 ~ 6 for data format == "J", and X (mm), Y (mm),
 *                                        Z (mm), Rx, Ry, Rz of TCP motion value relative to specified coordiate, i.e.
 *                                        "C" or "T"
 *
 *            int: The speed setting in percentage
 *            int: The time interval to accelerate to top speed in ms
 *            int: Blending value in percentage or in radius (mm)
 *            bool: precise position disabled, set to true to disable precise positioning
 */
TMR_MOTION_FUNC(Move_PTP, RETURN_TYPE(bool), SIGNATURE(std::string, std::array<float, 6>, int, int, int, bool),
                SIGNATURE(std::string, float, float, float, float, float, float, int, int, int, bool));

/**
 * @brief Define and send Line relative motion commands for execution.
 *
 * @details bool Move_Line(string t_data_format, float[] t_relative_motion_param, int t_speed_setting,
 *                         int t_time_to_top_speed, int t_blending_value, bool t_precise_positioning)             (1)
 *          bool Move_Line(string t_data_format, float, float, float, float, float, float, int t_speed_setting,
 *                         int t_time_to_top_speed, int t_blending_value, bool t_precise_positioning)             (2)
 *
 *            string: Definition of the data format, it is composed of three letters
 *
 *                     #1 - Data format: "C" for current base, or "T" for tool coordinate.
 *                     #2 - Speed format: "P" for percentage, or "A" for velocity (mm/s)
 *                     #3 - Blending format: "P" for percentage, or "R" for radius
 *
 *            Relative motion parameters: float[] (syntax 1), or float, float, float, float, float, float (syntax 2),
 *                                        which stands for X (mm), Y (mm), Z (mm), Rx, Ry, Rz of TCP motion value
 *                                        relative to specified coordiate, i.e. "C" or "T"
 *
 *            int: The speed setting in percentage or in velocity (mm/s)
 *            int: The time interval to accelerate to top speed in ms
 *            int: Blending value in percentage or in radius (mm)
 *            bool: precise position disabled, set to true to disable precise positioning
 */
TMR_MOTION_FUNC(Move_Line, RETURN_TYPE(bool), SIGNATURE(std::string, std::array<float, 6>, int, int, int, bool),
                SIGNATURE(std::string, float, float, float, float, float, float, int, int, int, bool));

/**
 * @brief Define and send PLine relative motion commands for execution.
 *
 * @details bool Move_PLine(string t_data_format, float[] t_relative_motion_param, int t_speed_setting,
 *                          int t_time_to_top_speed, int t_blending_value)             (1)
 *          bool Move_PLine(string t_data_format, float, float, float, float, float, float, int t_speed_setting,
 *                          int t_time_to_top_speed, int t_blending_value)             (2)
 *
 *            string: Definition of the data format, it is composed of three letters
 *
 *                     #1 - Data format: "C" for current base, or "T" for tool coordinate, or "J" for joint angle
 *                     #2 - Speed format: "A" for velocity (mm/s)
 *                     #3 - Blending format: "P" for percentage, or "R" for radius
 *
 *            Relative motion parameters: float[] (syntax 1), or float, float, float, float, float, float (syntax 2),
 *                                        which stands for joint angle 1 ~ 6 for data format == "J", and X (mm), Y (mm),
 *                                        Z (mm), Rx, Ry, Rz of TCP motion value relative to specified coordiate, i.e.
 *                                        "C" or "T"
 *
 *            int: The speed setting in percentage or in velocity (mm/s)
 *            int: The time interval to accelerate to top speed in ms
 *            int: Blending value in percentage
 */
TMR_MOTION_FUNC(Move_PLine, RETURN_TYPE(bool), SIGNATURE(std::string, std::array<float, 6>, int, int, int),
                SIGNATURE(std::string, float, float, float, float, float, float, int, int, int));

/**
 * @brief Send the command of changing the base of the follow-up motions into buffer for execution.
 *
 * @details bool ChangeBase(string t_base_name)                         (1)
 *          bool ChangeBase(float[] t_base_param)                       (2)
 *          bool ChangeBase(float, float, float, float, float, float)   (3)
 *
 *          1) string: base name
 *          2) float[]: base parameter, X, Y, Z, RX, RY, RZ
 *          3) float, float, float, float, float, float: base parameter, X, Y, Z, RX, RY, RZ
 */
TMR_MOTION_FUNC(ChangeBase, RETURN_TYPE(bool), SIGNATURE(std::string), SIGNATURE(std::array<float, 6>),
                SIGNATURE(float, float, float, float, float, float));

/**
 * @brief Send the command of changing the TCP of the follow-up motions into buffer for execution.
 *
 * @details bool ChangeTCP(string t_tcp_name)                                                               (1)
 *          bool ChangeTCP(float[] t_tcp_coor)                                                              (2)
 *          bool ChangeTCP(float[] t_tcp_coor, float t_tcp_weight)                                          (3)
 *          bool ChangeTCP(float... t_tcp_coor, float t_tcp_weight)                                         (4)
 *          bool ChangeTCP(float[] t_tcp_coor, float t_tcp_weight, float[] t_tcp_moi_and_reference_frame)   (5)
 *          bool ChangeTCP(float... t_tcp_coor, float t_tcp_weight, float... t_tcp_moi_and_reference_frame) (6)
 *
 *          1) string: tcp name
 *          2) float[]: tcp coordinate, X, Y, Z, Rx, Ry, Rz
 *          3) float[]: tcp coordinate, X, Y, Z, Rx, Ry, Rz
 *             float: tcp weight
 *          4) float... (first 6): tcp coordinate
 *             float: tcp weight
 *          5) float[]: tcp coordinate, X, Y, Z, Rx, Ry, Rz
 *             float: tcp weight
 *             float[]: tcp moment of intertia, Ixx, Iyy, Izz, and its frame of reference, X, Y, Z, Rx, Ry, Rz
 *          6) float... (first 6): tcp coordinate
 *             float: tcp weight
 *             float... (last 9): tcp moment of intertia, Ixx, Iyy, Izz, and its frame of reference, X, Y, Z,
 *                                          Rx, Ry, Rz
 */
TMR_MOTION_FUNC(ChangeTCP, RETURN_TYPE(bool), SIGNATURE(std::string), SIGNATURE(std::array<float, 6>),
                SIGNATURE(std::array<float, 6>, float), SIGNATURE(std::array<float, 6>, float, std::array<float, 9>),
                SIGNATURE(float, float, float, float, float, float),
                SIGNATURE(float, float, float, float, float, float, float),
                SIGNATURE(float, float, float, float, float, float, float, float, float, float, float, float, float,
                          float, float, float));

/**
 * @brief Send the command of changing the payload value of the follow-up motions into buffer for execution.
 *
 * @details bool ChangeLoad(float t_load)   (1)
 *
 *          1) float: load in kg
 */
TMR_MOTION_FUNC(ChangeLoad, RETURN_TYPE(bool), SIGNATURE(float));

/**
 * @brief Set PVT mode to start with Joint/Cartesian command
 *
 * @details bool PVTEnter(int t_pvt_mode)    (1)
 *          bool PVTEnter()                  (2)
 *
 *          1) int: PVTMode, 0 for joint, and 1 for Cartesian
 *          2) Default joint mode
 */
TMR_MOTION_FUNC(PVTEnter, RETURN_TYPE(bool), SIGNATURE(int), SIGNATURE(TMR_VOID));

/**
 * @brief Set PVT mode motion to exit
 *
 * @details bool PVTExit()  (1)
 */
TMR_MOTION_FUNC(PVTExit, RETURN_TYPE(bool), SIGNATURE(TMR_VOID));

/**
 * @brief Set the PVT mode parameters of motion in position, velocity, and duration.
 *
 * @details bool PVTPoint(float[] t_target_position, float[] t_target_velocity, float t_duration)     (1)
 *          bool PVTPpoint(float... t_target_position, float... t_target_velocity, float t_duration)  (2)
 *
 *          Target position: float[] (syntax 1), or first six float (syntax 2). Joint angle 1 ~ 6 for Joint mode, and X,
 *                           Y, Z, RX, RY, RZ for Cartesian mode
 *          Target velocity: float[] (syntax 1), or 7th - 12th float (syntax 2): joint speed for Joint mode, Vx, Vy, Vz,
 *                           Wx, Wy, Wz for Cartesian mode
 *          float: duration (second for version == 1.82, millisecond for version <= 1.80)
 */
TMR_MOTION_FUNC(PVTPoint, RETURN_TYPE(bool), SIGNATURE(std::array<float, 6>, std::array<float, 6>, float),
                SIGNATURE(float, float, float, float, float, float, float, float, float, float, float, float, float));

/**
 * @brief Set PVT mode motion to pause
 *
 * @details bool PVTPause() (1)
 */
TMR_MOTION_FUNC(PVTPause, RETURN_TYPE(bool), SIGNATURE(TMR_VOID));

/**
 * @brief Set PVT mode motion to resume
 *
 * @details bool PVTMotion() (1)
 */
TMR_MOTION_FUNC(PVTResume, RETURN_TYPE(bool), SIGNATURE(TMR_VOID));

#if CURRENT_TMFLOW_VERSION_GE(1, 82, 0000)

/**
 * @brief Set the present vision job in the current project in execution to queue up with motion commands.
 *
 * @details bool Vision_DoJob(string t_vision_job_name) (1)
 *
 *          string: Vision job name
 *
 * @note  To use this motion function, user should uncheck the "start at initial position" checkbox to disable "move to
 *        initial position". Also, the vision job called should be created in the first place to use this motion
 *        function.
 */
TMR_MOTION_FUNC(Vision_DoJob, RETURN_TYPE(bool), SIGNATURE(std::string));

/**
 * @brief Set the present vision job in the current project in execution, by moving to the initial position in PTP, to
 *        queue up with motion commands.
 *
 * @details bool Vision_DoJob_PTP(string t_vision_job_name, int t_speed_percentage, int t_time_to_top_speed,
 *                                bool t_smart_pose)
 *
 *          string: Vision job name
 *          int: The speed setting in percentage
 *          int: The time interval to accelerate to top speed in ms
 *          bool: true to use the pose of the robot determined by smart pose
 *                false to use the pose of the robot recorded taught in vision job
 *
 * @note  same as Vision_DoJob
 */
TMR_MOTION_FUNC(Vision_DoJob_PTP, RETURN_TYPE(bool), SIGNATURE(std::string, int, int, bool));

/**
 * @brief Set the present vision job in the current project in execution, by moving to the initial position in Line, to
 *        queue up with motion commands.
 *
 * @details bool Vision_DoJob_Line(string t_vision_job_name, int t_speed_percentage)                         (1)
 *          bool Vision_DoJob_Line(string t_vision_job_name, int t_speed_velocity, int t_time_to_top_speed)  (2)
 *
 *          1) string: Vision job name
 *             int: The speed setting in percentage
 *          2) string: Vision job name
 *             int: The speed setting in velocity (mm/s)
 *             int: The time interval to accelerate to top speed in ms
 *
 * @note  same as Vision_DoJob
 */
TMR_MOTION_FUNC(Vision_DoJob_Line, RETURN_TYPE(bool), SIGNATURE(std::string, int), SIGNATURE(std::string, int, int));

/**
 * @brief Check if the vision job in the current project present and valid
 *
 * @details bool Vision_IsJobAvailable(string t_vision_job_name)
 *
 *          string: Vision job name
 */
TMR_MOTION_FUNC(Vision_IsJobAvailable, RETURN_TYPE(bool), SIGNATURE(std::string));

#endif

TMR_SUBCMD(InExtScriptCtlMode, 00, SIGNATURE(TMR_VOID));
TMR_SUBCMD(QueueTagDone, 01, SIGNATURE(int));

}  // namespace motion_function

/**
 * @brief return empty commmand list
 *
 * @details If we don't want to response to the message immediately, or the data is not ready at that moment, one may
 *          possibily send empty command list to inform TMRobotListener.
 *
 * @return empty command list
 *
 * @code{.cpp}
 *
 *          class SomeListenNodeEventHandler final : public ListenerHandle {
 *           protected:
 *            motion_function::MessagePtr generate_cmd(MessageStatus const t_prev_response) override {
 *              using namespace motion_function;
 *              if (t_prev_response == MessageStatus::Responded) {
 *                // TM robot responded to the command you sent previously
 *                // do something...
 *              }
 *
 *              // TM robot hasn't responded yet
 *              return empty_command_list();  // this line inform TMRobotListener to do other action
 *            }
 *          };
 *
 * @endcode
 */
inline auto empty_command_list() noexcept { return boost::make_shared<prototype::Message<void>>(); }

inline auto dummy_command_list(std::string t_dummy_cmd_id) { return TMSCT << ID{std::move(t_dummy_cmd_id)} << End(); }

}  // namespace tmr_listener

#define TMR_FUSION_ADAPT_PACKET(name) BOOST_FUSION_ADAPT_STRUCT(name, length_, data_, checksum_)

TMR_FUSION_ADAPT_PACKET(tmr_listener::TMSCTPacket)
TMR_FUSION_ADAPT_PACKET(tmr_listener::TMSTAPacket)
TMR_FUSION_ADAPT_PACKET(tmr_listener::CPERRPacket)

#undef TMR_FUSION_ADAPT_PACKET
#undef TMR_MOTION_FUNC
#undef TMR_SUBCMD
#undef SIGNATURE
#undef RETURN_TYPE
#undef TMR_VOID

#endif