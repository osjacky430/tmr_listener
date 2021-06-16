#ifndef TMR_LISTENER_HANDLE_HPP_
#define TMR_LISTENER_HANDLE_HPP_

#include <boost/tokenizer.hpp>
#include <iostream>
#include <string>

#include "tmr_ext_script/tmr_motion_function.hpp"

namespace tmr_listener {

enum class Decision { Accept, Ignore };

/**
 * @brief This function is the main interface exposed to the user, end user implement listen node task handler by
 *        inheriting this class. For detail description, see ["Creating your own listener handle" part in top level
 *        README.md](tmr_listener/README.md)
 */
class ListenerHandle {
 public:
  enum class MessageStatus { Responded, NotYetRespond };

 private:
  MessageStatus responded_ = MessageStatus::NotYetRespond;

 protected:
  /**
   * @brief This function generates TM external script commands, this is left for end user to implement.
   *
   * @param t_prev_response Did TM robot responded to previous message
   * @return motion_function::MessagePtr
   */
  virtual MessagePtr generate_cmd(MessageStatus t_prev_response) = 0;

  virtual void response_msg(TMSTAResponse::Subcmd00 const& /*unused*/) {}
  virtual void response_msg(TMSTAResponse::Subcmd01 const& /*unused*/) {}
  virtual void response_msg(TMSCTResponse const& /*unused*/) {}
  virtual void response_msg(CPERRResponse const& /*unused*/) {}
  virtual void response_msg() {}

  /**
   * @brief This function informs tmr_listener whether current handle is going to take on the task, this is left
   *        for end user to implement
   *
   * @param t_data  message sent from TM when entered listener node, this argument contains only the data section, see
   *                TM expression editor and listen node manual, TMSCT section.
   *
   * @return Decision::Accept   informs listener to use current handler to send message to TM
   * @return Decision::Ignore   informs listener not to use current handler
   *
   * @note Only one handle at a time currently (@todo maybe extend to support multiple handles in the future)
   */
  virtual Decision start_task(std::string const& t_data) = 0;

 public:
  /**
   * @brief This function parses the message TM sent when entered listen node, and check if the handler is the one to
   *        handle the task
   *
   * @param t_data  message sent from TM when entered listener mode
   *
   * @return Decision::Accept   informs listener to use current handler to send message to TM
   * @return Decision::Ignore   informs listener not to use current handler
   */
  Decision start_task_handling(std::string const& t_data) noexcept;

  /**
   * @brief This function parses the messages sent from TM, after parsing the messages, it will call one of the
   *        callbacks (ListenerHandler::response_msg overload sets) according to the header of the message.
   *
   * @param t_packet  message sent from TM
   */
  template <typename T>
  void handle_response(T const& t_packet) noexcept {
    this->responded_ = MessageStatus::Responded;
    this->response_msg(t_packet);
    this->response_msg();
  }

  /**
   * @brief
   */
  virtual void handle_disconnect() noexcept {}

  /**
   * @brief This function generates request to send to TM robot, it calls ListenerHandle::generate_cmd internally
   *
   * @return MessagePtr
   */
  MessagePtr generate_request() noexcept;

  ListenerHandle()                                 = default;
  ListenerHandle(ListenerHandle const& /*unused*/) = default;
  ListenerHandle(ListenerHandle&& /*unused*/)      = default;

  ListenerHandle& operator=(ListenerHandle const& /*unused*/) = default;
  ListenerHandle& operator=(ListenerHandle&&) /*unused*/ = default;

  virtual ~ListenerHandle() = default;
};

}  // namespace tmr_listener

#endif