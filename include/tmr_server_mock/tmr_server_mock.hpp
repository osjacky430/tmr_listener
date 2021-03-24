#ifndef TMR_SERVER_MOCK_HPP_
#define TMR_SERVER_MOCK_HPP_

#include <boost/asio.hpp>

#include <boost/bind.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/make_unique.hpp>
#include <boost/program_options.hpp>

#include <boost/format.hpp>
#include <boost/regex.hpp>
#include <boost/thread/scoped_thread.hpp>

#include <boost/mpl/vector.hpp>
#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/front/state_machine_def.hpp>

#include <array>
#include <iostream>

#include "tmr_ext_script/tmr_motion_function.hpp"

namespace tmr_server {

class TMRConnection : public boost::enable_shared_from_this<TMRConnection> {
 public:
  using this_ptr = boost::shared_ptr<TMRConnection>;

  static this_ptr create(boost::asio::io_service& t_io_service) { return this_ptr{new TMRConnection(t_io_service)}; }

  boost::asio::ip::tcp::socket& get_socket() { return socket_; }

  boost::asio::streambuf& get_input_buffer() { return input_buffer_; }

  void write(std::string const& t_input) {
    using namespace boost::asio::placeholders;

    boost::asio::async_write(this->socket_, boost::asio::buffer(t_input),
                             boost::bind(&TMRConnection::handle_write, shared_from_this(), error, bytes_transferred));
  }

 private:
  void handle_write(boost::system::error_code const& /*unused*/, std::size_t const /*unused*/) {}

  std::size_t current_line_ = 0;

  std::string output_buffer_;
  boost::asio::ip::tcp::socket socket_;
  boost::asio::streambuf input_buffer_;
  TMRConnection(boost::asio::io_service& t_io_service) : socket_{t_io_service} {}
};

// event
struct project_start {};
struct project_stop {};
struct disconnect {};
struct receive_connection {};
struct enter_listen_node {};
struct script_exit {};
struct data_time_out {};

struct TMRStateMachine_ : public boost::msm::front::state_machine_def<TMRStateMachine_> {
  template <typename Event, typename FSM>
  void on_entry(Event const&, FSM&) {
    std::cout << "[out] TM robot started.\n";
  }

  template <typename Event, typename FSM>
  void on_exit(Event const&, FSM&) {
    std::cout << "[out] TM robot shutdown.\n";
  }

  struct Stopped : public boost::msm::front::state<> {
    template <typename Event, typename FSM>
    void on_entry(Event const&, FSM&) {
      std::cout << "[out] TM robot enter Stopped mode.\n";
    }

    template <typename Event, typename FSM>
    void on_exit(Event const&, FSM&) {}
  };
  using initial_state = Stopped;

  struct Running : public boost::msm::front::state<> {
    template <typename Event, typename FSM>
    void on_entry(Event const&, FSM&) {
      std::cout << "[out] TM robot enter running mode, starting tcp server.\n";
    }

    template <typename Event, typename FSM>
    void on_exit(Event const&, FSM&) {}
  };

  struct Connected : public boost::msm::front::state<> {
    template <typename Event, typename FSM>
    void on_entry(Event const&, FSM&) {
      std::cout << "[out] Connection success, ready to enter listen node.\n";
      std::cout << "[out] Input listen node data to enter listen node\n";
    }

    template <typename Event, typename FSM>
    void on_exit(Event const&, FSM&) {}
  };

  struct Listening : public boost::msm::front::state<> {
    template <typename Event, typename FSM>
    void on_entry(Event const&, FSM&) {
      std::cout << "[out] Listen node entered, sending message to client.\n";
    }

    template <typename Event, typename FSM>
    void on_exit(Event const&, FSM&) {}
  };

  void create_tcp_server(project_start const&) { std::cout << "[out] TM Robot is preparing for tcp connection.\n"; }

  struct transition_table
    : boost::mpl::vector<a_row<Stopped, project_start, Running, &TMRStateMachine_::create_tcp_server>,
                         _row<Running, receive_connection, Connected>,    //
                         _row<Running, project_stop, Stopped>,            //
                         _row<Running, enter_listen_node, Running>,       //
                         _row<Connected, receive_connection, Connected>,  //
                         _row<Connected, enter_listen_node, Listening>,   //
                         _row<Connected, disconnect, Running>,            //
                         _row<Connected, project_stop, Stopped>,          //
                         _row<Listening, disconnect, Running>,            //
                         _row<Listening, data_time_out, Connected>,       //
                         _row<Listening, project_stop, Stopped>,          //
                         _row<Listening, script_exit, Connected>> {};
};

using TMRStateMachine = boost::msm::back::state_machine<TMRStateMachine_>;

struct TMRTCPServer {
  TMRTCPServer(boost::asio::io_service& t_io_service, TMRStateMachine* t_state_machine)
    : acceptor_{t_io_service, server_}, user_interrupt_{t_io_service, SIGINT}, state_machine_{t_state_machine} {
    using boost::program_options::value;
    this->server_cmd_.add_options()                     //
      ("help", "Show this help message and exit")       //
      ("continue", "Continue running TM robot server")  //
      ("call",
       value<std::string>()->notifier(boost::bind(&TMRTCPServer::enter_listen_node, this, boost::placeholders::_1)),
       "Call listen node by broadcasting message to all listener handle")    //
      ("queue", "Print motion queue status")                                 //
      ("pop-queue", "Mark current motion as done")                           //
      ("exit", "Exit listen node from server side")                          //
      ("skip-ask", "Let server mock replies yes to all incoming message")    //
      ("no-skip-ask", "Ask for the correctness of every incoming messages")  //
      ("reply",
       value<std::string>()->notifier(boost::bind(&TMRTCPServer::reply_question, this, boost::placeholders::_1)),
       "Reply to the question");  //

    using namespace boost::asio::placeholders;
    this->user_interrupt_.async_wait(boost::bind(&TMRTCPServer::handle_user_interrupt, this, error, signal_number));
  }

  static constexpr auto LISTEN_PORT = 5890;

  boost::asio::ip::tcp::endpoint server_{boost::asio::ip::tcp::v4(), LISTEN_PORT};
  boost::asio::ip::tcp::acceptor acceptor_;
  boost::asio::signal_set user_interrupt_;

  boost::program_options::options_description server_cmd_{"TM robot server command"};

  using TagMotionPair = std::pair<std::string, tmr_listener::TMSCTPacket::DataFormat::FunctionCall>;
  std::deque<TagMotionPair> motion_queue_;
  int wait_count_ = 0;

  std::string output_buffer_;
  TMRStateMachine* state_machine_;

  std::vector<TMRConnection::this_ptr> all_connection_;

  void start_accept();

  /**
   * @brief This function handles user interrupt (Ctrl-C, i.e., SIGINT) event.
   *
   * @param t_err   error code
   * @param t_sig_num signal number, currently, only SIGINT is used
   *
   * @todo  It is still buggy, if user press Ctrl-C during handling user interrupt, it will go crazy
   */
  void handle_user_interrupt(boost::system::error_code const& t_err, int const t_sig_num);

  /**
   * @brief This function handles accepted socket connection. When this handle is called, the state machine will advance
   *        to Connected state.
   *
   * @param t_new_connection  new tcp connection request
   * @param t_err error code
   *
   * @note    This handle will let the new socket start an async read process so that user can send TMSTA header
   *          packet even no listen node entered. (see TMSTA in tm_expression_editor_and_listen_node_reference_manual)
   * @todo    Even though people who uses can only send command when listen node is entered, this functionality stated
   *          above should be added in the future
   */
  void handle_accept(TMRConnection::this_ptr t_new_connection, boost::system::error_code const& t_err);

  /**
   * @brief This function is called when async read until condition is satisfied
   */
  void handle_read(boost::system::error_code const& t_err, std::size_t const t_byte_read,
                   TMRConnection::this_ptr t_connection);

  using HeaderPacket = boost::variant<tmr_listener::TMSCTPacket, tmr_listener::TMSTAPacket>;
  HeaderPacket received_packet_;

 private:
  enum class QuestionStatus { NoQuestion, ErrorType, TMSCTErrorLine } question_status_;
  std::array<std::string, 2> question_{
    "No question currently",
    "[out] Is there any following error in the packet received? (Enter 0 if there is no error)\n"
    "[out] [1] Packet Error (CPERR)\t[2] Checksum Error   (CPERR)\t[3] Header Error (CPERR)\n"
    "[out] [4] Data Error   (CPERR)\t[5] Invalid Script   (TMSCT)\n",
  };

  bool skip_ask_ = false;

  void write_to_all_socket(std::string const& t_input) {
    std::for_each(this->all_connection_.begin(), this->all_connection_.end(),
                  [&](auto& t_comm) { t_comm->write(t_input); });
  }

  void enter_cmd_line_mode();

  void interprete_packet(HeaderPacket const& t_received_packet);

  /**
   * @brief This function send listen node message to all socket and advance from connected state to listening state
   *
   * @param t_listen_node_msg
   */
  void enter_listen_node(std::string const& t_listen_node_msg);

  void reply_question(std::string const& t_replied_msg);

  void response_to_error_msg(int const t_err);

};  // namespace tmr_server

class TMRobot {
 private:
  boost::asio::io_service io_service_;

  TMRStateMachine state_machine_;
  std::unique_ptr<TMRTCPServer> tcp_server_{};

 public:
  TMRobot() = default;

  void start() {
    this->state_machine_.start();

    std::cout << "[out] Press Enter to transition to running mode.\n[in ] ";
    std::cin.ignore();

    // create tcp server once
    this->state_machine_.process_event(project_start());

    this->tcp_server_ = std::move(boost::make_unique<TMRTCPServer>(io_service_, &state_machine_));
    this->tcp_server_->start_accept();
    this->io_service_.run();
  }
};

}  // namespace tmr_server

#endif