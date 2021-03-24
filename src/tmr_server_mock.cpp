#include "tmr_server_mock/tmr_server_mock.hpp"

#include <boost/variant/variant.hpp>

namespace {

template <std::size_t N>
void conflicting_options(boost::program_options::variables_map& /**/, char const* const (&/**/)[N]) {
  //
}

// @todo move to tmr_msg_parser.hpp (not created yet)
tmr_server::TMRTCPServer::HeaderPacket parse(std::string t_input) {
  tmr_server::TMRTCPServer::HeaderPacket ret_val;

  boost::spirit::qi::phrase_parse(
    t_input.begin(), t_input.end(),
    (tmr_listener::TMSCTPacket::parsing_rule() | tmr_listener::TMSTAPacket::parsing_rule()),
    boost::spirit::ascii::space, ret_val);

  return ret_val;
}

std::string extract_buffer_data(boost::asio::streambuf& t_buffer, size_t const t_byte_to_extract) {
  if (t_byte_to_extract > 0) {
    auto const buffer_begin = boost::asio::buffers_begin(t_buffer.data());
    std::string const result{buffer_begin, boost::next(buffer_begin, t_byte_to_extract)};
    t_buffer.consume(t_byte_to_extract);

    return result;
  }

  return std::string{""};
}

auto read_buffer_regex() noexcept {
  static auto const regex_pattern = boost::regex(",\\*[0-9a-fA-F]+\r\n");
  return regex_pattern;
}

}  // namespace

namespace tmr_server {

void TMRTCPServer::start_accept() {
  using namespace boost::asio::placeholders;
  auto new_connection = TMRConnection::create(this->acceptor_.get_io_service());

  this->acceptor_.async_accept(new_connection->get_socket(),
                               boost::bind(&TMRTCPServer::handle_accept, this, new_connection, error));
}

void TMRTCPServer::handle_user_interrupt(boost::system::error_code const& t_err, int const t_sig_num) {
  using namespace boost::asio::placeholders;

  if (not t_err and t_sig_num == SIGINT) {
    this->user_interrupt_.async_wait(boost::bind(&TMRTCPServer::handle_user_interrupt, this, error, signal_number));

    std::cout << "\r[out] User interrupt detected. TM robot server is halted now.\n"
                 "[out] Current state: \n"
                 "[out] Type \"--help\" to display all possible options\n";
    this->enter_cmd_line_mode();

    if (not this->output_buffer_.empty()) {
      write_to_all_socket(this->output_buffer_);
    }
  }
}

/**
 * @details This function will extract incoming message, and generate a suitable response (TODO). The generated
 *          response will then send to all connection established.
 */
void TMRTCPServer::handle_read(boost::system::error_code const& t_err, std::size_t const t_byte_read,
                               TMRConnection::this_ptr t_connection) {
  using namespace boost::asio::placeholders;
  if (not t_err) {
    auto const incoming_msg = ::extract_buffer_data(t_connection->get_input_buffer(), t_byte_read);
    std::cout << "[out] Receive incoming message: " << incoming_msg;

    // parse raw response //
    this->output_buffer_.clear();
    this->received_packet_ = ::parse(incoming_msg);
    this->interprete_packet(this->received_packet_);

    this->question_status_ = QuestionStatus::ErrorType;
    if (not this->skip_ask_) {
      // if there is queuetag that must be waited, then the question should not be error type
      std::cout << this->question_[static_cast<std::size_t>(this->question_status_)];
      this->enter_cmd_line_mode();
    } else {
      // @TODO: auto generate response
    }

    if (not this->output_buffer_.empty()) {
      write_to_all_socket(this->output_buffer_);
    }

    // initiate another read process for current client connection
    boost::asio::async_read_until(
      t_connection->get_socket(), t_connection->get_input_buffer(), read_buffer_regex(),
      boost::bind(&TMRTCPServer::handle_read, this, error, bytes_transferred, t_connection));
  }
}

/**
 *  @details TMRTCP server will keep a copy of the connection, so that the server can "broadcast" the message to all the
 *           connection.
 *
 *           Connected state happens if the server receives client connection during execution of the project. During
 *           this state, the server will wait for user input listen data, and once it detects user input, it will change
 *           the state from Connected to Listening.
 */
void TMRTCPServer::handle_accept(TMRConnection::this_ptr t_new_connection, boost::system::error_code const& t_err) {
  using namespace boost::asio::placeholders;

  if (not t_err) {
    this->all_connection_.push_back(t_new_connection);
    this->state_machine_->process_event(receive_connection());

    boost::asio::async_read_until(
      t_new_connection->get_socket(), t_new_connection->get_input_buffer(), read_buffer_regex(),
      boost::bind(&TMRTCPServer::handle_read, this, error, bytes_transferred, t_new_connection));
  }

  start_accept();
}

void TMRTCPServer::enter_cmd_line_mode() {
  using boost::program_options::split_unix;
  using boost::program_options::store;
  using namespace std::string_literals;

  while (true) {
    std::string user_input;
    std::cout << "[in ] ";
    std::cin >> user_input;

    boost::program_options::variables_map vm;
    store(boost::program_options::command_line_parser(split_unix(user_input)).options(this->server_cmd_).run(), vm);
    notify(vm);

    conflicting_options(vm, {"skip-ask", "no-skip-ask"});

    if (vm.count("help") != 0) {
      std::cout << "====\n" << this->server_cmd_ << "===\n";
    }

    if (vm.count("continue") != 0 || vm.count("call") != 0 || vm.count("reply") != 0) {
      // this means all message except themselves will be discarded when these option are used
      break;
    }

    if (vm.count("queue") != 0) {
      std::cout << "[out] Current Tagged motion function: \n";
      for (auto const cmd : this->motion_queue_) {
        std::cout << "[out] format: QueueTag(int tag_number, int wait = 0): QueueTag(" << cmd.first
                  << "), Motion function = " << cmd.second.name_ << ", arguments = " << cmd.second.args_ << '\n';
      }
    }

    if (vm.count("skip-ask") != 0) {
      this->skip_ask_ = true;
    } else if (vm.count("no-skip-ask") != 0) {
      this->skip_ask_ = false;
    }

    if (vm.count("pop-queue") != 0) {
      auto const current_tag = this->motion_queue_.front();

      std::cout << "[out] Marking current executing motion as done: " << current_tag.second.name_ << '('
                << current_tag.second.args_ << ")\n";
      auto const data      = "$TMSTA,%d,01,%d,true,"s;
      this->output_buffer_ = data + '*' + tmr_listener::calculate_checksum(data);
      std::cout << "[out] " << this->output_buffer_ << '\n';
      this->output_buffer_ += "\r\n";
      this->motion_queue_.pop_front();
      break;
    }
  }
}

void TMRTCPServer::interprete_packet(HeaderPacket const& t_received_packet) {
  auto const tmsct = boost::get<tmr_listener::TMSCTPacket>(&t_received_packet);

  if (tmsct != nullptr) {  // @TODO
    tmr_listener::TMSCTPacket::DataFormat::FunctionCall last_function_call;
    auto const queue_tag_predicate = [this, &last_function_call](auto const& t_cmd) {
      auto const motion_func = boost::get<tmr_listener::TMSCTPacket::DataFormat::FunctionCall>(&t_cmd);
      if (motion_func == nullptr) {  // @TODO
        return false;
      } else {
        if (motion_func->name_ == "QueueTag") {
          this->motion_queue_.emplace_back(motion_func->args_, last_function_call);
          return true;
        } else {
          last_function_call = *motion_func;
        }
      }

      return false;
    };

    // find if have queue tag, then append the previously appeared motion function
    // then find from last appeared queuetag iterator, do above, until iter = end
    for (auto begin = tmsct->data_.cmd_.begin(); begin != tmsct->data_.cmd_.end();) {
      auto const res = std::find_if(begin, tmsct->data_.cmd_.end(), queue_tag_predicate);
      if (res == tmsct->data_.cmd_.end()) {
        break;
      }

      begin = std::next(res);
    }
  }
}

void TMRTCPServer::enter_listen_node(std::string const& t_listen_node_msg) {
  auto const res = (boost::format("$TMSCT,%d,0,%s,") % (t_listen_node_msg.length() + 2) % t_listen_node_msg).str();

  auto const data = res + '*' + tmr_listener::calculate_checksum(res);
  std::cout << "[out] Sending command: " << data << " to client.\n";
  this->output_buffer_ = data + "\r\n";

  this->state_machine_->process_event(tmr_server::enter_listen_node{});
}

void TMRTCPServer::reply_question(std::string const& t_replied_msg) {
  switch (this->question_status_) {
    case QuestionStatus::ErrorType: {
      auto const num = boost::lexical_cast<int, std::string>(t_replied_msg);
      this->response_to_error_msg(num);
      break;
    }
    case QuestionStatus::NoQuestion:
      std::cerr << "No question to reply currently, value ignored.\n";
      break;
    case QuestionStatus::TMSCTErrorLine: {
      break;
    }
  }
}

void TMRTCPServer::response_to_error_msg(int const t_resp) {
  if (1 <= t_resp and t_resp <= 4) {
    auto const err_str     = "$CPERR,2,0" + std::to_string(t_resp) + ",";
    this->output_buffer_   = err_str + '*' + tmr_listener::calculate_checksum(err_str) + "\r\n";
    this->question_status_ = QuestionStatus::NoQuestion;
  } else {
    auto const t_packet = boost::get<tmr_listener::TMSCTPacket>(&this->received_packet_);
    if (t_resp == 5) {
      if (t_packet != nullptr) {  // @TODO: need other way to check validity, this line is used to remind me of it
        this->question_status_ = QuestionStatus::TMSCTErrorLine;
      } else {
        std::cerr << "[out] Invalid input, current received packet is not TMSCT packet\n";
        this->question_status_ = QuestionStatus::ErrorType;  // fallback to previous question
      }
    } else if (t_resp == 0) {
      if (t_packet != nullptr) {                                             // @TODO
        auto const data      = "$TMSCT,%d," + t_packet->data_.id_ + ",OK,";  // @TODO
        this->output_buffer_ = data + '*' + tmr_listener::calculate_checksum(data) + "\r\n";
      }

      this->question_status_ = QuestionStatus::NoQuestion;
    } else {
      std::cerr << "[out] Invalid input. Respond value should be between 0 to 5.\n";
    }
  }
}

}  // namespace tmr_server