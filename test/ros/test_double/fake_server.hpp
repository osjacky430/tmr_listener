#ifndef FAKE_SERVER_HPP_
#define FAKE_SERVER_HPP_

#include <boost/asio.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/make_shared.hpp>

#include <iostream>

#include "tmr_ext_script/tmr_motion_function.hpp"

namespace tmr_listener {
namespace fake_impl {

class TMRConnection : public boost::enable_shared_from_this<TMRConnection> {
  explicit TMRConnection(boost::asio::io_service& t_io_service) : socket_{t_io_service} {}

  boost::asio::ip::tcp::socket socket_;
  boost::asio::streambuf input_buffer_;

 public:
  using this_ptr = boost::shared_ptr<TMRConnection>;

  static this_ptr create(boost::asio::io_service& t_io_service) { return this_ptr{new TMRConnection(t_io_service)}; }

  boost::asio::ip::tcp::socket& get_socket() noexcept { return this->socket_; }
  boost::asio::streambuf& get_input_buffer() noexcept { return this->input_buffer_; }

  void write(std::string& t_value) { boost::asio::write(this->socket_, boost::asio::buffer(t_value)); }

  void stop() {
    boost::system::error_code ignore_error_code;
    this->socket_.shutdown(boost::asio::ip::tcp::socket::shutdown_send, ignore_error_code);
    boost::asio::read(this->socket_, this->input_buffer_, ignore_error_code);
    this->socket_.shutdown(boost::asio::ip::tcp::socket::shutdown_receive, ignore_error_code);
    this->socket_.close(ignore_error_code);
  }
};

static constexpr struct wait_response_t {
} WAIT_FOREVER;

class TMRobotServerComm {
  bool is_connected_ = false;

  std::string write_buffer_;
  TMRConnection::this_ptr connection_;

  boost::asio::io_service io_service_;
  boost::asio::ip::tcp::endpoint server_;
  boost::asio::ip::tcp::acceptor acceptor_{io_service_};

 public:
  explicit TMRobotServerComm(unsigned short const t_port) noexcept : server_{boost::asio::ip::tcp::v4(), t_port} {}

  /**
   * @brief This function starts async accept connection request, it runs io_service in the background
   */
  void start() {
    this->acceptor_.open(this->server_.protocol());
    this->acceptor_.set_option(boost::asio::socket_base::reuse_address(true));
    this->acceptor_.bind(this->server_);
    this->acceptor_.listen();

    auto new_connection = TMRConnection::create(io_service_);
    this->acceptor_.accept(new_connection->get_socket());  // blocking

    this->connection_   = new_connection;
    this->is_connected_ = true;
  }

  /**
   * @brief This function closes the socket, and acceptor. It also stops io service and reset the socket ptr in order
   *        to stop the server fully.
   */
  void stop();

  void blocking_write(std::string);

  std::string blocking_read();
};

class ListenNodeServer {
  using Content = Expression<std::string>;

  static constexpr auto LISTENER_PORT = 5890;
  TMRobotServerComm comm_{LISTENER_PORT};

 public:
  /**
   * @brief This function writes enter-listen-node message to client and wait for its response for amount of time
   *        specified in t_tag
   *
   * @param t_str Listen node message
   * @param t_tag for tag dispatch
   * @return Response from client
   *
   * @todo Implement timeout, or take timeout variable away, just use it for tag dispatch
   */
  std::string enter_listen_node(std::string t_str, wait_response_t t_tag);

  /**
   * @brief This function writes enter-listen-node message to client, this function doesn't wait for client response
   *
   * @param t_str Listen node message
   */
  void enter_listen_node(std::string t_str);

  /**
   * @brief This function writes TMSCT OK message to the client
   *
   * @param t_id   ID of the message
   */
  void response_ok_msg(tmr_listener::ID const& t_id);

  /**
   * @brief
   */
  void response_tmsta_msg(std::string t_subcmd) noexcept;

  /**
   * @brief This function writes CPERR Error message to the client
   *
   * @param t_err   @ref tmr_listener::ErrorCode
   */
  void send_error(tmr_listener::ErrorCode t_err);

  void start() { this->comm_.start(); }

  void stop() { this->comm_.stop(); }
};

}  // namespace fake_impl
}  // namespace tmr_listener

#endif