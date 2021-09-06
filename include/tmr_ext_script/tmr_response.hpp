#ifndef TMR_RESPONSE_HPP_
#define TMR_RESPONSE_HPP_

#include <string>
#include <vector>

namespace tmr_listener {

/**
 * @brief enum class representing CPERR error
 *
 */
enum class ErrorCode { NoError, BadArgument, BadCheckSum, BadHeader, InvalidData, NotInListenNode = 0xF1 };

/**
 * @brief enum class representing the result of TMSTA SubCmd 01
 *
 */
enum class TagNumberStatus { Complete, Incomplete, NotExisted };

struct TMSCTEnterNodeMsg {
  std::string id_;
  std::string msg_;

  friend bool operator==(TMSCTEnterNodeMsg const& t_lhs, TMSCTEnterNodeMsg const& t_rhs) noexcept {
    return (t_lhs.id_ == t_rhs.id_) and (t_lhs.msg_ == t_rhs.msg_);
  }

  friend bool operator!=(TMSCTEnterNodeMsg const& t_lhs, TMSCTEnterNodeMsg const& t_rhs) noexcept {
    return !(t_lhs == t_rhs);
  }
};

struct TMSCTResponse {
  std::string id_;
  bool script_result_ = false;
  std::vector<int> abnormal_lines_{};

  friend bool operator==(TMSCTResponse const& t_lhs, TMSCTResponse const& t_rhs) noexcept {
    return (t_lhs.id_ == t_rhs.id_) and (t_lhs.script_result_ == t_rhs.script_result_) and
           (t_lhs.abnormal_lines_ == t_rhs.abnormal_lines_);
  }

  friend bool operator!=(TMSCTResponse const& t_lhs, TMSCTResponse const& t_rhs) noexcept { return !(t_lhs == t_rhs); }
};

struct TMSTAResponse {
  struct Subcmd00 {
    bool entered_ = false;
    std::string node_name_;

    friend bool operator==(Subcmd00 const& t_lhs, Subcmd00 const& t_rhs) noexcept {
      return t_lhs.entered_ == t_rhs.entered_ and t_lhs.node_name_ == t_rhs.node_name_;
    }

    friend bool operator!=(Subcmd00 const& t_lhs, Subcmd00 const& t_rhs) noexcept { return !(t_lhs == t_rhs); }
  };

  struct Subcmd01 {
    std::string tag_number_;
    TagNumberStatus tag_stat_{};

    friend bool operator==(Subcmd01 const& t_lhs, Subcmd01 const& t_rhs) noexcept {
      return t_lhs.tag_number_ == t_rhs.tag_number_ and t_lhs.tag_stat_ == t_rhs.tag_stat_;
    }

    friend bool operator!=(Subcmd01 const& t_lhs, Subcmd01 const& t_rhs) noexcept { return !(t_lhs == t_rhs); }
  };

  struct DataMsg {
    int cmd_ = 0;
    std::string data_;

    friend bool operator==(DataMsg const& t_lhs, DataMsg const& t_rhs) noexcept {
      return t_lhs.cmd_ == t_rhs.cmd_ and t_lhs.data_ == t_rhs.data_;
    }

    friend bool operator!=(DataMsg const& t_lhs, DataMsg const& t_rhs) noexcept { return !(t_lhs == t_rhs); }
  };
};

struct CPERRResponse {
  ErrorCode err_ = ErrorCode::NoError;

  friend bool operator==(CPERRResponse const& t_lhs, CPERRResponse const& t_rhs) noexcept {
    return (t_lhs.err_ == t_rhs.err_);
  }

  friend bool operator!=(CPERRResponse const& t_lhs, CPERRResponse const& t_rhs) noexcept {
    return (t_lhs.err_ != t_rhs.err_);
  }
};

}  // namespace tmr_listener

#endif