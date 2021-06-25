#ifndef TMR_HEADER_TAG_HPP_
#define TMR_HEADER_TAG_HPP_

#include <boost/algorithm/string/join.hpp>
#include <boost/fusion/include/adapt_struct.hpp>
#include <boost/tuple/tuple.hpp>

#include <string>
#include <type_traits>
#include <vector>

#include "tmr_ext_script/detail/tmr_command.hpp"
#include "tmr_ext_script/tmr_variable.hpp"
#include "tmr_prototype/tmr_header.hpp"
#include "tmr_utility/tmr_mt_helper.hpp"
#include "tmr_utility/tmr_parser.hpp"

namespace tmr_listener {
namespace detail {

struct TMSCTTag {
  static constexpr auto NAME() noexcept { return "$TMSCT"; }

  struct DataFormat {
    struct FunctionCall {
      std::string name_;
      std::string args_;

      static ParseRule<FunctionCall>& parsing_rule() noexcept;
    };

    struct VariableDecl {
      std::string type_;
      std::string name_;
      std::string val_;

      static ParseRule<VariableDecl>& parsing_rule() noexcept;
    };

    struct ScriptResult {
      bool result_;
      std::vector<int> abnormal_lines_;

      static ParseRule<ScriptResult>& parsing_rule() noexcept;
    };

    using ServerResponse = boost::variant<ScriptResult, std::string>;
    using ClientRequest  = std::vector<boost::variant<FunctionCall, VariableDecl>>;
    using AvailableCmd   = boost::variant<ClientRequest, ServerResponse>;
    using Data           = AvailableCmd;

    std::string id_;
    Data cmd_;

    static ParseRule<DataFormat>& parsing_rule();
  };

  struct BuildRule {
    static void check(std::vector<std::string>& t_content_holder, ScriptExit /**/) noexcept {
      t_content_holder.emplace_back("ScriptExit()");
    }

    /**
     * @brief This function accept Command<TMSCTTag> struct to be appended in the list of command
     *
     * @param t_cmd Command to append
     *
     * @note templated here because I wanted to handle the compile error myself
     */
    template <typename Tag>
    static void check(std::vector<std::string>& t_content_holder, Command<Tag> const& t_cmd) noexcept {
      static_assert(std::is_same<Tag, TMSCTTag>::value, "This command cannot be used by TMSCT");
      t_content_holder.push_back(t_cmd.to_str());
    }

    /**
     * @brief This function accept Expression struct to be appended in the list of command
     *
     * @param t_expr Expression to append
     */
    template <typename Type>
    static void check(std::vector<std::string>& t_content_holder, Expression<Type> const& t_expr) noexcept {
      t_content_holder.push_back(t_expr.to_str());
    }
  };

  /**
   * @brief MessageBuilder factory for the start of the fluent interface
   *
   * @param t_id  ID in TMSTC
   * @return an instance of the builder
   */
  static auto create_builder(ID const& t_id) noexcept {
    prototype::MessageBuilder<TMSCTTag> ret_val{};
    ret_val.result_.content_.push_back(t_id.id_);
    return ret_val;
  }

  /**
   * @brief This function simply reject all other type for create_builder by static_assert
   *
   * @tparam T  Type identified by operator<< of the prototype class
   *
   * @note The function above is NOT a specialization of this function, these two, despite having same name, are totally
   *       different thing, and not related. However, they both participate overload resolution, and the less templated
   *       function (normal function) has precedence over templated function. Thus create_builder(ID{"1"}) will call the
   *       function above instead of this. (Also, note that function can't be overloaded based on return type)
   */
  template <typename T>
  static auto create_builder(T /*unused*/) noexcept {
    static_assert(std::is_same<T, ID>::value, "TMSCT must have ID, and must be the first.");
  }

  /**
   * @brief Assemble all individual messages to a complete TMSCT message
   *
   * @param t_input List of individual message
   * @return Assembled message
   *
   * @note one line one command only, delimited by CRLF
   */
  static std::string assemble(std::vector<std::string> const& t_input) noexcept {
    auto const id   = t_input.front();
    auto const data = std::vector<std::string>{std::next(t_input.begin()), t_input.end()};
    return id + ',' + boost::algorithm::join(data, "\r\n");
  }
};

/**
 * @brief Utility class for Header usage classification
 */
struct TMSTATag {
  static constexpr auto NAME() noexcept { return "$TMSTA"; }

  struct DataFormat {
    using Subcmd00Resp = boost::tuple<bool, std::string>;
    using Subcmd01Resp = boost::tuple<std::string, TagNumberStatus>;

    using Response = boost::variant<Subcmd00Resp, Subcmd01Resp>;
    Response resp_;

    static ParseRule<TMSTATag::DataFormat>& parsing_rule();
  };

  struct BuildRule {
    template <typename T, typename U = void>
    struct check_error_msg;

    template <typename T>
    static void check(std::vector<std::string>& /**/, T /**/) noexcept {
      check_error_msg<T> display_message{};
    }
  };

  /**
   * @brief Create a builder object
   *
   * @param t_cmd
   * @return auto
   *
   * @note TMSTA doesn't support multi-command, only first will be consumed and replied
   * @todo check if TMSTA support CRLF-ed message
   */
  static auto create_builder(Command<TMSTATag> const& t_cmd) noexcept {
    prototype::MessageBuilder<TMSTATag> ret_val{};
    ret_val.result_.content_.push_back(t_cmd.to_str());
    return ret_val;
  }

  template <typename T, typename V = void>
  struct create_builder_error_msg;

  /**
   * @brief This function simply reject all other type for create_builder by static_assert
   *
   * @tparam T  Type identified by operator<< of the prototype class
   */
  template <typename T>
  static auto create_builder(T /*unused*/) noexcept {
    return create_builder_error_msg<T>{};
  }

  static std::string assemble(std::vector<std::string> const& t_input) noexcept {
    return boost::algorithm::join(t_input, ",");
  }
};

/**
 * @brief This static_assert happens when ScriptExit is appended to TMSTA message
 *
 * @tparam T  This can only be ScriptExit
 * @note I need to make it templated so that it will only be evaluated if this is called
 */
template <typename T>
struct TMSTATag::BuildRule::check_error_msg<T, std::enable_if_t<std::is_same<ScriptExit, T>::value>> {
  static_assert(not std::is_same<T, T>::value, "ScriptExit() is TMSCT only");
};

template <typename T>
struct TMSTATag::BuildRule::check_error_msg<T, std::enable_if_t<std::is_same<Command<TMSTATag>, T>::value>> {
  static_assert(not std::is_same<T, T>::value, "TMSTA only accept one command per message");
};

template <typename T>
struct TMSTATag::create_builder_error_msg<T, std::enable_if_t<std::is_same<ID, T>::value>> {
  static_assert(not std::is_same<T, ID>::value, "ID can only be used in TMSCT and TMSVR");
};

template <typename T>
struct TMSTATag::create_builder_error_msg<T, std::enable_if_t<tmr_mt_helper::is_specialization_of<T, Command>::value and
                                                              not std::is_same<Command<TMSTATag>, T>::value>> {
  static_assert(not std::is_same<T, T>::value, "Mismatched tag for TMSTA command");
};

/**
 * @brief Utility class for Header usage classification
 */
struct CPERRTag {
  static constexpr auto NAME() noexcept { return "$CPERR"; }

  struct DataFormat {
    ErrorCode err_;

    static ParseRule<DataFormat>& parsing_rule() noexcept;
  };
};

}  // namespace detail
}  // namespace tmr_listener

BOOST_FUSION_ADAPT_STRUCT(tmr_listener::detail::TMSCTTag::DataFormat, id_, cmd_)
BOOST_FUSION_ADAPT_STRUCT(tmr_listener::detail::TMSTATag::DataFormat, resp_)
BOOST_FUSION_ADAPT_STRUCT(tmr_listener::detail::CPERRTag::DataFormat, err_)

#endif