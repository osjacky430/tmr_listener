#ifndef TMR_CONSTEXPR_STRING_HPP_
#define TMR_CONSTEXPR_STRING_HPP_

namespace tmr_listener {
namespace detail {

struct ConstString {
  char const* const name_;
  std::size_t const size_;

  template <std::size_t N>
  constexpr ConstString(char const (&t_name)[N]) noexcept  // NOLINT, implicit conversion is intended, for syntax sugar
    : name_{static_cast<char const* const>(t_name)}, size_{N} {
    static_assert(N > 0, "Cannot construct empty string");
  }

  std::string to_std_str() const noexcept { return std::string{this->name_}; }

  constexpr auto size() const noexcept { return this->size_; }

  template <std::size_t N>
  constexpr bool operator==(char const (&t_name)[N]) const noexcept {
    if (N != this->size_) {
      return false;
    }

    for (std::size_t i = 0; i < N; ++i) {
      if (this->name_[i] != t_name[i]) {
        return false;
      }
    }

    return true;
  }

  constexpr bool operator==(ConstString const& t_str) const noexcept {
    if (this->size_ != t_str.size_) {
      return false;
    }

    for (std::size_t i = 0; i < this->size_; ++i) {
      if (t_str.name_[i] != this->name_[i]) {
        return false;
      }
    }

    return true;
  }
};

}  // namespace detail
}  // namespace tmr_listener

#endif