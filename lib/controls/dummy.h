#pragma once
#include "base_control.h"
#include <cctype>
#include <algorithm>

class DummySwitch : public BaseControl {
public:
  explicit DummySwitch(std::string id,
                       bool initial = false,
                       std::vector<std::string> cmd_topics = {},
                       std::vector<std::string> state_topics = {})
    : BaseControl(std::move(id), std::move(cmd_topics), std::move(state_topics)),
      state_(initial) {}

  bool update_state(const StateVector& new_states) override {
    bool changed = false;
    for (const auto& state : new_states) {
      const std::string& name = state.first;
      const std::string& value = state.second;
      if (to_lower_copy(name) == "state") {
        bool new_state = parse_bool(value, state_);
        if (new_state != state_) {
          state_ = new_state;
          changed = true;
        }
      }
    }
    return changed;
  }

  StateVector get_state() const override {
    return { {"state", state_ ? "ON" : "OFF"} };
  }

  bool get_state_bool() const { return state_; }

private:
  static std::string to_lower_copy(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
    return s;
  }

  static bool parse_bool(std::string value, bool current) {
    auto v = to_lower_copy(value);
    if (v == "on" || v == "1" || v == "true")  return true;
    if (v == "off" || v == "0" || v == "false") return false;
    if (v == "toggle") return !current;
    return current; // unknown string -> no change
  }

  bool state_;
};