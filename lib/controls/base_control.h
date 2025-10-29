#pragma once
#include <string>
#include <vector>
#include <utility>   // for std::pair
#include <algorithm>

class BaseControl {
public:
  using StatePair = std::pair<std::string, std::string>; // {name, value}
  using StateVector = std::vector<StatePair>;

  BaseControl(std::string id,
              std::vector<std::string> cmd_topics = {},
              std::vector<std::string> state_topics = {})
    : id_(std::move(id)),
      cmd_topics_(std::move(cmd_topics)),
      state_topics_(std::move(state_topics)) {}

  virtual ~BaseControl() = default;

  // ---- Identity & Topics ----
  const std::string& id() const { return id_; }
  const std::vector<std::string>& cmd_topics()   const { return cmd_topics_; }
  const std::vector<std::string>& state_topics() const { return state_topics_; }

  void add_cmd_topic(const std::string& t)   { cmd_topics_.push_back(t); }
  void add_state_topic(const std::string& t) { state_topics_.push_back(t); }

  // ---- State interface ----
  // Accepts a batch update (vector of name/value pairs).
  // Returns true if at least one state changed.
  virtual bool update_state(const StateVector& new_states) = 0;

  // Returns all current state name/value pairs.
  virtual StateVector get_state() const = 0;

protected:
  std::string id_;
  std::vector<std::string> cmd_topics_;
  std::vector<std::string> state_topics_;
};