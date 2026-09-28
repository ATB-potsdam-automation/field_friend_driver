// Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
#pragma once

#include <string>
#include <vector>

#include "fieldfriend_driver/core_value.hpp"

namespace fieldfriend_driver
{

/// Observer notified whenever a "core" serial message has been parsed.
class CoreObserver
{
public:
  virtual ~CoreObserver() = default;
  virtual void update(const CoreData & data) = 0;
};

/// Observer notified whenever a "bms" expander serial message has been received.
class BmsObserver
{
public:
  virtual ~BmsObserver() = default;
  virtual void update(const std::vector<std::string> & words) = 0;
};

/// Implements the communication interface using the observer pattern.
class Communication
{
public:
  virtual ~Communication() = default;

  void register_core_observer(CoreObserver * observer)
  {
    core_observers_.push_back(observer);
  }

  void notify_core_observers(const CoreData & data)
  {
    for (auto * observer : core_observers_) {
      observer->update(data);
    }
  }

  void register_bms_observer(BmsObserver * observer)
  {
    bms_observers_.push_back(observer);
  }

  void notify_bms_observers(const std::vector<std::string> & words)
  {
    for (auto * observer : bms_observers_) {
      observer->update(words);
    }
  }

  virtual void send(const std::string & line) = 0;
  virtual void read() = 0;

private:
  std::vector<CoreObserver *> core_observers_;
  std::vector<BmsObserver *> bms_observers_;
};

}  // namespace fieldfriend_driver
