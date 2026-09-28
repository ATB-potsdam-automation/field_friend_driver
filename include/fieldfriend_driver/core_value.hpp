// Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
#pragma once

#include <map>
#include <string>

namespace fieldfriend_driver
{

/// Holds one value read from the "core" serial message (bool, int or double).
class CoreValue
{
public:
  CoreValue() = default;

  static CoreValue from_bool(bool value)
  {
    CoreValue v;
    v.type_ = Type::Bool;
    v.bool_value_ = value;
    return v;
  }

  static CoreValue from_int(int value)
  {
    CoreValue v;
    v.type_ = Type::Int;
    v.int_value_ = value;
    return v;
  }

  static CoreValue from_double(double value)
  {
    CoreValue v;
    v.type_ = Type::Double;
    v.double_value_ = value;
    return v;
  }

  bool as_bool() const
  {
    return type_ == Type::Bool ? bool_value_ : as_double() > 0.5;
  }

  int as_int() const
  {
    return type_ == Type::Int ? int_value_ : static_cast<int>(as_double());
  }

  double as_double() const
  {
    switch (type_) {
      case Type::Bool:
        return bool_value_ ? 1.0 : 0.0;
      case Type::Int:
        return static_cast<double>(int_value_);
      case Type::Double:
      default:
        return double_value_;
    }
  }

private:
  enum class Type
  {
    Bool,
    Int,
    Double
  };

  Type type_ = Type::Double;
  bool bool_value_ = false;
  int int_value_ = 0;
  double double_value_ = 0.0;
};

using CoreData = std::map<std::string, CoreValue>;

}  // namespace fieldfriend_driver
