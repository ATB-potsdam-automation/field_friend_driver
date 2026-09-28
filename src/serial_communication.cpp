// Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
#include "fieldfriend_driver/serial_communication.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <iterator>
#include <sstream>
#include <thread>

namespace fieldfriend_driver
{

namespace
{
constexpr const char * kDevicePath = "/dev/esp";

/// Run a shell command and return its combined stdout/stderr output.
std::string run_command(const std::string & command)
{
  std::string full_command = command + " 2>&1";
  std::array<char, 256> buffer{};
  std::string output;
  FILE * pipe = popen(full_command.c_str(), "r");
  if (pipe == nullptr) {
    return output;
  }
  while (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
    output += buffer.data();
  }
  pclose(pipe);
  return output;
}
}  // namespace

SerialCommunication::SerialCommunication(rclcpp::Node * node)
: logger_(node->get_logger())
{
  RCLCPP_INFO(logger_, "Init serial communication");

  node->declare_parameter("flashing_arguments", std::string(""));
  flashing_arguments_ = node->get_parameter("flashing_arguments").as_string();

  node->declare_parameter("sleep_after_flash", 0);
  sleep_after_flash_ = static_cast<int>(node->get_parameter("sleep_after_flash").as_int());

  open_port();
  init_core_data(node);
}

SerialCommunication::~SerialCommunication()
{
  if (fd_ >= 0) {
    ::close(fd_);
  }
}

void SerialCommunication::init_core_data(rclcpp::Node * node)
{
  node->declare_parameter("read_data.list", rclcpp::PARAMETER_STRING_ARRAY);
  const auto data_list = node->get_parameter("read_data.list").as_string_array();

  {
    std::ostringstream oss;
    for (const auto & entry : data_list) {
      oss << entry << " ";
    }
    RCLCPP_INFO(logger_, "Received list of strings: [%s]", oss.str().c_str());
  }

  std::size_t pos = 0;
  for (const auto & name : data_list) {
    const std::string type_param = "read_data." + name + ".type";
    node->declare_parameter(type_param, rclcpp::PARAMETER_STRING);
    const std::string type_str = node->get_parameter(type_param).as_string();

    const std::string default_param = "read_data." + name + ".default";
    CoreValue default_value;
    if (type_str == "bool") {
      node->declare_parameter(default_param, rclcpp::PARAMETER_BOOL);
      default_value = CoreValue::from_bool(node->get_parameter(default_param).as_bool());
    } else if (type_str == "int") {
      node->declare_parameter(default_param, rclcpp::PARAMETER_INTEGER);
      default_value = CoreValue::from_int(
        static_cast<int>(node->get_parameter(default_param).as_int()));
    } else if (type_str == "double") {
      node->declare_parameter(default_param, rclcpp::PARAMETER_DOUBLE);
      default_value = CoreValue::from_double(node->get_parameter(default_param).as_double());
    }

    core_data_list_.push_back(CoreDataDef{name, type_str, pos, default_value});
    ++pos;
  }

  for (const auto & def : core_data_list_) {
    core_data_[def.name] = def.default_value;
  }
}

void SerialCommunication::enable()
{
  const std::string command = "/root/.lizard/espresso.py enable " + flashing_arguments_;
  RCLCPP_INFO(logger_, "Enable esp with the following command: %s", command.c_str());
  const std::string output = run_command(command);
  RCLCPP_INFO(logger_, "ESP output:\n%s", output.c_str());
  if (sleep_after_flash_ > 0) {
    std::this_thread::sleep_for(std::chrono::seconds(sleep_after_flash_));
  }
  RCLCPP_INFO(logger_, "Esp is now enabled");
}

void SerialCommunication::open_port()
{
  enable();

  fd_ = ::open(kDevicePath, O_RDWR | O_NOCTTY);
  if (fd_ < 0) {
    RCLCPP_ERROR(logger_, "Could not open serial communication!");
    return;
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    RCLCPP_ERROR(logger_, "Could not open serial communication!");
    ::close(fd_);
    fd_ = -1;
    return;
  }

  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);
  cfmakeraw(&tty);
  // Non-blocking reads: return immediately with whatever bytes are available.
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(logger_, "Could not open serial communication!");
    ::close(fd_);
    fd_ = -1;
  }
}

int SerialCommunication::calculate_checksum(const std::string & line)
{
  int checksum = 0;
  for (unsigned char c : line) {
    checksum ^= c;
  }
  return checksum;
}

std::string SerialCommunication::append_checksum(const std::string & line)
{
  char hex[3];
  std::snprintf(hex, sizeof(hex), "%02x", calculate_checksum(line) & 0xff);
  return line + "@" + hex + "\n";
}

bool SerialCommunication::validate_checksum(const std::string & line)
{
  const auto at_pos = line.find('@');
  if (at_pos == std::string::npos) {
    return false;
  }
  const std::string data = line.substr(0, at_pos);
  const std::string checksum_hex = line.substr(at_pos + 1);
  int checksum = 0;
  try {
    checksum = std::stoi(checksum_hex, nullptr, 16);
  } catch (const std::exception &) {
    return false;
  }
  return calculate_checksum(data) == checksum;
}

void SerialCommunication::send(const std::string & line)
{
  if (fd_ < 0) {
    RCLCPP_WARN(logger_, "No Port open");
    return;
  }
  const std::string full_line = append_checksum(line);
  std::lock_guard<std::mutex> lock(mutex_);
  const ssize_t written = ::write(fd_, full_line.data(), full_line.size());
  (void)written;
}

void SerialCommunication::handle_core_message(const std::vector<std::string> & words)
{
  // words[0] is the "core" tag, actual values start at index 1.
  for (const auto & def : core_data_list_) {
    const std::size_t index = def.pos + 1;
    if (index >= words.size()) {
      return;
    }
    const std::string & value = words[index];
    if (def.type == "bool") {
      bool parsed;
      if (value == "true") {
        parsed = true;
      } else if (value == "false") {
        parsed = false;
      } else {
        parsed = std::stod(value) > 0.5;
      }
      core_data_[def.name] = CoreValue::from_bool(parsed);
    } else if (def.type == "int") {
      core_data_[def.name] = CoreValue::from_int(std::stoi(value));
    } else if (def.type == "double") {
      core_data_[def.name] = CoreValue::from_double(std::stod(value));
    } else {
      return;
    }
  }
  notify_core_observers(core_data_);
}

void SerialCommunication::handle_expander_message(const std::vector<std::string> & words)
{
  if (words.size() < 2) {
    return;
  }
  if (words[1] == "bms") {
    notify_bms_observers(std::vector<std::string>(words.begin() + 2, words.end()));
  }
}

void SerialCommunication::read()
{
  if (fd_ < 0) {
    RCLCPP_WARN(logger_, "No Port open");
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    char chunk[4096];
    ssize_t n;
    while ((n = ::read(fd_, chunk, sizeof(chunk))) > 0) {
      buffer_.append(chunk, static_cast<std::size_t>(n));
    }
  }

  std::size_t newline_pos;
  while ((newline_pos = buffer_.find('\n')) != std::string::npos) {
    std::string line = buffer_.substr(0, newline_pos);
    buffer_.erase(0, newline_pos + 1);

    // rstrip
    while (!line.empty() && std::isspace(static_cast<unsigned char>(line.back()))) {
      line.pop_back();
    }

    if (line.size() >= 3 && line[line.size() - 3] == '@' &&
      std::count(line.begin(), line.end(), '@') == 1)
    {
      if (!validate_checksum(line)) {
        // Matches the original implementation: an invalid checksum aborts
        // processing of the rest of the currently buffered lines too.
        return;
      }
      line = line.substr(0, line.size() - 3);
    }

    std::istringstream iss(line);
    std::vector<std::string> words{std::istream_iterator<std::string>{iss},
      std::istream_iterator<std::string>{}};

    if (words.empty()) {
      return;
    }

    try {
      if (words[0] == "core") {
        handle_core_message(words);
      } else if (words[0] == "expander:" || words[0] == "p0:") {
        handle_expander_message(words);
      } else if (words[0] == "error") {
        RCLCPP_DEBUG(logger_, "Error on serial line: %s", line.c_str());
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        logger_, "General exception in the following line: %s (%s)", line.c_str(), e.what());
    }
  }
}

}  // namespace fieldfriend_driver
