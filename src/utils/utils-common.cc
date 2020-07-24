/*
 *    Filename: utils-common.cc
 *  Created on: Oct 28, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "utils-common.h"

namespace utils {

std::string paramToString(const std::string& name, double value) {
  std::stringstream ss;
  ss << std::left << std::setw(nameWidth) << " - " + name + ": " << std::left
     << std::setw(nameWidth) << std::to_string(value) << std::endl;
  return ss.str();
}

std::string paramToString(const std::string& name, int value) {
  std::stringstream ss;
  ss << std::left << std::setw(nameWidth) << " - " + name + ": " << std::left
     << std::setw(nameWidth) << std::to_string(value) << std::endl;
  return ss.str();
}

std::string paramToString(const std::string& name, bool value) {
  std::stringstream ss;
  ss << std::left << std::setw(nameWidth) << " - " + name + ": " << std::left
     << std::setw(nameWidth) << std::to_string(value) << std::endl;
  return ss.str();
}

std::string paramToString(const std::string& name, const std::string& value) {
  std::stringstream ss;
  ss << std::left << std::setw(nameWidth) << " - " + name + ": " << std::left
     << std::setw(nameWidth) << value << std::endl;
  return ss.str();
}

}  // namespace utils

