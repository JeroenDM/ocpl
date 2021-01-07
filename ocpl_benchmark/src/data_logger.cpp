#include "ocpl_benchmark/data_logger.h"

#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <vector>
#include <cassert>

namespace ocpl
{
void Logger::separate()
{
  if (line_emtpy_)
    line_emtpy_ = false;
  else
    os_ << separator_;
}

Logger::Logger(const std::string& filename)
{
  os_.open(filename);
  assert(os_.is_open());
  os_ << std::setprecision(16);
}

Logger::~Logger()
{
  // os_ << std::endl;
  os_.close();
}

void Logger::header(const std::string& header)
{
  if (line_emtpy_)
    os_ << header << "\n";
  else
    std::cerr << "Cannot write header, line already constains data\n";
}

void Logger::nextLine()
{
  os_ << "\n";
  line_emtpy_ = true;
}

void Logger::log(const std::vector<double>& v)
{
  for (auto value : v)
  {
    log(value);
  }
}

}  // namespace ocpl
