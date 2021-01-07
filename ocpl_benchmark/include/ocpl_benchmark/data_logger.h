#pragma once

#include <fstream>
#include <string>
#include <vector>

namespace ocpl
{
class Logger
{
  std::ofstream os_;
  bool line_emtpy_{ true };
  const char separator_{ ',' };

  /** Add a separator the the stream, except when the line is still empty. **/
  void separate();

public:
  Logger(const std::string& filename);
  ~Logger();

  // a logger can't be copied
  Logger(Logger const&) = delete;
  Logger& operator=(Logger const&) = delete;

  /** Write a single string to an empty line, useful to add an header at the start. **/
  void header(const std::string& header);

  /** Start logging on a new line. **/
  void nextLine();

  void log(const std::vector<double>& v);

  /** Allow logging of any type that can be written to the stream.
   *
   * Defined here, as templates in shared libraries do not work, or do they?
   * **/
  template <class T>
  void log(T value)
  {
    separate();
    os_ << value;
  }
};
}  // namespace ocpl
