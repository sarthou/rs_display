#ifndef RS_DATAREADER_H
#define RS_DATAREADER_H

#include <string>
#include <map>
#include <vector>

#include "rs_display/PropertyData.h"

namespace rs
{

class DataReader
{
public:
  PropertyData get(std::string& str);

private:
  std::map<std::string, std::vector<rs::PropertyData>> processObject(std::string& str);

  size_t getInBraquet(size_t begin, std::string& in_bracket, std::string& text, char left, char right);
  bool split(const std::string &text, std::vector<std::string> &strs, const std::string& delim);

  std::string getProperty(std::string& str, size_t& pose);
  std::string getData(std::string& str, size_t& pose);
};

} // namespace rs

#endif // RS_DATAREADER_H
