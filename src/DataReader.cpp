#include "rs_display/DataReader.h"

namespace rs
{

PropertyData DataReader::get(std::string& str)
{
  rs::PropertyData res;

  res.map_data = processObject(str);

  return res;
}

std::map<std::string, std::vector<rs::PropertyData>> DataReader::processObject(std::string& str)
{
  std::map<std::string, std::vector<rs::PropertyData>> res;
  std::vector<std::string> strs;
  size_t end_pose = 0;
  do
  {
    std::string local_str;
    end_pose = getInBraquet(end_pose, local_str, str, '{', '}');
    if(local_str != "")
      strs.push_back(local_str);
    end_pose++;
  }
  while(end_pose != str.size());

  for(auto& s : strs)
  {
    size_t pose = 0;
    while(pose!= std::string::npos)
    {
      std::string property = getProperty(s, pose);
      std::string data = getData(s, pose);
      rs::PropertyData sub_res;
      if(data[0] == '{')
        sub_res.map_data = processObject(data);
      else
        sub_res.data = data;
      res[property].push_back(sub_res);
    }
  }

  return res;
}

size_t DataReader::getInBraquet(size_t begin, std::string& in_bracket, std::string& text, char left, char right)
{
  size_t bracket = begin;

  if(text[bracket] == left)
  {
    size_t first_bracket = bracket;
    int cpt = 1;
    while((cpt != 0) && (bracket+1 < text.length()))
    {
      ++bracket;
      if(text[bracket] == left)
        cpt++;
      else if(text[bracket] == right)
        cpt--;

    }

    in_bracket = text.substr(first_bracket+1, bracket-first_bracket-1);

    if(cpt == 0)
      return bracket;
    else
      return std::string::npos;
  }
  else
    return begin;
}

bool DataReader::split(const std::string &text, std::vector<std::string> &strs, const std::string& delim)
{
  std::string tmp_text = text;
  while(tmp_text.find(delim) != std::string::npos)
  {
    size_t pos = tmp_text.find(delim);
    std::string part = tmp_text.substr(0, pos);
    tmp_text = tmp_text.substr(pos + delim.size(), tmp_text.size() - pos - delim.size());
    if(part != "")
      strs.push_back(part);
  }
  strs.push_back(tmp_text);
  if(strs.size() > 1)
    return true;
  else
    return false;
}

std::string DataReader::getProperty(std::string& str, size_t& pose)
{
  size_t pose_start = str.find("\"", pose) + 1;
  size_t pose_end = str.find("\"", pose_start + 1);
  std::string property = str.substr(pose_start, pose_end - pose_start);
  pose = pose_end + 1;
  return property;
}

std::string DataReader::getData(std::string& str, size_t& pose)
{
  std::string data = "";
  size_t after_dote_pose = str.find(":", pose) + 1;
  if(str[after_dote_pose] == '\"')
  {
    after_dote_pose++;
    size_t pose_end = str.find("\"", after_dote_pose);
    if(pose_end != std::string::npos)
    {
      data = str.substr(after_dote_pose, pose_end - after_dote_pose);
      pose = pose_end + 1;
    }
    else
      pose = after_dote_pose + 2;
  }
  else if(str[after_dote_pose] == '[')
  {
    size_t pose_end = getInBraquet(after_dote_pose, data, str, '[', ']');
    pose = pose_end + 1;
  }
  else if(str[after_dote_pose] == '{')
  {
    size_t pose_end = getInBraquet(after_dote_pose, data, str, '{', '}');
    data = "{" + data + "}";
    pose = pose_end + 1;
  }
  else
  {
    size_t pose_end = str.find(",", after_dote_pose + 1);
    size_t pose_end_braq = str.find("}", after_dote_pose + 1);
    if(pose_end_braq < pose_end)
      pose_end = pose_end_braq;
    data = str.substr(after_dote_pose, pose_end - after_dote_pose);
    pose = pose_end;
  }

  pose = str.find(",", pose);
  if(pose != std::string::npos)
    pose++;

  return data;
}

} // namespace rs
