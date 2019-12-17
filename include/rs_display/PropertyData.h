#ifndef RS_PROPERTYDATA_H
#define RS_PROPERTYDATA_H

#include <experimental/optional>
#include <string>
#include <map>
#include <iostream>

namespace rs
{

class PropertyData
{
public:
  std::experimental::optional<std::string> data;
  std::experimental::optional<std::map<std::string, std::vector<PropertyData>>> map_data;

  PropertyData operator[](const std::string& name)
  {
    if(map_data)
      if(map_data.value()[name].size())
        return map_data.value()[name][0];
      else
        return PropertyData();
    else
      return PropertyData();
  }

  PropertyData at(const std::string& name, size_t index)
  {
    if(map_data)
      if(index < map_data.value()[name].size())
        return map_data.value()[name][index];
      else
        return PropertyData();
    else
      return PropertyData();
  }

  size_t size(const std::string& name)
  {
    if(map_data)
      return map_data.value()[name].size();
    else
      return 0;
  }

  std::string value()
  {
    if(data)
      return data.value();
    else
      return "";
  }

  void print(size_t tab = 0)
  {
    if(data)
      std::cout << data.value();
    else if(map_data)
    {
      for(auto d : map_data.value())
      {
        for(auto& second : d.second)
        {
          std::cout << std::endl;
          for(size_t i = 0; i < tab; i++)
            std::cout << "\t";
          std::cout << d.first << " = ";
          second.print(tab+1);
        }
      }
    }
  }
};

};

#endif // RS_PROPERTYDATA_H
