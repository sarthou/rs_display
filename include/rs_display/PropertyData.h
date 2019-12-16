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
  std::experimental::optional<std::map<std::string, PropertyData>> map_data;

  PropertyData operator[](std::string name)
  {
    if(map_data)
      return map_data.value()[name];
    else
      return PropertyData();
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
        std::cout << std::endl;
        for(size_t i = 0; i < tab; i++)
          std::cout << "\t";
        std::cout << d.first << " = ";
        d.second.print(tab+1);
      }
    }
  }
};

};

#endif // RS_PROPERTYDATA_H
