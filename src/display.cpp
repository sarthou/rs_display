#include <iostream>
#include <map>
#include <experimental/optional>
#include "ros/ros.h"

#include <robosherlock_msgs/RSObjectDescriptions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

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
};

size_t getInBraquet(size_t begin, std::string& in_bracket, std::string& text, char left, char right)
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

bool split(const std::string &text, std::vector<std::string> &strs, const std::string& delim)
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

std::string getProperty(std::string& str, size_t& pose)
{
  size_t pose_start = str.find("\"", pose) + 1;
  size_t pose_end = str.find("\"", pose_start + 1);
  std::string property = str.substr(pose_start, pose_end - pose_start);
  pose = pose_end + 1;
  return property;
}

std::string getData(std::string& str, size_t& pose)
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

std::map<std::string, PropertyData> processObject(std::string& str)
{
  std::map<std::string, PropertyData> res;
  str = str.substr(1, str.size()-2);
  //
  size_t pose = 0;
  while(pose!= std::string::npos)
  {
    std::string property = getProperty(str, pose);
    std::string data = getData(str, pose);
    if(data[0] == '{')
      res[property].map_data = processObject(data);
    else
      res[property].data = data;
  }
  return res;
}

void print(std::map<std::string, PropertyData>& obj, size_t tab = 0)
{
  for(auto prop : obj)
  {
    for(size_t i = 0; i < tab; i++)
      std::cout << "\t";
    std::cout << prop.first << " = ";
    if(prop.second.data)
      std::cout << prop.second.data.value() << std::endl;
    else if(prop.second.map_data)
    {
      std::cout << std::endl;
      print(prop.second.map_data.value(), tab+1);
    }
  }
}

std::vector<float> toVect(const std::string& str)
{
  std::vector<std::string> str_vect;
  std::vector<float> flt_vect;
  split(str, str_vect, ",");
  for(auto it : str_vect)
    flt_vect.push_back(std::stof(it));
  return flt_vect;
}

std::vector<float> toColor(const std::string& str)
{
  if(str == "yellow")
    return {1.0, 1.0, 0};
  else if(str == "blue")
    return {0, 0, 1.0};
  else if(str == "black")
    return {0, 0, 0};
  else if(str == "white")
    return {1.0, 1.0, 1.0};
  else if(str == "red")
    return {1.0, 0, 0};
  else if(str == "green")
    return {0, 1.0, 0};
  else if(str == "cyan")
    return {0, 1.0, 1.0};
  else if(str == "magenta")
    return {1.0, 0, 1.0};
  else
    return {0.5, 0.5, 0.5};
}

visualization_msgs::Marker getMarker(std::map<std::string, PropertyData>& obj)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = obj["rs.annotation.PoseAnnotation"]["world"]["rs.tf.StampedPose"]["frame"].value();
  marker.header.stamp = ros::Time(std::stof(obj["timestamp"].value()));
  marker.ns = "rs";
  marker.id = std::stoi(obj["rs.annotation.Tracking"]["trackingID"].value());
  if(obj["rs.annotation.Shape"]["shape"].value() == "box")
    marker.type = visualization_msgs::Marker::CUBE;
  else
    marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;

  auto pose = toVect(obj["rs.annotation.PoseAnnotation"]["world"]["rs.tf.StampedPose"]["translation"].value());
  marker.pose.position.x = pose[0];
  marker.pose.position.y = pose[1];
  marker.pose.position.z = pose[2];

  auto rot = toVect(obj["rs.annotation.PoseAnnotation"]["world"]["rs.tf.StampedPose"]["rotation"].value());
  marker.pose.orientation.x = rot[0];
  marker.pose.orientation.y = rot[1];
  marker.pose.orientation.z = rot[2];
  marker.pose.orientation.w = rot[3];

  marker.scale.x = std::stof(obj["rs.annotation.Geometry"]["boundingBox"]["rs.pcl.BoundingBox3D"]["width"].value());
  marker.scale.y = std::stof(obj["rs.annotation.Geometry"]["boundingBox"]["rs.pcl.BoundingBox3D"]["height"].value());
  marker.scale.z = std::stof(obj["rs.annotation.Geometry"]["boundingBox"]["rs.pcl.BoundingBox3D"]["depth"].value());

  auto color = toColor(obj["rs.annotation.SemanticColor"]["color"].value());
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration(2);

  return marker;
}

ros::Publisher* pub;

void Callback(const robosherlock_msgs::RSObjectDescriptions& msg)
{
  std::cout << msg.obj_descriptions.size() << std::endl;
  std::vector<std::map<std::string, PropertyData>> objects;
  for(auto obj : msg.obj_descriptions)
    objects.push_back(processObject(obj));

  for(auto obj : objects)
    print(obj);

  for(auto obj : objects)
    pub->publish(getMarker(obj));
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "rs_display");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("RoboSherlock_gsarthou/result_advertiser", 1000, Callback);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  pub = &marker_pub;

  ros::spin();

  return 0;
}
