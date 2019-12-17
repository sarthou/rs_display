#include <iostream>
#include <map>
#include <mutex>

#include "ros/ros.h"

#include <robosherlock_msgs/RSObjectDescriptions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/String.h>

#include "rs_display/PropertyData.h"
#include "rs_display/Object.h"
#include "rs_display/DataReader.h"

rs::Object getObject(rs::PropertyData& obj)
{
  rs::Object object;

  object.setTrackId(std::stoi(obj["rs.annotation.Tracking"]["trackingID"].value()));

  for(size_t i = 0; i < obj["rs.annotation.SemanticColor"].size("color"); i++)
  {
    object.setColor(obj["rs.annotation.SemanticColor"].at("color",i).value(),
                    std::stof(obj["rs.annotation.SemanticColor"].at("ratio", i).value()));
  }

  for(size_t i = 0; i < obj["rs.annotation.Shape"].size("shape"); i++)
  {
    object.setShape(obj["rs.annotation.Shape"].at("shape",i).value(),
                    std::stof(obj["rs.annotation.Shape"].at("confidence",i).value()));
  }

  object.setSize(obj["rs.annotation.SemanticSize"]["size"].value(),
                 std::stof(obj["rs.annotation.SemanticSize"]["confidence"].value()));

  object.set3DPose(obj["rs.annotation.PoseAnnotation"]["world"]["rs.tf.StampedPose"]);
  object.setScale(obj["rs.annotation.Geometry"]["boundingBox"]["rs.pcl.BoundingBox3D"]);

  object.setTimestamp(std::stof(obj["timestamp"].value()));

  return std::move(object);
}

ros::Publisher* pub;
ros::Publisher* click_pub;
OntologyManipulator* onto_;
std::mutex mut_;
std::vector<rs::Object> objects_;
std::vector<rs::Object> objects_prev_;

std::vector<rs::Object> merge(std::vector<rs::Object>& current, std::vector<rs::Object>& prev, int type)
{
  std::vector<rs::Object> res;

  for(size_t i = 0; i < current.size();)
  {
    int prev_index = -1;
    float max_similarity = 0;
    for(size_t j = 0; j < prev.size(); j++)
    {
      float sim = 0;
      if(type == 0)
        sim = current[i].poseSimilarity(prev[j]);
      else
        sim = current[i].scaleSimilarity(prev[j]);

      if(sim > max_similarity)
      {
        max_similarity = sim;
        prev_index = j;
      }
    }

    if(prev_index != -1)
    {
      std::cout << "match " << type << " " << current[i].getName() << " with " << prev[prev_index].getName() << " " << max_similarity << std::endl;
      current[i].merge(prev[prev_index]);
      res.push_back(std::move(current[i]));
      prev.erase(prev.begin() + prev_index);
      current.erase(current.begin() + i);
    }
    else
      i++;
  }

  return std::move(res);
}

void Callback(const robosherlock_msgs::RSObjectDescriptions& msg)
{
  rs::DataReader reader;

  std::cout << msg.obj_descriptions.size() << std::endl;
  std::vector<rs::PropertyData> datas;
  for(auto obj : msg.obj_descriptions)
    datas.push_back(reader.get(obj));

  /*for(auto obj : datas)
    obj.print();*/

  objects_prev_ = std::move(objects_);
  std::vector<rs::Object> objects;
  for(auto obj : datas)
    objects.push_back(getObject(obj));

  std::vector<rs::Object> objects_poses = merge(objects, objects_prev_, 0);
  std::vector<rs::Object> objects_scales = merge(objects, objects_prev_, 1);

  objects.insert(objects.end(), objects_poses.begin(), objects_poses.end());
  objects.insert(objects.end(), objects_scales.begin(), objects_scales.end());

  for(auto& obj : objects)
  {
    obj.setId();
    //obj.upadteInOntology(onto_);
    pub->publish(obj.getMarker());
    pub->publish(obj.getMarkerName());
  }

  mut_.lock();
  objects_ = std::move(objects);
  if(objects_prev_.size())
  {
    std::cout << objects_prev_.size() << " not found" << std::endl;
    objects_.insert(objects_.end(), objects_prev_.begin(), objects_prev_.end());
  }
  mut_.unlock();
}

void clickCallback(const geometry_msgs::PointStamped& msg)
{
  std::string name;
  float min_size = 100000;

  mut_.lock();
  for(auto& obj : objects_)
  {
    float dist = obj.dist(msg.point.x, msg.point.y, msg.point.z);
    if(dist < min_size)
    {
      name = obj.getName();
      min_size = dist;
    }
  }
  mut_.unlock();

  std_msgs::String res_msg;
  res_msg.data = name;
  click_pub->publish(res_msg);

  std::cout << "click on " << name << std::endl;
}

void tranparentCallback(const std_msgs::String& msg)
{
  mut_.lock();
  for(auto& obj : objects_)
  {
    if(msg.data == obj.getName())
    {
      obj.setOpacity(0.5);
      break;
    }
  }
  mut_.unlock();
}

void opaqueCallback(const std_msgs::String& msg)
{
  mut_.lock();
  for(auto& obj : objects_)
  {
    if(msg.data == obj.getName())
    {
      obj.setOpacity(1.0);
      break;
    }
  }
  mut_.unlock();
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "rs_display");
  ros::NodeHandle n;

  /*OntologyManipulator onto(&n);
  onto_ = &onto;*/

  ros::Subscriber sub = n.subscribe("RoboSherlock_gsarthou/result_advertiser", 1000, Callback);
  ros::Subscriber click_sub = n.subscribe("/clicked_point", 1000, clickCallback);

  ros::Subscriber transp_sub = n.subscribe("/set_transparent", 1000, tranparentCallback);
  ros::Subscriber opaque_sub = n.subscribe("/set_opaque", 1000, opaqueCallback);

  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  pub = &marker_pub;

  ros::Publisher c_pub = n.advertise<std_msgs::String>("clicked_object", 10);
  click_pub = &c_pub;

  std::cout << "rs_display init" << std::endl;

  ros::spin();

  return 0;
}
