/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
behavior:
 - listens on "map" topic.  On each map:
   - id_of_most_recent_map = Collection.publish(map, {session ID, map name})

service calls:
 - save_map(map name) returns void
   - save the map returned by dynamic_map as map name
 */

#include <warehouse_ros/message_collection.h>
#include <warehouse_ros_mongo/database_connection.h>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <map_store/SaveMap.h>

#include <string>
#include <sstream>

#include <uuid/uuid.h>

namespace wr = warehouse_ros;

std::string session_id;
wr::MessageCollection<nav_msgs::OccupancyGrid>::Ptr map_collection;
// wr::DatabaseConnection::Ptr conn_;
// ros::ServiceClient add_metadata_service_client;
// ros::ServiceClient dynamic_map_service_client;
nav_msgs::OccupancyGrid::ConstPtr latched_map_msg;

std::string uuidGenerate()
{
  uuid_t uuid;
  uuid_generate(uuid);
  char uuid_string[37]; // UUID prints into 36 bytes + NULL terminator
  uuid_unparse_lower(uuid, uuid_string);
  return std::string(uuid_string);
}

void onMapReceived(const nav_msgs::OccupancyGrid::ConstPtr &map_msg)
{
  // ROS_DEBUG("received map");
  // std::string uuid_string = uuidGenerate();

  // wr::Metadata::Ptr metadata = map_collection->createMetadata();
  // metadata->append("uuid", uuid_string);
  // metadata->append("session_id", session_id);

  // map_collection->insert(*map_msg, metadata);

  // ROS_DEBUG("saved map");
  latched_map_msg = map_msg;
}

bool saveMap(map_store::SaveMap::Request &req,
             map_store::SaveMap::Response &res)
{
  // nav_msgs::GetMap srv;
  // if (!dynamic_map_service_client.call(srv))
  // {
  //   ROS_ERROR("Dynamic map getter service call failed");
  //   return false;
  // }

  std::string uuid_string = uuidGenerate();

  wr::Metadata::Ptr metadata = map_collection->createMetadata();
  metadata->append("uuid", uuid_string);
  metadata->append("session_id", session_id);
  metadata->append("name", req.map_name);

  ROS_INFO("Save map %d by %d @ %f as %s", latched_map_msg->info.width,
           latched_map_msg->info.height, latched_map_msg->info.resolution, req.map_name.c_str());
  map_collection->insert(*latched_map_msg, metadata);

  // ROS_DEBUG("nameLastestMaps() service call done");
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_saver");
  std::shared_ptr<ros::NodeHandle> nh = std::make_shared<ros::NodeHandle>("~");

  // Use the current ROS time in seconds as the session id.
  char buff[256];
  snprintf(buff, 256, "%f", ros::Time::now().toSec());
  session_id = std::string(buff);
  warehouse_ros_mongo::MongoDatabaseConnection conn_;

  std::string host;
  int port;
  nh->param<std::string>("warehouse_host", host, "localhost");
  nh->param<int>("warehouse_port", port, 27017);
  conn_.setParams(host, port, 60.0);
  ROS_INFO("[map_saver] Connecting to warehouse_ros_mongo...");
  conn_.connect();
  ROS_INFO("[map_saver] Connected.");
  ROS_INFO("[map_saver] Opening collection...");
  map_collection = conn_.openCollectionPtr<nav_msgs::OccupancyGrid>("map_store", "maps");
  ROS_INFO("[map_saver] map_store collection opened.");

  ros::Subscriber map_subscriber = nh->subscribe("map", 1, onMapReceived);
  ros::ServiceServer map_saver_service = nh->advertiseService("save_map", saveMap);

  // dynamic_map_service_client = nh->serviceClient<nav_msgs::GetMap>("dynamic_map");

  ROS_DEBUG("spinning.");

  ros::spin();
  return 0;
}
