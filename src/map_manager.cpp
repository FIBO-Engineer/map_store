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
 - sets up connection to warehouse
 - tells warehouse to publish latest map of any session (or default map?  or nothing?)
 - spins, handling service calls

service calls:
 - list_maps() returns list of map metadata: {id, name, timestamp, maybe thumbnail}
   - query for all maps.
 - delete_map(map id) returns void
   - Deletes the given map
 - rename_map[(map id, name) returns void
   - renames a given map
 - publish_map(map id) returns void
   - queries warehouse for map of given id
   - publishes the map on /map
   - sets dynamic map up to load it\
 - dynamic_map() returns nav_msgs/OccupancyGrid
   - returns the dynamic map
 */

#include <warehouse_ros/message_collection.h>
#include <warehouse_ros_mongo/database_connection.h>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_store/ListMaps.h>
#include <map_store/PublishMap.h>
#include <map_store/DeleteMap.h>
#include <map_store/RenameMap.h>
#include <map_store/SetOrigin.h>
#include <map_store/MapListEntry.h>
#include <nav_msgs/GetMap.h>

#include <std_msgs/Header.h>

#include <string>
#include <sstream>
#include <exception>
namespace mr = warehouse_ros;

mr::DatabaseConnection::Ptr conn_;
mr::MessageCollection<nav_msgs::OccupancyGrid>::Ptr map_collection;
ros::Publisher map_publisher;
std::string last_map;

typedef std::vector<mr::MessageWithMetadata<nav_msgs::OccupancyGrid>::ConstPtr> MapVector;

bool listMaps(map_store::ListMaps::Request &request,
              map_store::ListMaps::Response &response)
{
  ROS_DEBUG("listMaps() service call");

  MapVector all_maps;

  mr::Query::Ptr q = map_collection->createQuery();
  all_maps = map_collection->queryList(q, true, "creation_time", false);

  // Loop over all_maps to get the first of each session.
  for (MapVector::const_iterator map_iter = all_maps.begin(); map_iter != all_maps.end(); map_iter++)
  {
    ROS_DEBUG("listMaps() reading a map");

    ROS_DEBUG("listMaps() adding a map to the result list.");
    // ROS_DEBUG("listMaps() metadata is: '%s'", (*map_iter)->metadata_.toString().c_str());

    // Add the map info to our result list.
    map_store::MapListEntry new_entry;
    new_entry.name = (*map_iter)->lookupString("name");
    new_entry.date = (int64_t)(*map_iter)->lookupDouble("creation_time");
    new_entry.session_id = (*map_iter)->lookupString("session_id");
    new_entry.map_id = (*map_iter)->lookupString("uuid");

    response.map_list.push_back(new_entry);
  }

  ROS_DEBUG("listMaps() service call done");
  return true;
}

bool lookupMap(std::string name, nav_msgs::OccupancyGridConstPtr &ptr)
{
  MapVector matching_maps;
  try
  {
    mr::Query::Ptr q = map_collection->createQuery();
    q->append("uuid", name);
    matching_maps = map_collection->queryList(q, false);
  }
  catch (const std::exception &e)
  {
    ROS_ERROR("Error during query: %s", e.what());
    return false;
  }

  if (matching_maps.size() != 1)
  {
    ROS_ERROR("publishMap() found %d matching maps instead of 1.  Failing.", (int)matching_maps.size());
    return false;
  }
  ptr = nav_msgs::OccupancyGridConstPtr(matching_maps[0]);
  return true;
}

bool publishMap(map_store::PublishMap::Request &request,
                map_store::PublishMap::Response &response)
{
  ROS_DEBUG("publishMap() service call");
  ROS_DEBUG("Searching for '%s'", request.map_id.c_str());

  last_map = request.map_id;
  ros::NodeHandle nh;

  std::string frame_id;
  if (!nh.param<std::string>("map_frame_id", frame_id, "/map"))
  {
    ROS_WARN("Parameter 'map_frame_id' not set. Using default frame ID: '/map'");
  }

  nh.setParam("last_map_id", last_map);
  nav_msgs::OccupancyGridConstPtr map;
  if (lookupMap(request.map_id, map))
  {
    try
    {
      nav_msgs::OccupancyGrid prefixedMap = *map;
      prefixedMap.header.frame_id = frame_id;

      map_publisher.publish(prefixedMap);
    }
    catch (...)
    {
      ROS_ERROR("Error publishing map");
    }
  }
  else
  {
    return false;
  }

  return true;
}

bool deleteMap(map_store::DeleteMap::Request &request,
               map_store::DeleteMap::Response &response)
{
  ros::NodeHandle nh;
  std::string param;
  if (nh.getParam("last_map_id", param))
  {
    if (param == request.map_id)
    {
      nh.deleteParam("last_map_id");
    }
  }
  if (last_map == request.map_id)
  {
    last_map = "";
  }

  mr::Query::Ptr q = map_collection->createQuery();
  q->append("uuid", request.map_id);
  return map_collection->removeMessages(q) == 1;
}

bool renameMap(map_store::RenameMap::Request &request,
               map_store::RenameMap::Response &response)
{
  mr::Query::Ptr q = map_collection->createQuery();
  q->append("uuid", request.map_id);

  mr::Metadata::Ptr metadata = map_collection->createMetadata();
  metadata->append("name", request.new_name);

  map_collection->modifyMetadata(q, metadata);
  return true;
}

bool getMap(nav_msgs::GetMap::Request &request,
            nav_msgs::GetMap::Response &response)
{
  if (last_map == "")
  {
    return false;
  }
  nav_msgs::OccupancyGridConstPtr map;
  if (lookupMap(last_map, map))
  {
    response.map = *map;
  }
  else
  {
    return false;
  }
  return true;
}

bool setOrigin(map_store::SetOrigin::Request &request,
               map_store::SetOrigin::Response &response)
{
  // TODO: this is a long-winded method to set the origin of the map. Fix it later.

  ROS_DEBUG("Searching for '%s'", request.map_id.c_str());

  nav_msgs::OccupancyGridConstPtr map;
  nav_msgs::OccupancyGrid offsetMap;

  // Create new map with specified offset:
  if (lookupMap(request.map_id, map))
  {
    offsetMap = *map;

    offsetMap.info.origin.position.x = request.pos_x;
    offsetMap.info.origin.position.y = request.pos_y;

    offsetMap.info.origin.orientation.x = request.rot_x;
    offsetMap.info.origin.orientation.y = request.rot_y;
    offsetMap.info.origin.orientation.z = request.rot_z;
    offsetMap.info.origin.orientation.w = request.rot_w;
  }
  else
  {
    ROS_ERROR("Map ID specified for origin offset was not found in the database");
    return false;
  }

  // Retrieve and copy metadata from the old map:
  MapVector all_maps;
  map_store::MapListEntry new_offset_entry;

  mr::Query::Ptr q = map_collection->createQuery();
  all_maps = map_collection->queryList(q, true, "creation_time", false);
  bool wasMapFound = false;

  for (MapVector::const_iterator map_iter = all_maps.begin(); map_iter != all_maps.end() && !wasMapFound; map_iter++)
  {
    if ((*map_iter)->lookupString("uuid").compare(request.map_id) == 0)
    {
      new_offset_entry.name = (*map_iter)->lookupString("name");
      new_offset_entry.date = (int64_t)(*map_iter)->lookupDouble("creation_time");
      new_offset_entry.session_id = (*map_iter)->lookupString("session_id");
      new_offset_entry.map_id = (*map_iter)->lookupString("uuid");

      wasMapFound = true;
    }
  }

  if (!wasMapFound)
  {
    ROS_ERROR("The desired map (for setting the origin) was not found in the database");
    return false;
  }

  // Delete old map with old origin:
  mr::Query::Ptr q2 = map_collection->createQuery();
  q2->append("uuid", request.map_id);
  if (map_collection->removeMessages(q2) != 1)
  {
    ROS_ERROR("FAILED: Could not remove previous origin settings");
    return false;
  }

  // Add new map with updated origin (maintain the same metadata from the previous map):
  mr::Metadata::Ptr metadata = map_collection->createMetadata();
  metadata->append("uuid", new_offset_entry.map_id);
  metadata->append("session_id", new_offset_entry.session_id);
  metadata->append("name", new_offset_entry.name);

  map_collection->insert(offsetMap, metadata);

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_manager");
  ros::NodeHandle nh;

  warehouse_ros_mongo::MongoDatabaseConnection conn_;
  std::string host;
  int port;
  nh.param<std::string>("warehouse_host", host, "localhost");
  nh.param<int>("warehouse_port", port, 27017);
  conn_.setParams(host, port, 60.0);
  ROS_INFO("[map_manager] Connecting to warehouse_ros_mongo...");
  conn_.connect();
  ROS_INFO("[map_manager] Connected.");
  ROS_INFO("[map_manager] Opening collection...");
  map_collection = conn_.openCollectionPtr<nav_msgs::OccupancyGrid>("map_store", "maps");
  ROS_INFO("[map_manager] map_store collection opened.");

  // map_collection->ensureIndex("uuid");

  if (!nh.getParam("last_map_id", last_map))
  {
    last_map = "";
  }

  map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  if (last_map != "")
  {
    nav_msgs::OccupancyGridConstPtr map;
    if (lookupMap(last_map, map))
    {
      try
      {
        map_publisher.publish(map);
      }
      catch (...)
      {
        ROS_ERROR("Error publishing map");
      }
    }
    else
    {
      ROS_ERROR("Invalid last_map_id");
    }
  }

  ros::ServiceServer list_maps_service = nh.advertiseService("list_maps", listMaps);
  ros::ServiceServer publish_map_service = nh.advertiseService("publish_map", publishMap);
  ros::ServiceServer delete_map_service = nh.advertiseService("delete_map", deleteMap);
  ros::ServiceServer rename_map_service = nh.advertiseService("rename_map", renameMap);
  ros::ServiceServer get_map = nh.advertiseService("get_map", getMap);
  ros::ServiceServer set_map_origin = nh.advertiseService("set_origin", setOrigin);

  ROS_DEBUG("spinning.");

  ros::spin();

  // delete map_collection;

  return 0;
}
