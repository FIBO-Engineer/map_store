#!/usr/bin/env python3

PKG = 'map_store'
import roslib; roslib.load_manifest(PKG)
import rospy
import sys
import map_store.srv

map_name = None
try:
    map_name = sys.argv[1]
except:
    print("You must specify a name for the map on the server")
    sys.exit(1)

# print("Waiting for /map_manager/dynamic_map...")
# rospy.wait_for_service('/map_manager/dynamic_map')
print("Waiting for /map_manager/save_map...")
rospy.wait_for_service('/map_manager/save_map')
print("Waiting for /map_manager/list_maps...")
rospy.wait_for_service('/map_manager/list_maps')
print("Waiting for /map_manager/delete_map...")
rospy.wait_for_service('/map_manager/delete_map')

print("Checking for duplicates...")
list_last_maps = rospy.ServiceProxy('/map_manager/list_maps', map_store.srv.ListMaps)
delete_map = rospy.ServiceProxy('/map_manager/delete_map', map_store.srv.DeleteMap)
maps = []
try:
    maps = list_last_maps().map_list
except:
    print("Getting maps from the map_manager has failed")
    sys.exit(2)

for i in maps:
    if (i.name == map_name):
        print("Deleting map", i.map_id)
        delete_map(i.map_id)

print("Starting service...")
save_map = rospy.ServiceProxy('/map_manager/save_map', map_store.srv.SaveMap)
print("Saving map as", map_name)
save_map(map_name)
print("Done")

