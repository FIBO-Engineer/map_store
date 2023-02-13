map_store
=========

Storage manager for OccupancyGrid maps.  Supports naming the most recent map, getting a list of map names, and publishing a specific map.
TODO: use warehouse_ros with warehouse_ros_mongo pluginlib instead of warehouse_ros_mongo directly.

It's recommend to use docker to create a mongodb server. docker-compose.yml is provided.

Note: [AFAIK](https://salsa.debian.org/mongodb-team/mongo-cxx-driver-legacy/-/commit/33ccd8a74ad973bfa38fc66a1e1226fd89460b7f) the last support version the libmongoclient-dev [1.1.3-3](https://packages.ubuntu.com/focal/libmongoclient-dev) from Ubuntu 20.04 does support MongoDB at version MongoDB [3.2](https://github.com/mongodb/mongo-cxx-driver/tree/f0b46f05a054c3043d07f130c23d7419a5fb0cba)
But ChatGPT told that it supports up to version 3.6


For reference (Tested but not working on Ubuntu 20.04, got error code 14)

https://repo.mongodb.org/apt/ubuntu/dists/bionic/mongodb-org/3.6/multiverse/binary-amd64/mongodb-org-server_3.6.22_amd64.deb
https://repo.mongodb.org/apt/ubuntu/dists/bionic/mongodb-org/3.6/multiverse/binary-amd64/mongodb-org-mongos_3.6.22_amd64.deb
https://repo.mongodb.org/apt/ubuntu/dists/bionic/mongodb-org/3.6/multiverse/binary-amd64/mongodb-org-tools_3.6.22_amd64.deb
https://repo.mongodb.org/apt/ubuntu/dists/bionic/mongodb-org/3.6/multiverse/binary-amd64/mongodb-org-shell_3.6.22_amd64.deb

---

This is the new repository location for map_store.  The old one at https://kforge.ros.org/mapstore/hg/ should no longer be used.
