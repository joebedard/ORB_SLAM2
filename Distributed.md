# Distributed SLAM - Experimental

The ability to run tracking clients and a mapping server (distributed over a network) is experimental and in development. This document is intended to help contributors learn about the current implementation more quickly.


# How to Run

Note that it uses two well-known patterns for communication: request-reply (client-server) and publish-subscribe. You'll need to configure both of these communication pathways.

## Mapping Server

1. Modify `src-server/mapper_server.yaml` 
   * Server Parameters - typical ZeroMQ settings for authorized IP addresses, ports, timeouts and linger
   * Map Viewer Parameters - to display the server's copy of the map

2. Execute the following command. 
```
./src-server/ORB_SLAM2_server src-server/mapper_server.yaml
```

## Tracking Client

At this time, there is only one example of a tracking client: `Examples/RealSense2/realsense2client`

1. Modify the mapper settings: `Examples/RealSense2/mapper_client.yaml`
   * Client Parameters - typical ZeroMQ settings for the server's IP addresses and ports, as well as client timeouts and linger
   * Map Viewer Parameters - to display the client's copy of the map

2. Modify the tracking client settings. Example files are at:
   * `Examples/RealSense2/realsense2-1.yaml` (Viewer Parameters are ignored by realsense2client)
   * `Examples/RealSense2/realsense2-2.yaml` (Viewer Parameters are ignored by realsense2client)

3. Execute the following command.
```
./Examples/RealSense2/realsense2client vocabulary_file_and_path mapper_settings_file_and_path tracker_settings_file_and_path 
```


# For Developers

Here we describe some of the technical details of the ZeroMQ implementation for our contributors.

The following information is transmitted via request-reply. The reply always includes a success/fail code.
* Tracker Login
  * (request) pivot calibration
  * (reply) tracker id, key frame and map point id creation parameters
* Tracker Logout
  * (request) tracker id
* Initialize Mono
  * (request) tracker id, two keyframes and many new map points
* Initialize Stereo
  * (request) tracker id, one key frame and many new map points
* Insert Key Frame
  * (request) tracker id, one key frame and if stereo, many new map points
* Update Pose
  * (request) tracker id and current tracking pose
* Get Map (used by secondary tracking clients to request a copy of the map via publish-subscribe)
  * (request) tracker id
* Reset (requested by one tracker, then the server will notify all tracking clients via publish-subscribe)
  * (request) none

The following information is transmitted via publish-subscribe. Messages include a subscriber id (a.k.a. tracker id) when the message is directed at a specific tracking client--all other tracking clients will ignore this message. However, most messages are broadcast to all clients.
* Map Change
  * set of created/updated map points, including their data
  * set of deleted map point identifiers, excludes map point data
  * set of created/updated key frames, including their data
  * set of deleted key frame identifiers, excludes key frame data
* Idle 
  * boolean indicating whether the mapper is idle
* Map Reset 
  * triggers the client to reset the map
* Pause Requested 
  * boolean indicating whether tracking clients should pause during loop closure or global bundle adjustment
* Pivot Update
  * tracker id - of tracking client that is changing its pivot calibration
  * pivot calibration
* Pose Update
  * tracker id - of tracking client that is updating its pose
  * new pose


The most important source code to look at is:

* src/MapperClient.cc - to understand the client communications
* src-server/server.cc - to understand the server communications
* src/KeyFrame.cc - serializes itself to a buffer, see the functions: GetBufferSize, ReadBytes and WriteBytes
* src/MapPoint.cc - serializes itself to a buffer, see the functions: GetBufferSize, ReadBytes and WriteBytes



