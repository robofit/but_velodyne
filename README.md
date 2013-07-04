rt_road_detection
===========

* there will be set of road/not_road detectors
* output of detector is a mask with 8bit values (0-255)
* road detector
 * 255 for regions with road
 * publishing to topic /detectors/road/detector_name
* not_road detector
 * 255 for regions which are not traversable
 * publishing to topic /detectors/not_road/detector_name


* sample_grass_detector
 * is an dumb example of not_road detector
 * it works only with left camera image
 * please follow proposed code structure
 * there should be always both - node and nodelet variant of code


* TODOs
 * node/nodelet for projection of masks to ground plane (should it work with depth somehow??)
 * plugin to navigation stack for building costmap from these detections (taking into account detector type)
