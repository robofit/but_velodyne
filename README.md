rt_road_detection
===========

* there will be set of detectors publishing images (masks) with values from zero to one
* values > 0.5 are for pixels/places which are occupied (grass etc.)
* values < 0.5 are for places which are traversable (pavement etc.)
* 0.5 for unknown
* fusion of detectors' outputs is done using Bayes formula and output is probabilistic 2D occupancy map
* detectors should use only RGB image (depth is used in fusion stage)
* all detectors should be configurable using parameters and dynamic reconfigure
* for all detectors there should be node and nodelet version
* when writing a new detector please, check provided code sample (sample_hue_detector)
