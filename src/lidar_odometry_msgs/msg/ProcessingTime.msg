# Time spent by the odometry node on a single LiDAR scan.
# Published once per scan, so the numbers can be checked against the 100 ms budget.

# Stamp of the scan these timings refer to, not the time the computation finished.
std_msgs/Header header

# Transform to base_link, cropping, ground removal and downsampling.
float64 preprocessing_ms

# Alignment of the scan against the local submap.
float64 registration_ms

# Whole callback, from receiving the scan to publishing the results.
float64 total_ms

# Points in the raw scan and after preprocessing, to relate timing to workload.
uint32 input_points
uint32 filtered_points