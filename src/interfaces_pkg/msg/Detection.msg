# defines a YOLOv8 detection result

# class probability
int32 class_id
string class_name
float64 score

# ID for tracking
string id

# 2D bounding box surrounding the object in pixels
interfaces_pkg/BoundingBox2D bbox

# 3D bounding box surrounding the object in meters
interfaces_pkg/BoundingBox3D bbox3d

# segmentation mask of the detected object
# it is only the boundary of the segmented object
interfaces_pkg/Mask mask

# keypoints for human pose estimation
interfaces_pkg/KeyPoint2DArray keypoints

# keypoints for human pose estimation
interfaces_pkg/KeyPoint3DArray keypoints3d