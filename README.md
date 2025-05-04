# 360° Multi-Person Tracking for Ricoh Theta Z1 (ROS 2)

## Overview

`tracker_360` is a ROS 2 package for **real-time person detection and tracking** in 360° equirectangular images captured with a **Ricoh Theta Z1** camera. It supports both **multi-object tracking** using YOLO + ByteTrack and **single-object tracking** using CSRT, with panoramic crop and seam-wrap support.

The system enables detection and tracking across full 360° images, with the ability to extract cropped views of specific tracked targets.

---

## Single Object Tracking (SOT)

This mode focuses on tracking a **single target** once it has been initially detected. The cropped view is adjusted dynamically frame-by-frame.

### CSRT Algorithm

* CSRT (Discriminative Correlation Filter with Channel and Spatial Reliability) is a tracker that builds a model of the target appearance and updates it over time.
* It operates by learning a correlation filter that maximizes response at the target location while suppressing background noise.
* Channel reliability helps weight feature channels (like color or gradients) based on their consistency over time.
* Spatial reliability masks help focus on stable regions within the object.
* CSRT is robust to **scale variations, occlusion, and deformation**, but it requires a good initial bounding box.

### Set Tracked

* The tracking target is automatically initialized as the first detection found by YOLO in the first frame.
* If tracking is lost, YOLO reinitializes the tracker on the next visible detection.
* The cropped region is sent to `/object_crop`, and the full frame with annotations is published on `/object_annotated`.

---

## Multi Object Tracking (MOT)

This mode supports tracking **multiple persons simultaneously**, each with its own persistent ID.

### ByteTrack

* ByteTrack is a **tracking-by-detection** algorithm that associates detections across frames.
* After YOLO predicts bounding boxes, ByteTrack matches detections frame-to-frame using **IoU (Intersection over Union)**.
* It tracks both **high-confidence** and **low-confidence** detections, improving robustness in crowded scenes.
* A Kalman filter predicts object motion, and a Hungarian algorithm performs optimal data association.
* ByteTrack is extremely lightweight and fast, making it ideal for real-time use.

### Follow a Specific ID

You can dynamically change the ID to follow using ROS parameters:

```bash
ros2 param set /multi_person_tracker object_tracked "'3'"
```

To reset to full image mode:

```bash
ros2 param set /multi_person_tracker object_tracked "'full_image'"
```

The annotated panoramic image with all IDs is published on `/multi_object_tracker`.

---

## Crop and Wrap

360° panoramas have a **wrap-around seam** at the 0°/360° boundary. Objects split by the seam can be hard to crop directly due to their split appearance.

To handle this, the `crop_and_wrap()` utility reconstructs a **continuous crop** even when the object spans the seam:

### How it works:

1. The bounding box is padded and its `x1`, `x2` coordinates are checked against the image width.
2. If the crop range goes negative (left of 0) or exceeds the width (right of 360°), the function wraps around:

   * **Left wrap**: crop the right-side of the image and the beginning portion.
   * **Right wrap**: crop the end of the image and the leftmost part.
3. The wrapped crops are **recombined horizontally** using `np.hstack()`.
4. This ensures the object remains visually centered even if its coordinates cross the panorama boundary.

<p align="center">
  <img src="readme_images/wrapped_crop.png" alt="Wrapped Cropping Example" width="600">
</p>

---

## ROS 2 Usage

### Run Nodes Separately

In one terminal:

```bash
ros2 run tracker_360 theta_node
```

In another terminal (multi-object tracking mode):

```bash
ros2 run tracker_360 multi_person_tracker.py --ros-args -p object_tracked:=full_image
```

Or to follow a specific ID (e.g., ID 3):

```bash
ros2 param set /multi_person_tracker object_tracked "'3'"
```

To go back to full-frame mode:

```bash
ros2 param set /multi_person_tracker object_tracked "'full_image'"
```

For single-object CSRT tracking:

```bash
ros2 run tracker_360 yolo_tracker.py
```

### Launch Files

Alternatively, use launch files:

```bash
ros2 launch tracker_360 multi_person_tracker.launch.py
```

```bash
ros2 launch tracker_360 yolo_tracker.launch.py
```

---

## Topics

| Topic Name              | Type                | Direction | Description                               |
| ----------------------- | ------------------- | --------- | ----------------------------------------- |
| `/stitched_image`       | `sensor_msgs/Image` | Sub       | Input panorama from Ricoh Theta Z1        |
| `/multi_object_tracker` | `sensor_msgs/Image` | Pub       | Annotated image with tracked IDs (MOT)    |
| `/object_crop`          | `sensor_msgs/Image` | Pub       | Cropped image around tracked object (SOT) |
| `/object_annotated`     | `sensor_msgs/Image` | Pub       | Full image annotated with current object  |

---

## Requirements

* ROS 2 Humble
* Python ≥ 3.8
* Packages:

  * `ultralytics`, `opencv-python`, `cv_bridge`, `rclpy`, `sensor_msgs`
  * GStreamer 1.x and `libuvc`

---

## Credits

* Ricoh Theta Z1 + libuvc-theta driver
* YOLOv8 by Ultralytics
* ByteTrack (Zhang et al.)
* OpenCV CSRT Tracker
* ROS 2 Foxy/Humble

---

## License

MIT License
