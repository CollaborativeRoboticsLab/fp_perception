
# Perception - Eye Gaze Detection

This plugin implements a real-time gaze detection and attention monitoring system using **OpenCV**, **TensorFlow Lite**, **FaceMesh**, and **ROS2**. It detects whether a user is paying attention based on head pose estimation and facial landmark tracking.

## Features

- Real-time face landmark detection using **CLFML FaceMesh (.tflite)** available in [perception_utils](https://github.com/CollaborativeRoboticsLab/perception/blob/main/perception_utils/readme.md)
- Head pose estimation via `solvePnP` using selected 2D/3D keypoints
- Attention state and sustained attention tracking
- Live gaze direction visualization and entropy-based metrics
- ROS2 node integration for perception pipelines
- Calibration module for personalized thresholds

## Architecture Overview

- `FaceMesh`: Loads `.tflite` model, performs inference, extracts 468 facial landmarks
- `AttentionDetector`: Tracks head pose, calculates pitch/yaw angles, detects attention
- `CalibratedAttentionDetector`: Adds baseline calibration logic for personalized thresholds
- `GazeAlgorithm`: Manages detection lifecycle, calibration, ROS2 integration as a perception plugin

## Dependencies

- ROS2 (Humble or later)
- OpenCV (4.x)
- TensorFlow Lite
- C++17
- [KalanaRatnayake/Face_Mesh.Cpp](https://github.com/KalanaRatnayake/Face_Mesh.Cpp) (included in [perception_utils](https://github.com/CollaborativeRoboticsLab/perception/blob/main/perception_utils/readme.md))


## Parameters

You can configure parameters via the ROS2 node:

```yaml
algorithm:
  GazeAlgorithm:
    name: EyeGazeDetection
    calibration:
      time: 10.0
      sample_size: 300
      angle_tolerance: 15.0
    detection:
      attention_threshold: 0.5
      pitch_threshold: 15.0
      yaw_threshold: 20.0
      history_size: 10
      model_path: install/face_mesh/models/CPU/face_mesh.tflite
      attention_state: false
      debug_mode: false

```

## 👨‍💻 Authors

* Kalana Ratnayake (ROS2 Integration & Enhancement)
* Victor Hogeweij, Jeroen Veen [Original FaceMesh C++ Implementation](https://github.com/CLFML/Face_Mesh.Cpp)
