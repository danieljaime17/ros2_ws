# 🧠 ROS 2 Perception Project with Multi-Camera Object Detection

## 🛠️ Project Description

This ROS 2-based project simulates a perception module for autonomous systems. It captures:

- Live video from a USB webcam
- Simulated front and rear cameras using pre-recorded video files

It uses a second node to perform real-time object detection on each video stream using a pre-trained MobileNet SSD model (COCO dataset). Detected objects are highlighted with bounding boxes and confidence scores.

Both the raw and annotated video streams are published as ROS topics and can be visualized using RViz2 or rqt_image_view.

This modular structure is ideal for integration in:
- Autonomous vehicles
- Simulation environments
- Robotics perception pipelines

## 📁 Project Structure

```text
ros2_ws/
└── src/
    └── usb_camera_publisher/
        ├── usb_camera_publisher/
        │   ├── usb_camera_node.py
        │   ├── camera_front_node.py
        │   ├── camera_rear_node.py
        │   ├── detector_objetos.py
        │   ├── models/
        │   │   ├── frozen_inference_graph.pb
        │   │   └── ssd_mobilenet_v1_coco.pbtxt
        │   └── videos/
        │       ├── video_camera_front.mp4
        │       └── video_camera_rear.mp4
        ├── package.xml
        └── setup.py
```
## ⚙️ Setup

### 📹 Required videos

To simulate the front and rear cameras, you must add two video files in the following location:

ros2_ws/src/usb_camera_publisher/usb_camera_publisher/videos/

### ✅ Required filenames:
- video_camera_front.mp4 → Used by the front camera node
- video_camera_rear.mp4 → Used by the rear camera node

If these files are not present, the simulation will not run.

## 🚀 How to Run

Open a new terminal for each node. In every terminal, always source the workspace first:

cd ~/ros2_ws
source install/setup.bash

Then launch the nodes as follows:

### 🟢 Terminal 1 — USB Camera Node

ros2 run usb_camera_publisher usb_camera_node

### 🔵 Terminal 2 — Simulated Front Camera (video)

ros2 run usb_camera_publisher camera_front_node

### 🔵 Terminal 3 — Simulated Rear Camera (video)

ros2 run usb_camera_publisher camera_rear_node

### 🧠 Terminal 4 — Object Detection Node

ros2 run usb_camera_publisher detector_objetos

### 👁️ Terminal 5 — Visualization (RViz2)

rviz2

In RViz2, add three Image displays, and set their topics to:

- /usb_camera/image_annotated
- /camera/front/image_annotated
- /camera/rear/image_annotated

Set Image Transport to raw and Fixed Frame to camera_frame or base_link.

## 🧠 Object Detection

The model used is MobileNet SSD (COCO dataset), with support for common object classes such as:

- person
- car
- truck
- dog
- cat
- cellphone
- and more...

You can expand the labels dictionary in detector_objetos.py to cover all COCO labels.

## 📦 Dependencies

Ensure the following ROS 2 packages and Python dependencies are installed:

sudo apt install ros-humble-rviz2 ros-humble-cv-bridge python3-opencv

If using a different ROS 2 version (e.g., Jazzy), replace humble accordingly.

## 📝 License

MIT License

## 👨‍💻 Author

Fernando Daniel Jaime Alonso  
https://github.com/danieljaime17
