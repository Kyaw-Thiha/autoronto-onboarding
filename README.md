# Autoronto Onboarding
These are the projects of application/onboarding projects for Autoronto's 3D Visualization, 2D Visualization, and Localization teams.

## 📑 Table of Contents

- [🟦 3D Visualization — ROS2 Two-Sum Node](#-3d-visualization--ros2-two-sum-node)  
- [🟥 2D Visualization — Hotdog Detection (YOLOv11)](#-2d-visualization--hotdog-detection-yolov11)  
- [🟩 Localization — Lane Offset & Heading from Binary Lane Mask](#-localization--lane-offset--heading-from-binary-lane-mask)

## 🟦 3D Visualization — ROS2 Two-Sum Node

### 📖 Background  
This project is based on the **R2Y5 3DOD Coding Baseline Question (C++)**.  
In aUToronto’s autonomy system, ROS2 Humble is used as the middleware for inter-process communication.  
The task is to subscribe to two ROS2 topics:

- **`/input`** (`std_msgs/Int8MultiArray`): an array of integers  
- **`/target`** (`std_msgs/Int8`): a single integer  

The goal is to publish, on a third topic, the **indices of two numbers in `/input` that sum to `/target`**.

- **`/solution`** (`std_msgs/Int8MultiArray`): indices of the two numbers  

---

### ⚙️ Implementation  
- A full ROS2 project was generated via shell automation scripts.  
- Two independent ROS2 packages were implemented:  
  - **C++ node** (baseline requirement)  
  - **Python node** (alternative implementation for demonstration & testing)  
- For each package, helper shell scripts were created to:  
  - **Enable/disable** the node automatically  
  - **Test** the node’s behavior end-to-end (publishing `/input` and `/target`, verifying `/solution`)  

Repository: [autoronto-onboarding/3d-viz](https://github.com/Kyaw-Thiha/autoronto-onboarding/tree/main/3d-viz)

---

### ▶️ Usage
```bash
# Build workspace
colcon build --packages-select <package_name>
source install/setup.bash

# Run either C++ or Python implementation
ros2 run cpp_two_sum two_sum_node
ros2 run py_two_sum two_sum_node
```

Custom **shell scripts** are provided in the repo to streamline:  
- Starting/stopping the node  
- Automated testing with sample inputs  

---

### 📌 Notes
- Both nodes subscribe to `/input` and `/target` at **1 Hz**, ensuring synchronous operation.  
- Solution guarantees correctness since the problem states **exactly one valid pair exists**.  
- Includes working **CMakeLists.txt** and **package.xml** for compilation & ROS2 compliance.  
- A demo recording is available showing publisher/subscriber interaction.

---

## 🟥 2D Visualization — Hotdog Detection (YOLOv11)

### 📖 Background  
This project is based on the **R2Y5 2D Vision Task**.  
Inspired by *Silicon Valley*’s "Hot Dog, Not Hot Dog" app, the goal is to build an **object detection pipeline** that can accurately detect and localize hot dogs in images by drawing bounding boxes.  

Dataset: [Google Drive link](https://drive.google.com/drive/folders/1UoM2fdY9uALEiuDtHyTwNU3ldgD1l6oB?usp=sharing)

---

### ⚙️ Implementation  
- **Data Preparation**
  - Loaded training & validation datasets  
  - Normalized bounding box annotations to YOLO format  
  - Applied preprocessing & augmentation  

- **Model Development**
  - Trained YOLOv11 models (`nano` and `small` variants)  
  - Leveraged [Ultralytics YOLO](https://docs.ultralytics.com/quickstart/#use-ultralytics-with-python) for model training and evaluation  
  - Integrated augmentation to improve generalization  

- **Evaluation**
  - Used **mAP (mean Average Precision)** at IoU thresholds (`50`, `75`, `50–95`)  
  - Compared **precision, recall, inference speed** across models  
  - Verified results with both quantitative metrics and confusion matrices  

Repository: [autoronto-onboarding/2d_viz](https://github.com/Kyaw-Thiha/autoronto-onboarding/tree/main/2d_viz)

---

### 📊 Results

| Model   | Precision (P) | Recall (R) | mAP50 | mAP75 | mAP50–95 | Inference Speed |
|---------|---------------|------------|-------|-------|----------|-----------------|
| YOLO11n (nano) | 0.673 | 0.548 | 0.613 | 0.485 | 0.425 | 2.3 ms / image |
| YOLO11s (small) | 0.665 | 0.560 | 0.613 | 0.473 | 0.419 | 3.2 ms / image |

**Confusion Matrix Summary**

| Model        | Predicted → Hotdog | Predicted → Background |
|--------------|---------------------|-------------------------|
| **YOLO11n**  | Hotdog: 697<br>Background: 323 | Hotdog: 554<br>Background: 0 |
| **YOLO11s**  | Hotdog: 708<br>Background: 345 | Hotdog: 543<br>Background: 0 |

---

### ▶️ Usage
```bash
# clone repo
git clone https://github.com/Kyaw-Thiha/autoronto-onboarding.git
cd autoronto-onboarding/2d_viz

# install dependencies
pip install -r requirements.txt

# train model (example with YOLOv11-nano)
yolo detect train data=hotdog.yaml model=yolo11n.pt epochs=50 imgsz=640

# run inference
yolo detect predict model=runs/detect/train/weights/best.pt source=./data/val
```

---

### 📌 Notes
- Both YOLOv11 **nano** and **small** models achieved similar accuracy (**mAP50 ≈ 0.61**).  
- **Nano** is significantly faster at inference, making it preferable for real-time applications.  
- Codebase includes training configs, evaluation scripts, and saved predictions.  

---

## 🟩 Localization — Lane Offset & Heading from Binary Lane Mask

### 📖 Background  
Based on the **R2Y5 Localization Task**, this project estimates a vehicle’s **lateral offset** from lane center and **heading angle** relative to lane markings using a **binary image** of detected lanes.

You’ll complete two core steps:
1) **Rasterize ROI**: thicken a thin polyline into a tube (dilation with an elliptical kernel).  
2) **Line Fit & Quality**: fit \(x = m\,y + c\) and compute **\(R^2\)**; reject low-quality fits.

Repository: [autoronto-onboarding/localization](https://github.com/Kyaw-Thiha/autoronto-onboarding/tree/main/localization)

---

### ⚙️ Implementation Highlights
- **`lane_align_task.cpp`**
  - `rasterizeROI(...)`:  
    - Create elliptical kernel of size `(2*half_width_px+1, 2*half_width_px+1)` with `cv::MORPH_ELLIPSE`  
    - Apply `cv::dilate(roi, roi, kernel)`
  - `fitLineXY(...)`:  
    - Solve least-squares for `m` and `c` (model: \( \hat{x} = m\,y + c \))  
    - Compute \( R^2 = 1 - \frac{\sum (x - \hat{x})^2}{\sum (x - \bar{x})^2} \) with small-denominator guard  
    - **Validation rules**: if **< 20 points** remain or **\(R^2 < 0.90\)** → output `nan` results
- **Automation**: Shell scripts are included to **build**, **run**, and **auto-test** the pipeline end-to-end.

---

### 🧪 Testing
Use the provided generator to produce synthetic cases:
```bash
# from repo root
cd localization
python3 gen_test.py  # generates/updates test images/cases
```

Then run the automated test script (provided in this repo) to execute the pipeline on generated cases and summarize results:
```bash
./scripts/test_all.sh
```

---

### ▶️ Build & Run (CMake + OpenCV)
```bash
# deps: Ubuntu 22.04, OpenCV (libopencv-dev), CMake, g++
sudo apt-get update
sudo apt-get install -y build-essential cmake libopencv-dev python3 python3-numpy

# build
cd localization
cmake -S . -B build
cmake --build build -j

# run (example)
./build/lane_align_app --input ./data/bin_lane.png --meters_per_px 0.02
```

---

### 🧾 Output Format
The executable prints a single line per image:
```
offset_m=<lateral_offset_in_meters>, heading_rad=<heading_angle_in_radians>, r2=<fit_quality>
```
Examples:
```
offset_m=0.12, heading_rad=-0.03, r2=0.97
offset_m=nan, heading_rad=nan, r2=0.72
```
- **`nan`** indicates low-quality fit or insufficient inliers per the task rules.

---

### 📌 Notes
- The **elliptical dilation** controls lane “tube” thickness via `half_width_px`; tune this to match expected lane width in pixels.  
- \(R^2\) near **1.0** indicates a strong linear alignment between lane samples and the fitted line.  
- All key steps are reproducible via the **automation scripts** (build + run + test) included in the repo.
