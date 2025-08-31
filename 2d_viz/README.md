# R2Y5 2D Vision Task — Hotdog Detection

## 📖 Background
Inspired by the popular *Silicon Valley* show, where the character Jian-Yang creates a "Hot Dog, Not Hot Dog" app, this project implements an **object detection model** to detect and localize hot dogs in images. The model outputs bounding boxes around detected hot dogs.

You can find the dataset [here](https://drive.google.com/drive/folders/1UoM2fdY9uALEiuDtHyTwNU3ldgD1l6oB?usp=sharing).

---

## 🧩 Project Structure
1. **Data Preparation**
   - Loaded and preprocessed training & validation datasets.
   - Converted bounding box annotations into a consistent YOLO format.
   - Applied augmentation techniques for robustness.

2. **Model Development**
   - Trained YOLOv11 models (`nano` and `small` variants).
   - Used Ultralytics framework for streamlined training & evaluation.
   - Integrated preprocessing and augmentation into pipeline.

3. **Evaluation**
   - Evaluated using **mAP (mean Average Precision)** at IoU thresholds (`50`, `75`, `50–95`).
   - Compared precision, recall, and speed between models.
   - Analyzed confusion matrix for qualitative insights.

---

## ⚙️ Setup
```bash
# clone repo
git clone https://github.com/your-username/hotdog-detection.git
cd hotdog-detection

# install dependencies
pip install -r requirements.txt

# train model (example with YOLOv11-nano)
yolo detect train data=hotdog.yaml model=yolo11n.pt epochs=50 imgsz=640
```

---

## 📊 Results

| Model   | Precision (P) | Recall (R) | mAP50 | mAP75 | mAP50–95 | Inference Speed |
|---------|---------------|------------|-------|-------|----------|-----------------|
| YOLO11n (nano) | 0.673 | 0.548 | 0.613 | 0.485 | 0.425 | 2.3 ms / image |
| YOLO11s (small) | 0.665 | 0.560 | 0.613 | 0.473 | 0.419 | 3.2 ms / image |

**Confusion Matrix Summary**

| Predicted   | Hotdog | Background |
|-------------|--------|------------|
| **Hotdog**  | 697    | 323        |
| **Background** | 554    | 0          | *(nano)* |
| **Hotdog**  | 708    | 345        |
| **Background** | 543    | 0          | *(small)* |

---

## 📌 Observations
- Both **nano** and **small** YOLOv11 models achieved similar **mAP50 ≈ 0.61**.
- **Nano model** was faster (2.3 ms vs 3.2 ms inference per image) while maintaining comparable accuracy.
- Recall was slightly higher for the small model, but the nano variant remains competitive and more efficient.

---

## 🚀 References
- [YOLO Ultralytics Documentation](https://docs.ultralytics.com/quickstart/#use-ultralytics-with-python)  
- Dataset provided in the task description.

---
