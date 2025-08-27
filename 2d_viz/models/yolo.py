from typing import Optional
import yaml
from pathlib import Path

from ultralytics import YOLO
from ultralytics.utils.benchmarks import benchmark

from models.adapter import Adapter

# Absolute path for yolo dateset yaml
THIS_FILE = Path(__file__).resolve()
ROOT = THIS_FILE.parents[1]  # project root (one level above 'models')
DATA_YAML = ROOT / "data" / "yolo" / "dataset.yaml"

assert DATA_YAML.exists(), f"Missing dataset.yaml at {DATA_YAML}"


class Yolo(Adapter):
    model: YOLO
    data_path: Path = DATA_YAML

    def __init__(self, model_name: str, checkpoint: Optional[str] = None) -> None:
        super().__init__()
        self.model = YOLO(model_name)
        if checkpoint is not None:
            assert checkpoint.endswith(".pt"), "Enter a valid checkpoint ending with .pt"
            self.model.load(checkpoint)

    def train(self, training_name="yolo_train"):
        """
        Wrapper around YOLO training
        Training Parameters: https://docs.ultralytics.com/modes/train/#train-settings
        Image Augmentations: https://docs.ultralytics.com/guides/yolo-data-augmentation/
        """
        return self.model.train(
            # Model Configs
            data=self.data_path,
            epochs=100,
            imgsz=256,
            patience=10,
            pretrained=True,
            # Checkpoint Saving Configs
            save=True,
            save_period=10,
            project="outputs",
            name=training_name,
            exist_ok=True,
            # Validationg & Metrics
            val=True,
            plots=True,
            # Image Augmentations
            hsv_h=0.2,
            hsv_s=0.3,
            hsv_v=0.3,
            degrees=15,
            translate=0.25,
            scale=0.2,
            shear=5,
            flipud=0.5,
            fliplr=0.5,
            mosaic=0.25,
        )

    def validate(self, validation_name="yolo_val"):
        """
        Wrapper around YOLO validation.
        Validation Arguments: https://docs.ultralytics.com/modes/val/
        """
        metrics = self.model.val(
            data=self.data_path,
            # Checkpoing Saving Configs
            project="outputs",
            name=validation_name,
            # Output & Visualization
            plots=True,
            visualize=True,
            save_json=True,
        )
        print(f"Map (50-95): {metrics.box.map}")
        print(f"Map 50: {metrics.box.map50}")
        print(f"Map 75: {metrics.box.map75}")
        print(metrics.confusion_matrix.to_df())

    def benchmark(self, checkpoint="yolo11n.pt"):
        """
        Wrapper around YOLO benchmark.
        Validation Arguments: https://docs.ultralytics.com/modes/benchmark/
        """
        benchmark(Path(checkpoint), data=self.data_path, imgsz=256)

    # def _load_configs(self):
    #     with open("common.yaml", "r") as f:
    #         common_cfg = yaml.safe_load(f)
    #     with open("yolo.yaml", "r") as f:
    #         yolo_cfg = yaml.safe_load(f)
    #
    #     self.configs = {**common_cfg, **yolo_cfg}
