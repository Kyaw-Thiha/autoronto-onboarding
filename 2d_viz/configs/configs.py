from typing import Literal, Tuple, TypedDict, Optional


class GeneralCfg(TypedDict):
    seed: int
    device: Literal["cuda", "cpu"]


class DataPathsCfg(TypedDict):
    train_images: str
    valid_images: str
    train_annotations: str
    valid_annotations: str


class HyperparamsCfg(TypedDict):
    epochs: int
    batch_size: int
    learning_rate: float
    weight_decay: float


class PreprocCfg(TypedDict):
    image_size: Tuple[int, int]
    num_workers: int
    augmentations: bool


class YoloOverridesCfg(TypedDict, total=False):
    model: str
    data: Optional[str]


class Config(TypedDict):
    general: GeneralCfg
    data: DataPathsCfg
    hyper: HyperparamsCfg
    preproc: PreprocCfg
    yolo: YoloOverridesCfg
