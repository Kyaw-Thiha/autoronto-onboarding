from models.yolo import Yolo


def main():
    yolo_model = Yolo("yolo11n.pt")
    yolo_model.train("yolo_train")

    # yolo_model.validate()
    # yolo_model.benchmark()


if __name__ == "__main__":
    main()
