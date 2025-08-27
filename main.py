from models.yolo import Yolo


def main():
    yolo_model = Yolo("yolo11n.pt")
    yolo_model.benchmark()


if __name__ == "__main__":
    main()
