# gen_test.py
import cv2
import numpy as np
import csv

H, W = 480, 640
img = np.zeros((H, W), dtype=np.uint8)
y0 = int(0.5 * H)
slope = 0.22  # pixels of x per pixel of y
x_center = W // 2 + 10  # offset from the image center

# Draw synthetic lane points (bottom half only)
for y in range(y0, H):
    x = int(x_center + slope * (y - y0))
    cv2.circle(img, (x, y), 2, 255, -1)

cv2.imwrite("mask.png", img)

# Create a sparse polyline along the same line as the "prior"
ys = np.linspace(y0, H - 1, 25).astype(int)
prior_pts = [(int(x_center + slope * (y - y0)), int(y)) for y in ys]
with open("prior.csv", "w", newline="") as f:
    w = csv.writer(f)
    for u, v in prior_pts:
        w.writerow([u, v])

print("Wrote mask.png and prior.csv")
