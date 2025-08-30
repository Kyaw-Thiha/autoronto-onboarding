// lane_align_task.cpp
// Build: g++ lane_align_task.cpp -O2 -std=gnu++17 -I/usr/include/eigen3
// `pkg-config --cflags --libs opencv4` -o lane_align Run:   ./lane_align
// mask.png prior.csv [--roi 10] [--horizon_frac 0.67] [--debug debug.png]

#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <vector>

struct Pt {
  int u{0};
  int v{0};
};

static bool loadPriorCSV(const std::string &path, std::vector<Pt> &pts) {
  std::ifstream f(path);
  if (!f.is_open())
    return false;
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty())
      continue;
    std::istringstream ss(line);
    std::string tok;
    Pt p{};
    if (!std::getline(ss, tok, ','))
      return false;
    p.u = std::stoi(tok);
    if (!std::getline(ss, tok, ','))
      return false;
    p.v = std::stoi(tok);
    pts.push_back(p);
  }
  return !pts.empty();
}

static cv::Mat rasterizeROI(const std::vector<Pt> &poly, cv::Size sz,
                            int half_width_px) {
  cv::Mat roi(sz, CV_8UC1, cv::Scalar(0));
  if (poly.size() < 2)
    return roi;

  // Draw a 1‑px anti‑aliased polyline along the prior
  for (size_t i = 1; i < poly.size(); ++i) {
    cv::line(roi, cv::Point(poly[i - 1].u, poly[i - 1].v),
             cv::Point(poly[i].u, poly[i].v), 255, 1, cv::LINE_AA);
  }

  // TODO A: thicken into a tube via dilation
  // 1) cv::getStructuringElement(cv::MORPH_ELLIPSE, Size(2*half_width_px+1,
  // 2*half_width_px+1)) 2) cv::dilate(roi, roi, kernel)

  cv::Mat kernel = cv::getStructuringElement(
      cv::MORPH_ELLIPSE,
      cv::Size(2 * half_width_px + 1, 2 * half_width_px + 1));
  cv::dilate(roi, roi, kernel);

  return roi;
}

struct FitResult {
  double m{0.0}, c{0.0}, r2{0.0};
  bool ok{false};
};

static FitResult fitLineXY(const std::vector<cv::Point> &pts) {
  FitResult fr;
  const int N = (int)pts.size();
  if (N < 3)
    return fr;

  // Build normal equations: A*[m c]^T ~ x  where each row of A is [y 1]
  Eigen::MatrixXd A(N, 2);
  Eigen::VectorXd x(N);
  double meanx = 0.0;
  for (int i = 0; i < N; ++i) {
    A(i, 0) = pts[i].y;
    A(i, 1) = 1.0;
    x(i) = pts[i].x;
    meanx += pts[i].x;
  }
  meanx /= N;

  // Solve least squares
  Eigen::Vector2d theta = A.colPivHouseholderQr().solve(x);
  fr.m = theta(0);
  fr.c = theta(1);

  // TODO B: compute R^2
  // Use SS_tot and SS_res as described in the instructions.
  // Guard division by ~0. Set fr.r2 to 0 if SS_tot is extremely small.

  // your code here

  fr.ok = true;
  return fr;
}

int main(int argc, char **argv) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0]
              << " mask.png prior.csv [--roi 10] [--horizon_frac 0.67] "
                 "[--debug debug.png]\n";
    return 1;
  }

  std::string maskPath = argv[1], priorPath = argv[2];
  int roi_half = 10;
  double horizon_frac = 2.0 / 3.0;
  std::string debugPath;

  for (int i = 3; i + 1 <= argc - 1; ++i) {
    std::string k = argv[i];
    if (k == "--roi")
      roi_half = std::max(0, std::atoi(argv[++i]));
    else if (k == "--horizon_frac")
      horizon_frac = std::atof(argv[++i]);
    else if (k == "--debug")
      debugPath = argv[++i];
  }

  cv::Mat mask = cv::imread(maskPath, cv::IMREAD_GRAYSCALE);
  if (mask.empty()) {
    std::cerr << "Failed to load mask\n";
    return 1;
  }
  cv::threshold(mask, mask, 127, 255, cv::THRESH_BINARY);

  std::vector<Pt> prior;
  if (!loadPriorCSV(priorPath, prior)) {
    std::cerr << "Failed to load prior CSV\n";
    return 1;
  }

  cv::Mat roi = rasterizeROI(prior, mask.size(), roi_half);

  cv::Mat overlap;
  cv::bitwise_and(mask, roi, overlap);

  // Keep only bottom part of image (mimics “below the horizon”)
  const int H = overlap.rows, W = overlap.cols;
  const int horizon = std::clamp((int)std::round(horizon_frac * H), 0, H);
  for (int y = 0; y < horizon; ++y)
    overlap.row(y).setTo(0);

  // Collect points
  std::vector<cv::Point> pts;
  pts.reserve(5000);
  for (int y = horizon; y < H; ++y) {
    const uchar *row = overlap.ptr<uchar>(y);
    for (int x = 0; x < W; ++x)
      if (row[x])
        pts.emplace_back(x, y);
  }

  if ((int)pts.size() < 20) {
    std::cout << "offset_m=nan, heading_rad=nan, r2=0.0\n";
    return 0;
  }

  FitResult fit = fitLineXY(pts);
  if (!fit.ok || fit.r2 < 0.90) {
    std::cout << "offset_m=nan, heading_rad=nan, r2=" << fit.r2 << "\n";
    return 0;
  }

  // Convert to offset & heading
  const double y_eval = 0.9 * H;
  const double x_eval = fit.m * y_eval + fit.c;
  const double cx = 0.5 * W;
  const double mx = 0.01; // meters per pixel
  const double offset_m = (x_eval - cx) * mx;
  const double heading_rad = std::atan(fit.m);

  std::cout << std::fixed << std::setprecision(4) << "offset_m=" << offset_m
            << ", heading_rad=" << heading_rad << ", r2=" << fit.r2 << "\n";

  // Optional debug overlay
  if (!debugPath.empty()) {
    cv::Mat dbg;
    cv::cvtColor(mask, dbg, cv::COLOR_GRAY2BGR);
    // ROI in blue
    cv::Mat roi_bgr;
    cv::cvtColor(roi, roi_bgr, cv::COLOR_GRAY2BGR);
    dbg = cv::max(dbg, roi_bgr);
    // fitted line
    int y1 = horizon, y2 = H - 1;
    int x1 = (int)std::round(fit.m * y1 + fit.c);
    int x2 = (int)std::round(fit.m * y2 + fit.c);
    cv::line(dbg, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 255),
             2, cv::LINE_AA);
    // center & eval point
    cv::line(dbg, cv::Point((int)cx, 0), cv::Point((int)cx, H - 1),
             cv::Scalar(0, 255, 0), 1);
    cv::circle(dbg, cv::Point((int)std::round(x_eval), (int)std::round(y_eval)),
               4, cv::Scalar(0, 0, 255), -1);
    cv::imwrite(debugPath, dbg);
  }
  return 0;
}
