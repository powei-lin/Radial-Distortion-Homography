#include <iostream>
#include <memory>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <random>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#include <apriltags/TagDetector.h>
#include <apriltags/Tag36h11.h>

#include "templateUtillity.hpp"
#include "EuRoC_dictionary.h"

using namespace std;
using namespace cv;

template <typename Scalar>
Scalar random(Scalar lower_bound, Scalar upper_bound) {
  return lower_bound + 1.0 * (upper_bound - lower_bound) * rand() / RAND_MAX;
}

// struct fisheyeProject{
//   fisheyeProject(){}

//   template <typename T>
//   bool operator()(const T *const parameters, const T *const parameter2,
//                   T *residuals) const {
//     const T &x = parameters[0];
//     const T &y = parameters[1];
//     const T &z = parameters[2];
//     residuals[0] = x + 2. * y + 4. * z;
//     residuals[1] = y * z;
//     residuals[2] = x - T(10);
//     residuals[4] = parameter2[0];
//     return true;
//   }
// };

int main() {

  vector<string> file_names;
  glob("../data/*.png", file_names);

  // construct tag detector
  auto tagDetector_ptr =
      std::make_unique<AprilTags::TagDetector>(AprilTags::tagCodes36h11, 2);

  // colors for debug
  vector<Scalar> colors = {Scalar(0, 0, 255), Scalar(200, 90, 55),
                           Scalar(110, 255, 25), Scalar(20, 190, 155)};

  vector<unordered_map<uint16_t, Point2f>> pts_in_frame;
  pts_in_frame.reserve(file_names.size());

  // load frames and detect tags
  for (const auto &name : file_names) {
    cout << name << endl;
    Mat image = imread(name, IMREAD_GRAYSCALE);
    const uint16_t img_w = image.cols;
    const uint16_t img_h = image.rows;
    Mat imageCopy = image.clone();
    cvtColor(imageCopy, imageCopy, cv::COLOR_GRAY2BGR);
    std::vector<AprilTags::TagDetection> detections =
        tagDetector_ptr->extractTags(image);

    unordered_map<uint16_t, Point2f> p2d;
    for (auto it = detections.begin(); it != detections.end(); ++it) {
      for (int i = 0; i < 4; i++) {
        Point2f pt(it->p[i].first, it->p[i].second);
        const uint16_t id = it->id * 4 + i;
        if (it->good) {
          if (inBoard(pt, img_w, img_h, 10)) {
            cv::putText(imageCopy, to_string(it->id * 4 + i), pt, 1, 1.0,
                        colors[i], 1);
            cv::circle(imageCopy, pt, 2, colors[i]);
            // const auto &pt3d = Dict<>::EuRoC[id];
            p2d[id] = Point2f(pt.x, pt.y);
            // cout << id << p2d.at(id) << endl;
          } else {
            cout << "bad" << pt.x << "\t" << pt.y << endl;
          }

        } else {
          cv::putText(imageCopy, to_string(it->id * 4 + i), pt, 1, 1.0,
                      Scalar(0, 255, 0), 1);
          cv::circle(imageCopy, pt, 10, Scalar(0, 255, 0), -1);
        }
      }
    }
    pts_in_frame.push_back(p2d);
    // frame_pt2d.push_back(p2d);
    // frame_pt3d.push_back(p3d);
    // cout << p2d.size() << endl;
    Mat imageCopy2;
    cv::resize(imageCopy, imageCopy2, Size(), 2.0, 2.0);
    imshow("april grid", imageCopy2);
    waitKey(1);
  }

  vector<Point2f> xy0, xy1;
  vector<Point3f> xyz;

  unordered_set<uint16_t> ids;
  constexpr uint16_t p3d_number = 20;
  while (ids.size() < p3d_number) {
    ids.insert(rand() % 144);
  }

  for (const auto &id : ids) {
    xy0.push_back(pts_in_frame[0][id]);
    xy1.push_back(pts_in_frame[1][id]);
    const auto &pt3d = Dict<>::EuRoC[id];
    xyz.emplace_back(pt3d[0], pt3d[1], pt3d[2]);
    cout << id << xyz.back() << endl;
  }
  // waitKey(0);

  const Mat im_in = imread(file_names[0], IMREAD_GRAYSCALE);
  const Mat im_in1 = imread(file_names[1], IMREAD_GRAYSCALE);

  // test project points
  vector<vector<Point3f>> p3ds = {xyz, xyz};
  vector<vector<Point2f>> p2ds = {xy0, xy1};
  auto flag = cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
  Mat K, D;
  vector<Vec3d> rvecs, tvecs;
  auto err =
      fisheye::calibrate(p3ds, p2ds, im_in.size(), K, D, rvecs, tvecs, flag);
  cout << "err: " << err << endl;
  cout << "K:\n" << K << endl << endl;
  cout << "D:\n" << D << endl << endl;
  cout << rvecs[0] << tvecs[0] << endl;

  const double fx = *K.ptr<double>(0, 0);
  const double alpha_fx = *K.ptr<double>(0, 1);
  const double alpha = alpha_fx / fx;

  // validate after calibration
  vector<Point2f> out_p2f;
  fisheye::projectPoints(xyz, out_p2f, rvecs[0], tvecs[0], K, D, alpha);
  Mat img0show;

  cvtColor(im_in, img0show, COLOR_GRAY2BGR);
  for (const auto &pt : xy0) {
    cv::circle(img0show, pt, 2, Scalar(0, 0, 255), -1);
  }
  for (const auto &pt : out_p2f) {
    cv::circle(img0show, pt, 5, Scalar(255, 0, 0), 2);
  }
  imshow("project points", img0show);
  waitKey(0);

  {
    Mat mask;
    Mat H = findHomography(xy0, xy1, mask);

    Mat img0color, img1color;
    cvtColor(im_in, img0color, COLOR_GRAY2BGR);
    cvtColor(im_in1, img1color, COLOR_GRAY2BGR);

    // Mat im_out, im_out1;

    // warpPerspective(im_in, im_out, H, im_in.size());
    // imshow("out0", im_out);
    // imshow("out1", im_in1);
    // addWeighted(im_out, 0.5, im_in1, 0.5, 0, im_out1);
    // imshow("out2", im_out1);
    // imshow("mask", mask);
    // waitKey(0);

    for (size_t p = 0; p < 6; p++) {
      Scalar color(rand() % 256, rand() % 256, rand() % 256);
      // cout << color << endl;
      cv::circle(img0color, xy0[p], 5, color, -1);
      cv::circle(img1color, xy1[p], 5, color, -1);
    }
    imshow("img0", img0color);
    imshow("img1", img1color);
    // cv::waitKey(0);
  }

  // solve H
  {
    const double cx = im_in.size().width / 2.0 - 0.5;
    const double cy = im_in.size().height / 2.0 - 0.5;
    vector<pair<Eigen::Vector2d, Eigen::Vector2d>> point_pair(6);
    for (int p = 0; p < 6; p++) {
      Eigen::Vector2d p0(xy0[p].x - cx, xy0[p].y - cy);
      Eigen::Vector2d p1(xy1[p].x - cx, xy1[p].y - cy);
      // cout << p0.x() << ", " << p0.y() << endl;
      // cout << p1.x() << ", " << p1.y() << endl << endl;
      point_pair[p] = make_pair(p0, p1);
    }

    // M = [6x8] Matrix
    // v = [h11, h12, h13, h21, h22, h23, lh13, lh23]T
    Eigen::MatrixXd M(6, 8);
    for (int r = 0; r < 6; r++) {
      const double x = point_pair[r].first.x();
      const double y = point_pair[r].first.y();
      const double x_p = point_pair[r].second.x();
      const double y_p = point_pair[r].second.y();
      M.row(r)(0) = -1.0 * y_p * x;
      M.row(r)(1) = -1.0 * y_p * y;
      M.row(r)(2) = -1.0 * y_p;
      M.row(r)(3) = x_p * x;
      M.row(r)(4) = x_p * y;
      M.row(r)(5) = x_p;
      M.row(r)(6) = -1.0 * y_p * (x * x + y * y);
      M.row(r)(7) = x_p * (x * x + y * y);
    }
    cout << M << endl << endl;
    Eigen::FullPivLU<Eigen::MatrixXd> lu(M);
    Eigen::MatrixXd M_n = lu.kernel();
    M_n.transposeInPlace();
    cout << M_n << endl;

    // ax^2 + bx + c = 0
    const double a = -1.0 * M_n(0, 2) * M_n(0, 7) + M_n(0, 5) * M_n(0, 6);
    const double b = -1.0 * M_n(0, 2) * M_n(1, 7) - M_n(1, 2) * M_n(0, 7) +
                     M_n(1, 5) * M_n(0, 6) + M_n(0, 5) * M_n(1, 6);
    const double c = -1.0 * M_n(1, 2) * M_n(1, 7) + M_n(1, 5) * M_n(1, 6);
    const double g0 = (-b + sqrt(b*b-4.0*a*c))/(2.0*a);
    const double g1 = (-b - sqrt(b*b-4.0*a*c))/(2.0*a);
    const double lambda0 = (g0 * M_n(0, 6) + M_n(1, 6)) / (g0 * M_n(0, 2) + M_n(1, 2));
    const double lambda1 = (g1 * M_n(0, 6) + M_n(1, 6)) / (g1 * M_n(0, 2) + M_n(1, 2));
    cout << a << endl;
    cout << "gamma0: " << g0 << ", lambda0: " << lambda0 << endl;
    cout << "gamma1: " << g1 << ", lambda1: " << lambda1 << endl;

    // cout << "vector_v:\n" << vector_v << endl << endl;
    // cout << vector_v(0, 2)*lambda0 << " " << vector_v(0, 6) << endl;

    Eigen::Matrix<double, 1, 8> vector_v = g1 * M_n.row(0) + M_n.row(1);
    const double h11 = vector_v(0, 0);
    const double h12 = vector_v(0, 1);
    const double h13 = vector_v(0, 2);
    const double h21 = vector_v(0, 3);
    const double h22 = vector_v(0, 4);
    const double h23 = vector_v(0, 5);
    const double l = lambda1;

    Eigen::MatrixXd A(6, 4);
    Eigen::MatrixXd b_v(6, 1);
    for (int r = 0; r < 6; r++) {
      const double x = point_pair[r].first.x();
      const double y = point_pair[r].first.y();
      const double x_p = point_pair[r].second.x();
      const double y_p = point_pair[r].second.y();
      A.row(r)(0) = h11*x*x_p*x_p + h11*x*y_p*y_p + h12*x_p*x_p*y + h12*y*y_p*y_p + h13*l*x*x*x_p*x_p + h13*l*x*x*y_p*y_p + h13*l*x_p*x_p*y*y + h13*l*y*y*y_p*y_p + h13*x_p*x_p + h13*y_p*y_p;
      A.row(r)(1) = -x*x_p;
      A.row(r)(2) = -x_p*y;
      A.row(r)(3) = -l*x*x*x_p - l*x_p*y*y - x_p;
      b_v(r, 0) = -h11*x - h12*y - h13*l*x*x - h13*l*y*y - h13;
    }
    cout << A << endl;
    cout << b_v << endl;
    const auto result = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b_v);
    cout << result << endl;

        

    // cout << cx << endl;
    //  Eigen::Vector3d
  }

  return 0;
}