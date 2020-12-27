#include <iostream>
#include <memory>
#include <vector>
#include <unordered_map>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#include <apriltags/TagDetector.h>
#include <apriltags/Tag36h11.h>

#include "templateUtillity.hpp"

using namespace std;
using namespace cv;

void test_elimination(){

}

int main() {
  vector<string> file_names;
  glob("../data/*.png", file_names);

  auto tagDetector_ptr =
      std::make_unique<AprilTags::TagDetector>(AprilTags::tagCodes36h11, 2);

  vector<Scalar> colors = {Scalar(0, 0, 255), Scalar(200, 90, 55),
                           Scalar(110, 255, 25), Scalar(20, 190, 155)};

  vector<unordered_map<uint16_t, Point2f>> pts_in_frame;
  pts_in_frame.reserve(file_names.size());

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
            // cout << id << endl;
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
    // cv::resize(imageCopy, imageCopy, Size(), 2.0, 2.0);
    imshow("april grid", imageCopy);
    waitKey(1);
  }

  vector<Point2f> xy0, xy1;
  vector<uint16_t> ids = {8, 46, 66, 72, 136};
  for(const auto &id:ids){
    xy0.push_back(pts_in_frame[0][id]);
    xy1.push_back(pts_in_frame[1][id]);
  }

  Mat mask;
  Mat H = findHomography(xy0, xy1, mask);

  Mat im_in = imread(file_names[0], IMREAD_GRAYSCALE);
  Mat im_in1 = imread(file_names[1], IMREAD_GRAYSCALE);
  Mat im_out, im_out1;

  warpPerspective(im_in, im_out, H, im_in.size());
  imshow("out0", im_out);
  imshow("out1", im_in1);
  addWeighted(im_out, 0.5, im_in1, 0.5, 0, im_out1);
  imshow("out2", im_out1);
  imshow("mask", mask);
  waitKey(0);

  return 0;
}