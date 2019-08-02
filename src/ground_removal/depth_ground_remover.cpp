// Copyright (C) 2017  I. Bogoslavskyi, C. Stachniss, University of Bonn

// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.

// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
// more details.

// You should have received a copy of the GNU General Public License along
// with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "depth_ground_remover.h"

namespace kamaz {
namespace hagen {

DepthGroundRemover::DepthGroundRemover(const ProjectionParams& params)
: _params{params} {};

void DepthGroundRemover::RepairDepth(cv::Mat& inpainted_depth, int step,
                                    float depth_threshold, int index_) {
    for (int c = 0; c < inpainted_depth.cols; ++c) {
      for (int r = 0; r < inpainted_depth.rows; ++r) {
        float& curr_depth = inpainted_depth.at<float>(r, c);
        if (curr_depth < 0.001f) {
          int counter = 0;
          float sum = 0.0f;
          for (int i = 1; i < step; ++i) {
            if (r - i < 0) {
              continue;
            }
            for (int j = 1; j < step; ++j) {
              if (r + j > inpainted_depth.rows - 1) {
                continue;
              }
              const float& prev = inpainted_depth.at<float>(r - i, c);
              const float& next = inpainted_depth.at<float>(r + j, c);
              if (prev > 0.001f && next > 0.001f &&
                  fabs(prev - next) < depth_threshold) {
                sum += prev + next;
                counter += 2;
              }
            }
          }
          if (counter > 0) {
            curr_depth = sum / counter;
          }
        }
      }
    }

    filtered_map = cv::Mat::zeros(inpainted_depth.size(), cv::DataType<float>::type);
    LocalMaximaFilter local_maximum_filter;
    local_maximum_filter.persistence(inpainted_depth, filtered_map);
    // local_maximum_filter.persistence_and_save_data(inpainted_depth, filtered_map, index_);
    // int folder_index = 9;
    // std::string folder = "/dataset/images/result/";
    // folder = folder + std::to_string(folder_index) + "/";
    // cv::imwrite(folder + std::to_string(index_) + "_processed.jpg", inpainted_depth);
    cv::resize(filtered_map, filtered_map, cv::Size(), 1.0, 1.0);
    cv::resize(inpainted_depth, inpainted_depth, cv::Size(),1.0, 1.0);
    cv::Mat prossed_image = (filtered_map + inpainted_depth)/2;
    depth_img_pointer = prossed_image;
    // DepthCatcher depth_current = {depth_img_pointer, current_cloud_time_stamp};
    // while(depth_catcher.size()>0){
    //   ros::Duration time_diff = current_cloud_time_stamp - depth_catcher.front().time_stamp;
    //   if(time_diff.toSec() > options.depth_expiration_time){
    //     depth_catcher.pop_front();
    //   }else{
    //     break;
    //   }
    // }
    // if(depth_catcher.size()>0){
    //   for(auto const depth_catch: depth_catcher){
    //     depth_img_pointer += depth_catch.depth_map;
    //   }
    //   depth_img_pointer = depth_img_pointer/depth_catcher.size();
    // }
    // depth_catcher.push_back(depth_current);
}

void DepthGroundRemover::CreateAngleImage() {
  cv::Mat angle_image = cv::Mat::zeros(depth_img_pointer.size(), cv::DataType<float>::type);
  cv::Mat x_mat = cv::Mat::zeros(depth_img_pointer.size(), cv::DataType<float>::type);
  cv::Mat y_mat = cv::Mat::zeros(depth_img_pointer.size(), cv::DataType<float>::type);
  const auto& sines_vec = _params.RowAngleSines();
  const auto& cosines_vec = _params.RowAngleCosines();
  float dx, dy;
  x_mat.row(0) = depth_img_pointer.row(0) * cosines_vec[0];
  y_mat.row(0) = depth_img_pointer.row(0) * sines_vec[0];
  for (int r = 1; r < angle_image.rows; ++r) {
    x_mat.row(r) = depth_img_pointer.row(r) * cosines_vec[r];
    y_mat.row(r) = depth_img_pointer.row(r) * sines_vec[r];
    for (int c = 0; c < angle_image.cols; ++c) {
      dx = fabs(x_mat.at<float>(r, c) - x_mat.at<float>(r - 1, c));
      dy = fabs(y_mat.at<float>(r, c) - y_mat.at<float>(r - 1, c));
      angle_image.at<float>(r, c) = atan2(dy, dx);
    }
  }
  angle_img_pointer = angle_image;
}


cv::Mat DepthGroundRemover::GetUniformKernel(int window_size, int type) const {
  if (window_size % 2 == 0) {
    throw std::logic_error("only odd window size allowed");
  }
  cv::Mat kernel = cv::Mat::zeros(window_size, 1, type);
  kernel.at<float>(0, 0) = 1;
  kernel.at<float>(window_size - 1, 0) = 1;
  kernel /= 2;
  return kernel;
}


void DepthGroundRemover::ApplySSASmoothing(int window_size, int bin_size, bool is_normalized) {
  Eigen::MatrixXf smoothed_image(angle_img_pointer.rows, angle_img_pointer.cols);
  Eigen::Matrix<float,Eigen::Dynamic, Eigen::Dynamic> eigen_img;
  cv::cv2eigen(angle_img_pointer, eigen_img);
  for(auto i(0); i< eigen_img.cols(); i++){
    kamaz::hagen::SingularSpectrumAnalysis ssa(eigen_img.col(i), window_size);
    smoothed_image.col(i) = ssa.execute(bin_size, is_normalized);
    // ssa.save_data(i);
    // std::cout<< f.transpose() << std::endl;
    // ssa.save_vec(f, "smoothed_signal_"+ std::to_string(i));
  }
  cv::Mat smoothed_img;
  cv::eigen2cv(smoothed_image, smoothed_img);
  angle_img_pointer = smoothed_img;
}

Radians DepthGroundRemover::GetLineAngle(const cv::Mat& depth_image, int col,
                                         int row_curr, int row_neigh) {
  Radians current_angle;
  Radians neighbor_angle;
  current_angle = _params.AngleFromRow(row_curr);
  neighbor_angle = _params.AngleFromRow(row_neigh);
  // for easiness copy references to depth of current and neighbor positions
  const float& depth_current = depth_image.at<float>(row_curr, col);
  const float& depth_neighbor = depth_image.at<float>(row_neigh, col);
  if (depth_current < _eps || depth_neighbor < _eps) {
    return 0_deg;
  }
  auto x_current = depth_current * cos(current_angle.val());
  auto y_current = depth_current * sin(current_angle.val());
  auto x_neighbor = depth_neighbor * cos(neighbor_angle.val());
  auto y_neighbor = depth_neighbor * sin(neighbor_angle.val());
  auto dx = fabs(x_current - x_neighbor);
  auto dy = fabs(y_current - y_neighbor);
  auto angle = Radians::FromRadians(std::atan2(dy, dx));
  return angle;
}

void DepthGroundRemover::labelOneComponent(int label, kamaz::hagen::Point& start
                  , cv::Mat& label_image, float threshold) {

    std::queue<kamaz::hagen::Point> labeling_queue;
    labeling_queue.push(start);
    size_t max_queue_size = 0;
    while (!labeling_queue.empty()) {
      max_queue_size = std::max(labeling_queue.size(), max_queue_size);
      kamaz::hagen::Point current = labeling_queue.front();
      labeling_queue.pop();
      uint current_label = label_image.at<float>(current.x, current.y);
      if (current_label > 0) {
        continue;
      }
      label_image.at<float>(current.x, current.y) = label;
      auto current_depth = current.depth;
      if (current_depth < 0.001f) {
        continue;
      }
      for (auto& step : adjacent) {
        kamaz::hagen::Point neighbor = current + step;
        if (neighbor.x < 0 || neighbor.x >= label_image.rows) {
          continue;
        }
        neighbor.y = WrapCols(neighbor.y, label_image.cols);
        int index_neighbor = depth_img_cols*neighbor.x + neighbor.y;
        if(index_neighbor >= index_upper_limit){
          std::cout<< "This point is not valid..."<< neighbor.x <<","<< neighbor.y<< std::endl;
          continue;
        }
        neighbor = depth_points[index_neighbor];
        uint neigh_label = label_image.at<float>(neighbor.x, neighbor.y);
        if (neigh_label > 0) {
          continue;
        }
        auto diff = std::fabs(current.angle - neighbor.angle);
        if (diff < threshold) {
          labeling_queue.push(neighbor);
        }
      }
    }
  }

  int DepthGroundRemover::WrapCols(int col, int _label_image_cols) {
    if (col < 0) {
      return col + _label_image_cols;
    }
    if (col >= _label_image_cols) {
      return col - _label_image_cols;
    }
    return col;
  }
}
}
