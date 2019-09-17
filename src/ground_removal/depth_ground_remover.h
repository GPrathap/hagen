// Copyright (C) 2019  Geesara Kulathunga, R. Fedorenko, University of Innopolis, Russia
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

#ifndef GROUND_REMOVAL_DEPTH_GROUND_REMOVER_H_
#define GROUND_REMOVAL_DEPTH_GROUND_REMOVER_H_

#include <opencv2/opencv.hpp>

#include <algorithm>
#include "../projections/projection_params.h"
#include "../utils/radians.h"
#include "../utils/cloud.h"
#include "ssa.h"
#include "quadtree.h"
#include <opencv2/highgui/highgui.hpp>
#include "local_maxima_filter.h"

namespace kamaz {
namespace hagen{

class DepthGroundRemover {

public:

  struct HagenFilterOptions {
     int bin_size;
     float ground_remove_angle;
     int _counter;
     int step;
     float depth_threshold;
     int window_size;
     int kernel_size;
     double depth_expiration_time;
  };

  struct DepthCatcher {
    cv::Mat depth_map;
    ros::Time time_stamp;
  };

  DepthGroundRemover(const ProjectionParams& params);

  ~DepthGroundRemover() = default;

  void CreateAngleImage();

  template <typename T>
  void execute(T& cloud_ptr_current_ptr, int counter){
        int folder_index = 10;
        // std::string folder = "/dataset/images/result/" ;
        // folder = folder + std::to_string(folder_index) + "/";
        options.ground_remove_angle = ((options.ground_remove_angle/180.0)*M_PI);
        current_cloud_time_stamp = cloud_ptr_current_ptr->time_stamp;
        // previous_cloud_time_stamp += current_cloud_time_stamp;
        RepairDepth(cloud_ptr_current_ptr->projection_ptr()->depth_image()
        , options.step, options.depth_threshold, counter);
        // cv::imwrite(folder + std::to_string(counter) + "_filtered_img.jpg", filtered_map);
        // cv::imwrite(folder + std::to_string(counter) + "_processed.jpg", depth_img_pointer);
        // cv::imwrite(folder + std::to_string(counter) + "_depth_img.jpg", cloud_ptr_current_ptr->projection_ptr()->depth_image());
        CreateAngleImage();
        ApplySSASmoothing(options.window_size, options.bin_size, false);
        ZeroOutGroundBFS<T>(options.ground_remove_angle, cloud_ptr_current_ptr
        , options.kernel_size, counter);
        // cv::imwrite(folder + std::to_string(counter) + "_angle_img.jpg", angle_img_pointer);

  }

  template <typename T>
      void ZeroOutGroundBFS(float threshold, T& cloud_ptr_current_ptr, int kernel_size, int counter) {

    cv::Mat _label_image = cv::Mat::zeros(depth_img_pointer.size(), cv::DataType<float>::type);
    std::vector<int> selected_depth_points;
    depth_img_cols =   depth_img_pointer.cols;
    depth_img_rows = depth_img_pointer.rows;
    index_upper_limit = depth_img_cols*depth_img_rows + depth_img_cols;
    for (int c = 0; c < depth_img_cols; ++c) {
      int selected_r = -1;
      int selected_c = -1;
      float selected_depth = 5;
      bool depth_is_set = false;
      for(int r = depth_img_rows - 1; r>=0; r--){
        auto depth = depth_img_pointer.at<float>(r, c);
        auto angle = angle_img_pointer.at<float>(r, c);
        auto current_coord = Point(r, c, angle, depth);
        int index = depth_img_cols*r + c;
        depth_points[index] = current_coord;

        if(depth>0.001f && !depth_is_set){
            selected_depth_points.push_back(r*depth_img_cols+c);
            depth_is_set = true;
        }
        if(depth<selected_depth){
              selected_depth = depth;
              selected_r = r;
              selected_c = c;
        }
      }
      if((selected_depth>0.001f) && (selected_depth<5)){
        selected_depth_points.push_back(selected_r*depth_img_cols+selected_c);
      }
    }

    for(auto index : selected_depth_points){
      auto current_coord = depth_points[index];
      labelOneComponent(1, current_coord, _label_image, threshold);
    }

    kernel_size = std::max(kernel_size - 2, 3);
    cv::Mat kernel = GetUniformKernel(kernel_size, CV_8U);
    cv::Mat dilated = cv::Mat::zeros(_label_image.size(), _label_image.type());
    cv::dilate(_label_image, dilated, kernel);

    auto cloud_depth_mapper = cloud_ptr_current_ptr->cloud_depth_mapper;
    for (int r = 0; r < dilated.rows; ++r) {
      for (int c = 0; c < dilated.cols; ++c) {
        std::list<size_t> point_list = cloud_ptr_current_ptr->projection_ptr()->at(r, c).points();
        if (dilated.at<uint>(r, c) == 0) {
          for (auto const& index_of_point : point_list) {
              cloud_ptr_current_ptr->point_cloud_non_ground_plane->points.push_back(cloud_depth_mapper[index_of_point]);
          }
        }else{
          for (auto const& index_of_point : point_list) {
              cloud_ptr_current_ptr->point_cloud_ground_plane->points.push_back(cloud_depth_mapper[index_of_point]);
          }
        }
      }
    }
  }

  cv::Mat GetUniformKernel(int window_size, int type = CV_32F) const;
  void ApplySSASmoothing(int window_size, int bin_size, bool is_normalized);
  Radians GetLineAngle(const cv::Mat& depth_image, int col, int row_curr, int row_neigh);

  void RepairDepth( cv::Mat& no_ground_image, int step,
                      float depth_threshold, int index_);

  void labelOneComponent(int label, kamaz::hagen::Point& start,
                        cv::Mat& label_image, float threshold);

  int WrapCols(int col, int _label_image_cols);
  ProjectionParams _params;
  float _eps = 0.001f;
  std::array<Point, 4> adjacent = {Point(-1,0), Point(1,0), Point(0,-1)
                          , Point(0,1)};
  mutable int _counter = 0;
  cv::Mat depth_img_pointer;
  cv::Mat angle_img_pointer;
  cv::Mat filtered_map;
  HagenFilterOptions options;
  std::map<int, Point> depth_points;
  int depth_img_cols;
  int depth_img_rows;
  int index_upper_limit;
  float previous_cloud_time_stamp;
  ros::Time current_cloud_time_stamp;
  float max_time_limit = 1;
  std::deque<DepthCatcher> depth_catcher;
};


}
} 

#endif  
