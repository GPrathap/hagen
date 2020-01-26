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

#ifndef GROUND_REMOVAL_LOCAL_MAXIMA_FILTER_H_
#define GROUND_REMOVAL_LOCAL_MAXIMA_FILTER_H_

#include <opencv2/opencv.hpp>
// #include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <algorithm>
#include <iostream>
#include <map>
#include <cassert>
#include <tuple>
#include <set>
#include "union_find.h"
// #include <parallel/algorithm>
#include <cnpy.h>

namespace kamaz {
    namespace hagen{

    struct groups {
        float p1;
        float p2;
        Pixel p3;
    };

class LocalMaximaFilter{
 public:
    LocalMaximaFilter() = default;
    ~LocalMaximaFilter() = default;

    float get_pixel_value(Pixel p);
    void iter_neighbors(Pixel p, std::vector<Pixel> &item_list);
    void persistence(const cv::Mat& img, cv::Mat& filtered_image, std::map<int, int>& detected_indics);
    void persistence_and_save_data(const cv::Mat& img, cv::Mat& filtered_image, int index);

    float distance(int x, int y, int i, int j);
    void print_tuple(std::tuple<int, int> object);
    float gaussian(float x, float sigma);
    void applyBilateralFilter(cv::Mat& filteredImage, int source_index, int diameter, float sigmaI, float sigmaS);

private:
    UnionFindDS uf; 
    int img_width;
    int img_height;

    std::vector<float> depth_img_row_vector;
};
}
}
#endif 