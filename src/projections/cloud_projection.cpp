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

#include "cloud_projection.h"
#include <string>
#include <vector>


namespace kamaz {

namespace hagen{

CloudProjection::PointContainer::PointContainer() {}

CloudProjection::CloudProjection(const ProjectionParams& params)
    : _params(params) {
  if (!_params.valid()) {
    throw std::runtime_error(FBLU("_params not valid for projection."));
  }
  _data = PointMatrix(_params.cols(), PointColumn(_params.rows()));
  _depth_image =
      cv::Mat::zeros(_params.rows(), _params.cols(), cv::DataType<float>::type);
}

void CloudProjection::FixDepthSystematicErrorIfNeeded() {
  if (_depth_image.rows < 1) {
    fprintf(stderr, FBLU("[INFO]: image of wrong size, not correcting depth\n"));
    return;
  }
  if (_corrections.size() != static_cast<size_t>(_depth_image.rows)) {
    //fprintf(stderr, FBLU("[INFO]: Not correcting depth data.\n"));
    return;
  }
  for (int r = 0; r < _depth_image.rows; ++r) {
    auto correction = _corrections[r];
    for (int c = 0; c < _depth_image.cols; ++c) {
      if (_depth_image.at<float>(r, c) < 0.001f) {
        continue;
      }
      _depth_image.at<float>(r, c) -= correction;
    }
  }
}

const cv::Mat& CloudProjection::depth_image() const {
  return this->_depth_image;
}

cv::Mat& CloudProjection::depth_image() { return this->_depth_image; }

} 

}
