/**
* Copyright 2018, ftdlyc <yclu.cn@gmail.com>
*
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 3 of the License, or (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with this library; if not, see <http://www.gnu.org/licenses/>.
*/

/*
% Copyright 2012. All rights reserved.
% Author: Andreas Geiger
%         Institute of Measurement and Control Systems (MRT)
%         Karlsruhe Institute of Technology (KIT), Germany

% This is free software; you can redistribute it and/or modify it under the
% terms of the GNU General Public License as published by the Free Software
% Foundation; either version 3 of the License, or any later version.

% This software is distributed in the hope that it will be useful, but WITHOUT ANY
% WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
% PARTICULAR PURPOSE. See the GNU General Public License for more details.

% You should have received a copy of the GNU General Public License along with
% this software; if not, write to the Free Software Foundation, Inc., 51 Franklin
% Street, Fifth Floor, Boston, MA 02110-1301, USA
*/

#pragma once
#ifndef LIBCBDETECT_CONFIG_H
#define LIBCBDETECT_CONFIG_H

#include <vector>

#include <opencv2/opencv.hpp>

#ifdef _MSC_VER
#define M_PI 3.14159265358979323846   /* pi */
#define M_PI_2 1.57079632679489661923 /* pi/2 */
#define M_PI_4 0.78539816339744830962 /* pi/4 */
#define M_1_PI 0.31830988618379067154 /* 1/pi */
#define M_2_PI 0.63661977236758134308 /* 2/pi */
#endif

#ifndef LIBCBDETECT_DLL_DECL
#if IS_A_DLL && defined(_MSC_VER)
#define LIBCBDETECT_DLL_DECL __declspec(dllexport)
#else
#define LIBCBDETECT_DLL_DECL
#endif
#endif

namespace cbdetect {

enum DetectMethod {
  // origin method fast mode
  TemplateMatchFast = 0,
  // origin method slow mode
  TemplateMatchSlow,

  // compute hessian of image, detect by a threshold
  // form https://github.com/facebookincubator/deltille
  HessianResponse,

  // paper: Accurate Detection and Localization of Checkerboard Corners for Calibration
  LocalizedRadonTransform
};

enum CornerType {
  SaddlePoint = 0,
  MonkeySaddlePoint
};

typedef struct Params {
  bool show_processing;
  bool show_debug_image;
  bool show_grow_processing;
  bool norm;
  bool polynomial_fit;
  int norm_half_kernel_size;
  int polynomial_fit_half_kernel_size;
  double init_loc_thr;
  double score_thr;
  bool strict_grow;
  bool overlay;
  bool occlusion;
  DetectMethod detect_method;
  CornerType corner_type;
  std::vector<int> radius;

  Params()
      : show_processing(false)
      , show_debug_image(false)
      , show_grow_processing(false)
      , norm(false)
      , polynomial_fit(true)
      , norm_half_kernel_size(31)
      , polynomial_fit_half_kernel_size(4)
      , init_loc_thr(0.01)
      , score_thr(0.01)
      , strict_grow(true)
      , overlay(false)
      , occlusion(false)
      , detect_method(HessianResponse)
      , corner_type(SaddlePoint)
      , radius({3, 5}) {}
} Params;

typedef struct Corner {
  std::vector<cv::Point2d> p;
  std::vector<int> r;
  std::vector<cv::Point2d> v1;
  std::vector<cv::Point2d> v2;
  std::vector<cv::Point2d> v3;
  std::vector<double> score;
} Corner;

typedef struct Board {
  std::vector<std::vector<int>> idx;
  std::vector<std::vector<std::vector<double>>> energy;
  int num;

  Board()
      : num(0) {}
} Board;

} // namespace cbdetect

#endif //LIBCBDETECT_CONFIG_H
