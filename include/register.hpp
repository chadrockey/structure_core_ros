
/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

// Adapted from image_pipeline/depth_image_proc/src/nodelets/register.cpp

#include <Eigen/Geometry>
#include <ST/CaptureSession.h>

void register_convert(const ST::DepthFrame& depth_frame, const ST::ColorFrame& color_frame, std::vector<uint8_t>& output)
{
  // Allocate memory for registered depth image
  output.resize(2*color_frame.width()*color_frame.height(), 0); // Allocate as all zeroes

  // Extract all the parameters we need
  double inv_depth_fx = 1.0 / depth_frame.intrinsics().fx;
  double inv_depth_fy = 1.0 / depth_frame.intrinsics().fy;
  double depth_cx = depth_frame.intrinsics().cx, depth_cy = depth_frame.intrinsics().cy;
  double depth_Tx = 0.0, depth_Ty = 0.0;
  double rgb_fx = color_frame.intrinsics().fx, rgb_fy = color_frame.intrinsics().fy;
  double rgb_cx = color_frame.intrinsics().cx, rgb_cy = color_frame.intrinsics().cy;
  double rgb_Tx = 0.0, rgb_Ty = 0.0;

  Eigen::Matrix3d rot;
  const ST::Matrix4 pose = depth_frame.visibleCameraPoseInDepthCoordinateFrame();
  for(size_t j = 0; j < 4; j++)
  {
    for(size_t i = 0; i < 4; i++){
      rot(j, i) = pose.atRowCol(j, i);
    }
  }

  Eigen::Affine3d depth_to_rgb = Eigen::Translation3d(pose.atRowCol(0,3),
                                                      pose.atRowCol(1,3),
                                                      pose.atRowCol(2,3)) *
                                         Eigen::Quaterniond(rot);

  
  // Transform the depth values into the RGB frame
  /// @todo When RGB is higher res, interpolate by rasterizing depth triangles onto the registered image  
  const float* depth_row = depth_frame.depthInMillimeters();
  int row_step = depth_frame.width();
  uint16_t* registered_data = reinterpret_cast<uint16_t*>(output.data());
  int raw_index = 0;
  for (unsigned v = 0; v < depth_frame.height(); ++v, depth_row += row_step)
  {
    for (unsigned u = 0; u < depth_frame.width(); ++u, ++raw_index)
    {
      float raw_depth = depth_row[u];
      if (!std::isfinite(raw_depth))
        continue;
      
      double depth = raw_depth/1000.0;

      /// @todo Combine all operations into one matrix multiply on (u,v,d)
      // Reproject (u,v,Z) to (X,Y,Z,1) in depth camera frame
      Eigen::Vector4d xyz_depth;
      xyz_depth << ((u - depth_cx)*depth - depth_Tx) * inv_depth_fx,
                   ((v - depth_cy)*depth - depth_Ty) * inv_depth_fy,
                   depth,
                   1;

      // Transform to RGB camera frame
      Eigen::Vector4d xyz_rgb = depth_to_rgb * xyz_depth;

      // Project to (u,v) in RGB image
      double inv_Z = 1.0 / xyz_rgb.z();
      int u_rgb = (rgb_fx*xyz_rgb.x() + rgb_Tx)*inv_Z + rgb_cx + 0.5;
      int v_rgb = (rgb_fy*xyz_rgb.y() + rgb_Ty)*inv_Z + rgb_cy + 0.5;
      
      if (u_rgb < 0 || u_rgb >= (int)color_frame.width() ||
          v_rgb < 0 || v_rgb >= (int)color_frame.height())
        continue;
      
      uint16_t& reg_depth = registered_data[v_rgb*color_frame.width() + u_rgb];
      uint16_t new_depth = 1000.0*xyz_rgb.z();
      // Validity and Z-buffer checks
      if (reg_depth == 0 || reg_depth > new_depth)
        reg_depth = new_depth;
    }
  }
}

// If you want a depth_frame exactly aligned and same resolution as infrared frame
void register_convert(const ST::DepthFrame& depth_frame, const ST::InfraredFrame& ir_frame, std::vector<uint8_t>& output)
{
  // Allocate memory for registered depth image
  output.resize(2*ir_frame.width()/2*ir_frame.height(), 0); // Allocate as all zeroes

  // Extract all the parameters we need
  double inv_depth_fx = 1.0 / depth_frame.intrinsics().fx;
  double inv_depth_fy = 1.0 / depth_frame.intrinsics().fy;
  double depth_cx = depth_frame.intrinsics().cx, depth_cy = depth_frame.intrinsics().cy;
  double depth_Tx = 0.0, depth_Ty = 0.0;
  double rgb_fx = ir_frame.intrinsics().fx, rgb_fy = ir_frame.intrinsics().fy;
  double rgb_cx = ir_frame.intrinsics().cx, rgb_cy = ir_frame.intrinsics().cy;
  double rgb_Tx = 0.0, rgb_Ty = 0.0;

  Eigen::Affine3d depth_to_rgb = Eigen::Translation3d(0,
                                                      0,
                                                      0) *
                                         Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);

  
  // Transform the depth values into the RGB frame
  /// @todo When RGB is higher res, interpolate by rasterizing depth triangles onto the registered image  
  const float* depth_row = depth_frame.depthInMillimeters();
  int row_step = depth_frame.width();
  uint16_t* registered_data = reinterpret_cast<uint16_t*>(output.data());
  int raw_index = 0;
  for (unsigned v = 0; v < depth_frame.height(); ++v, depth_row += row_step)
  {
    for (unsigned u = 0; u < depth_frame.width(); ++u, ++raw_index)
    {
      float raw_depth = depth_row[u];
      if (!std::isfinite(raw_depth))
        continue;
      
      double depth = raw_depth/1000.0;

      /// @todo Combine all operations into one matrix multiply on (u,v,d)
      // Reproject (u,v,Z) to (X,Y,Z,1) in depth camera frame
      Eigen::Vector4d xyz_depth;
      xyz_depth << ((u - depth_cx)*depth - depth_Tx) * inv_depth_fx,
                   ((v - depth_cy)*depth - depth_Ty) * inv_depth_fy,
                   depth,
                   1;

      // Transform to RGB camera frame
      Eigen::Vector4d xyz_rgb = depth_to_rgb * xyz_depth;

      // Project to (u,v) in RGB image
      double inv_Z = 1.0 / xyz_rgb.z();
      int u_rgb = (rgb_fx*xyz_rgb.x() + rgb_Tx)*inv_Z + rgb_cx + 0.5;
      int v_rgb = (rgb_fy*xyz_rgb.y() + rgb_Ty)*inv_Z + rgb_cy + 0.5;
      
      if (u_rgb < 0 || u_rgb >= (int)ir_frame.width()/2 ||
          v_rgb < 0 || v_rgb >= (int)ir_frame.height())
        continue;
      
      uint16_t& reg_depth = registered_data[v_rgb*ir_frame.width()/2 + u_rgb];
      uint16_t new_depth = 1000.0*xyz_rgb.z();
      // Validity and Z-buffer checks
      if (reg_depth == 0 || reg_depth > new_depth)
        reg_depth = new_depth;
    }
  }
}