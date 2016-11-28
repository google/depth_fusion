// Copyright 2016 Google Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http ://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "rgbd_input.h"

#include <cassert>

#include <camera_wrappers/Kinect1x/KinectUtils.h>
#include <camera_wrappers/PixelFormat.h>
#include <core/common/ArrayUtils.h>
#include <core/imageproc/ColorMap.h>
#include <core/imageproc/Swizzle.h>

#include "input_buffer.h"

using libcgt::camera_wrappers::PixelFormat;
using libcgt::camera_wrappers::RGBDInputStream;
using libcgt::camera_wrappers::kinect1x::rawDepthMapToMeters;
using libcgt::core::arrayutils::copy;
using libcgt::core::arrayutils::flipY;
using libcgt::core::imageproc::linearRemapToLuminance;
using libcgt::core::imageproc::RGBToBGR;

RgbdInput::RgbdInput(InputType input_type, const char* filename) :
  input_type_(input_type) {
  if (input_type == InputType::OPENNI2) {
    std::vector<libcgt::camera_wrappers::StreamConfig> config;
    config.emplace_back(
      StreamType::COLOR,
      Vector2i{ 640, 480 }, PixelFormat::RGB_U888, 30,
      false
    );
    config.emplace_back(
      StreamType::DEPTH,
      Vector2i{ 640, 480 }, PixelFormat::DEPTH_MM_U16, 30,
      false
    );
    openni2_camera_ = std::make_unique<OpenNI2Camera>(config);
    openni2_buffer_rgb_.resize(openni2_camera_->colorConfig().resolution);
    openni2_buffer_depth_.resize(openni2_camera_->depthConfig().resolution);
    openni2_frame_.color = openni2_buffer_rgb_.writeView();
    openni2_frame_.depth = openni2_buffer_depth_.writeView();
    openni2_camera_->start();
  } else if (input_type == InputType::FILE) {
    file_input_stream_ = std::make_unique<RGBDInputStream>(filename);

    // Find the rgb stream.
    // TODO: take a dependency on cpp11-range
    for(int i = 0; i < file_input_stream_->metadata().size(); ++i) {
      const auto& metadata = file_input_stream_->metadata()[ i ];
      if(metadata.type == StreamType::COLOR &&
        metadata.format ==  PixelFormat::RGB_U888) {
        color_stream_id_ = i;
        color_metadata_ = metadata;
        break;
      }
    }

    // Find the depth stream.
    // TODO: take a dependency on cpp11-range
    for (int i = 0; i < file_input_stream_->metadata().size(); ++i) {
      const auto& metadata = file_input_stream_->metadata()[i];
      if (metadata.type == StreamType::DEPTH) {
        raw_depth_stream_id_ = i;
        depth_metadata_ = metadata;
        break;
      }
    }
  }
}

Vector2i RgbdInput::colorSize() const {
  return color_metadata_.size;
}

Vector2i RgbdInput::depthSize() const {
  return depth_metadata_.size;
}

// TODO: simplify: read only rgb, etc
// TODO: by default, read all

void RgbdInput::read(InputBuffer* buffer,
  bool* rgb_updated, bool* depth_updated) {
  assert(rgb_updated != nullptr);
  assert(depth_updated != nullptr);

  // TODO: warn if input buffer is the wrong size. Resize it?

  *rgb_updated = false;
  *depth_updated = false;

  if (input_type_ == InputType::OPENNI2) {
    // TODO: if closed, return false

    bool succeeded = openni2_camera_->pollAll(openni2_frame_);
    if (openni2_frame_.colorUpdated) {
      // Copy the buffer, flipping it upside down for OpenGL.
      copy<uint8x3>(openni2_frame_.color, flipY(buffer->color_rgb.writeView()));
      // Convert RGB to BGR for OpenCV.
      RGBToBGR(openni2_frame_.color, buffer->color_bgr_ydown.writeView());
      buffer->color_timestamp_ns = openni2_frame_.colorTimestampNS;
      buffer->color_frame_index = openni2_frame_.colorFrameNumber;
      *rgb_updated = openni2_frame_.colorUpdated;
    }

    if (openni2_frame_.depthUpdated) {
      rawDepthMapToMeters(openni2_frame_.depth, buffer->depth_meters,
        false, true);
      buffer->depth_timestamp_ns = openni2_frame_.depthTimestampNS;
      buffer->depth_frame_index = openni2_frame_.depthFrameNumber;
      *depth_updated = openni2_frame_.depthUpdated;
    }

  } else if(input_type_ == InputType::FILE) {
    uint32_t stream_id;
    int64_t timestamp_ns;
    int32_t frame_index;
    Array1DReadView<uint8_t> src = file_input_stream_->read(
      stream_id, frame_index, timestamp_ns);

    if (src.notNull()) {
      if (stream_id == color_stream_id_) {
        buffer->color_timestamp_ns = timestamp_ns;
        buffer->color_frame_index = frame_index;

        Array2DReadView<uint8x3> src_rgb(
          src.pointer(), color_metadata_.size);
        RGBToBGR(src_rgb, buffer->color_bgr_ydown.writeView());
        bool succeeded = copy(src_rgb, flipY(buffer->color_rgb.writeView()));

        *rgb_updated = succeeded;
      } else if (stream_id == raw_depth_stream_id_ ) {
        buffer->depth_timestamp_ns = timestamp_ns;
        buffer->depth_frame_index = frame_index;

        if (depth_metadata_.format == PixelFormat::DEPTH_MM_U16) {
          Array2DReadView<uint16_t> src_depth(src.pointer(),
            depth_metadata_.size);
          rawDepthMapToMeters(src_depth, buffer->depth_meters,
            false);
          *depth_updated = true;
        } else if(depth_metadata_.format == PixelFormat::DEPTH_M_F32) {
          Array2DReadView<float> src_depth(src.pointer(),
            depth_metadata_.size);
          bool succeeded = copy(src_depth, buffer->depth_meters.writeView());
          *depth_updated = succeeded;
        }
      }
    }
  }
}
