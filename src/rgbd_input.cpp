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
#include <core/common/ArrayUtils.h>
#include <core/imageproc/ColorMap.h>
#include <core/imageproc/Swizzle.h>

#include "input_buffer.h"

using libcgt::camera_wrappers::RGBDInputStream;
using libcgt::camera_wrappers::kinect1x::rawDepthMapToMeters;
using libcgt::core::arrayutils::copy;
using libcgt::core::arrayutils::flipY;
using libcgt::core::imageproc::linearRemapToLuminance;
using libcgt::core::imageproc::RGBToBGR;

RgbdInput::RgbdInput(InputType input_type, const char* filename) {
  // TODO(jiawen): if type is ...
  file_input_stream_ = std::unique_ptr<RGBDInputStream>(
    new RGBDInputStream(filename));

  // TODO(jiawen): loop over looking for a depth stream and get its resolution.
  //raw_depth_mm_.resize(file_input_stream_->metadata())
  raw_depth_mm_.resize({ 640, 480 });
}

// TODO(jiawen): simplify: read only rgb, etc

// TODO(jiawen): need to pass in serialized RGBDParameters
//  or if it's a real camera, then it's easy

void RgbdInput::read(InputBuffer* buffer,
  bool* rgb_updated, bool* depth_updated) {
  assert(rgb_updated != nullptr);
  assert(depth_updated != nullptr);

  *rgb_updated = false;
  *depth_updated = false;

  // TODO(jiawen): if type is ...
  uint32_t stream_id;
  int64_t timestamp;
  int frame_index;
  Array1DView<const uint8_t> src = file_input_stream_->read(
    stream_id, frame_index, timestamp);

  if (src.notNull()) {
    // HACK: dimensions, stream id
    if (stream_id == 0) {
      // TODO: check stream format.
      Array2DView<const uint8x3> src_rgb(src.pointer(), { 640, 480 });
      RGBToBGR(src_rgb, buffer->color_bgr_ydown.writeView());
      copy(src_rgb, flipY(buffer->color_rgb.writeView()));

      buffer->color_timestamp = timestamp;
      buffer->color_frame_index = frame_index;

      *rgb_updated = true;
    } else if (stream_id == 1) {
      Array2DView<const uint16_t> src_depth(src.pointer(), { 640, 480 });
      copy(src_depth, raw_depth_mm_.writeView());
      rawDepthMapToMeters(raw_depth_mm_.readView(), buffer->depth_meters,
        false);

      buffer->depth_timestamp = timestamp;
      buffer->depth_frame_index = frame_index;

      *depth_updated = true;
    }
  }
}

#if 0
// TODO(jiawen): this is the code for reading the cameras.
// This needs to be cleaned up and supported.
QTimer timer;
timer.setInterval(33);
timer.start();

QObject::connect(&timer, &QTimer::timeout,
  [&]() {
#if USE_KINECT1X
  camera.pollAll(frame);
  // Kinect data is mirrored and BGRA.
  // TODO(jiawen): provide a mirroring option and set it to true
  BGRAToBGR(
    flipX(frame.bgra),
    buffer.raw_color);
  copy<uint16_t>(input_depth_frame,
    flipX(buffer.raw_depth_mm.writeView()));
#endif

#if USE_OPENNI2


#if 1
  printf("polling...");
  camera.pollAll(frame, kWaitMillis);
  printf("frame.colorUpdated = %d, frame.depthUpdated = %d\n",
    frame.colorUpdated, frame.depthUpdated);
  if (frame.colorUpdated) {
    RGBToBGR(frame.rgb, buffer.raw_color.writeView());
  }
  if (frame.depthUpdated) {
    copy(input_depth_frame.readView(), buffer.raw_depth_mm.writeView());
  }
#else
  frame.colorUpdated = false;
  frame.depthUpdated = false;
  uint32_t streamId;
  int frameId;
  int64_t timestamp;
  Array1DView<const uint8_t> data =
    input_stream.read(streamId, frameId, timestamp);
  //if ((data.isNull() || frameId > 10) && !saved) {
  if (data.isNull() && !saved) {
    saved = true;

    TriangleMesh mesh = pipeline.Triangulate();
    mesh.saveOBJ("c:/tmp/foo.obj");
    exit(0);
  }
  if (streamId == 0) {
    Array2DView<const uint8x3> color_frame(data.pointer(), { 640, 480 });
    RGBToBGR(color_frame, buffer.raw_color);
    frame.colorUpdated = true;
  }
  if (streamId == 1) {
    Array2DView<const uint16_t> depth_frame(data.pointer(), { 640, 480 });
    copy(depth_frame, buffer.raw_depth_mm.writeView());
    frame.depthUpdated = true;
  }
#endif
#endif

  // Process color.
  if (frame.colorUpdated) {
    // Flip color upside down.
    copy<uint8x3>(buffer.raw_color, flipY(buffer.color.writeView()));
    copy<uint8x3>(color_vis_view, flipY(buffer.color_vis.writeView()));
  }

  // Process depth.
  if (frame.depthUpdated) {
    libcgt::camera_wrappers::kinect1x::rawDepthMapToMeters(
      buffer.raw_depth_mm, buffer.depth_meters, false);

    linearRemapToLuminance(
      buffer.depth_meters, camera_params.depth_range,
      Range1f::fromMinMax(0.2f, 1.0f), buffer.depth_vis);
  }

  if (frame.depthUpdated) {
#if 0
    if (kDoColorTracking && pose.valid) {
      pipeline.UpdateDepthImage(buffer->depth_meters);
      pipeline.UpdatePoseUsingColorCamera(pose.camera_from_board);
      pipeline.Fuse();

      pipeline.Raycast();
    }
    else {
#endif
        {
          pipeline.UpdateDepthImage(buffer.depth_meters);
          bool icpSucceeded = pipeline.UpdatePoseWithICP();
          printf("ICP succeeded = %d\n", icpSucceeded);
          if (icpSucceeded) {
            pipeline.Fuse();

            pipeline.Raycast();
          }
        }
    }

#if 1
    if (frame.colorUpdated || frame.depthUpdated) {
      main_widget.update();
    }
#endif
  }
  );
#endif
