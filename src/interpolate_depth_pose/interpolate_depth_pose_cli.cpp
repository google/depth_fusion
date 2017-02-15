#include <vector>

#include <gflags/gflags.h>
#include "libcgt/camera_wrappers/PoseStream.h"
#include "libcgt/camera_wrappers/RGBDStream.h"
#include "libcgt/core/math/MathUtils.h"
#include "libcgt/core/vecmath/EuclideanTransform.h"

#include "../rgbd_camera_parameters.h"

using libcgt::camera_wrappers::PoseInputStream;
using libcgt::camera_wrappers::PoseOutputStream;
using libcgt::camera_wrappers::PoseStreamFormat;
using libcgt::camera_wrappers::PoseStreamMetadata;
using libcgt::camera_wrappers::PoseStreamTransformDirection;
using libcgt::camera_wrappers::PoseStreamUnits;
using libcgt::camera_wrappers::RGBDInputStream;
using libcgt::core::math::fraction;
using libcgt::core::vecmath::EuclideanTransform;
using libcgt::core::vecmath::lerp;

DEFINE_string(calibration_dir, "",
  "calibration directory for the RGBD camera.");

DEFINE_string(reference_pose, "", "Reference camera path file (.pose).");

DEFINE_string(input_rgbd, "", "Input RGBD stream file (.rgbd) whose depth"
  " positions should be interpolated.");

DEFINE_string(output_merged_pose, "", "Filename (.pose) for merged output"
  " stream.");

struct PoseFrame {
  int32_t frame_index;
  int64_t timestamp;
  EuclideanTransform e;
};

bool less(const PoseFrame& x, const PoseFrame& y) {
  return x.timestamp < y.timestamp;
}

std::vector<PoseFrame> LoadPoses(const std::string& filename,
  PoseStreamMetadata& metadata) {
  PoseInputStream inputStream(filename.c_str());
  metadata = inputStream.metadata();
  std::vector<PoseFrame> poses;
  PoseFrame f;
  bool ok = inputStream.read(f.frame_index, f.timestamp,
    f.e.rotation, f.e.translation);
  while(ok)
  {
    poses.push_back(f);
    ok = inputStream.read(f.frame_index, f.timestamp,
      f.e.rotation, f.e.translation);
  }
  return poses;
}

std::vector<std::pair<int32_t, int64_t>> LoadDepthTimestamps(const std::string& rgbd_filename) {
  std::vector<std::pair<int32_t, int64_t>> output;
  RGBDInputStream stream(rgbd_filename.c_str());

  // Find metadata stream id.
  int depth_stream_id = -1;
  for (size_t i = 0; i < stream.metadata().size(); ++i) {
    if (stream.metadata()[i].type == StreamType::DEPTH) {
      depth_stream_id = static_cast<int>(i);
    }
  }

  if (depth_stream_id == -1) {
    return output;
  }

  uint32_t stream_id;
  int32_t frame_index;
  int64_t timestamp;
  while(stream.read(stream_id, frame_index, timestamp).notNull()) {
    if (stream_id == depth_stream_id) {
      output.push_back(std::make_pair(frame_index, timestamp));
    }
  }

  return output;
}

float fraction(int64_t x, int64_t lo, int64_t size) {
  return static_cast<float>(x - lo) / size;
}

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  if (FLAGS_calibration_dir == "") {
    printf("calibration_dir is required.\n");
    return 1;
  }
  if (FLAGS_reference_pose == "") {
    printf("reference_pose is required.\n");
  }
  if (FLAGS_reference_pose == "") {
    printf("reference_pose is required.\n");
  }
  if (FLAGS_input_rgbd == "") {
    printf("input_rgbd is required.\n");
    return 1;
  }
  if (FLAGS_output_merged_pose == "") {
    printf("output_merged_pose is required.\n");
    return 1;
  }

  RGBDCameraParameters params =
    LoadRGBDCameraParameters(FLAGS_calibration_dir);
  PoseStreamMetadata input_metadata;
  auto sfm_aligned_poses = LoadPoses(FLAGS_reference_pose, input_metadata);
  int num_sfm_poses = static_cast<int>(sfm_aligned_poses.size());

  std::vector<int64_t> sfm_aligned_timestamps(sfm_aligned_poses.size());
  for (size_t i = 0; i < sfm_aligned_poses.size(); ++i) {
    sfm_aligned_timestamps[i] = sfm_aligned_poses[i].timestamp;
  }

  auto depth_timestamps = LoadDepthTimestamps(FLAGS_input_rgbd);
  int num_depth_timestamps = static_cast<int>(depth_timestamps.size());

  std::vector<PoseFrame> depth_poses;
  for (const auto& ft : depth_timestamps) {
    PoseFrame depth_pose;
    depth_pose.frame_index = ft.first;
    depth_pose.timestamp = ft.second;

    // Find the pair of timestamps in sfm_aligned_poses bracketing
    // depth_timestamp.
    int lbIndex = static_cast<int>(
      std::lower_bound(sfm_aligned_timestamps.begin(),
                       sfm_aligned_timestamps.end(),
                       depth_pose.timestamp) -
      sfm_aligned_timestamps.begin());

    // Lower bound is past the edge, do nothing.
    if (lbIndex == num_sfm_poses) {
      continue;
    }

    int64_t lbt = sfm_aligned_timestamps[lbIndex];

    // Exact match: do nothing - no need to lerp since it will already exist.
    if (lbt == depth_pose.timestamp) {
      continue;
    }

    // Lower bound is not an exact match and it's the first index. This means
    // there's no earlier frame from which to interpolate, do nothing.
    if (lbIndex == 0) {
      continue;
    }

    // Lerp.
    int64_t t0 = sfm_aligned_timestamps[lbIndex - 1];
    int64_t t1 = lbt;
    float f = fraction(depth_pose.timestamp, t0, t1 - t0);
    PoseFrame p0 = sfm_aligned_poses[lbIndex - 1];
    PoseFrame p1 = sfm_aligned_poses[lbIndex];

    depth_pose.e = lerp(p0.e, p1.e, f);
    depth_poses.push_back(depth_pose);
  }

  std::vector<PoseFrame> merged_poses(num_sfm_poses + depth_poses.size());
  std::merge(sfm_aligned_poses.begin(), sfm_aligned_poses.end(),
    depth_poses.begin(), depth_poses.end(),
    merged_poses.begin(), less);

  PoseStreamMetadata output_metadata = input_metadata;
  PoseOutputStream output_stream(output_metadata, FLAGS_output_merged_pose);
  for (PoseFrame p : merged_poses) {
    output_stream.write(p.frame_index, p.timestamp,
      p.e.rotation, p.e.translation);
  }
}
