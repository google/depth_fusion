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
#include <iostream>
#include <string>
#include <unordered_map>

#include <gflags/gflags.h>
#include "libcgt/camera_wrappers/PoseStream.h"
#include "libcgt/core/io/File.h"
#include "libcgt/core/math/Random.h"
#include "libcgt/core/vecmath/EuclideanTransform.h"
#include "libcgt/core/vecmath/SimilarityTransform.h"
#include "libcgt/core/vecmath/Quat4f.h"
#include "third_party/pystring/pystring.h"
#include "third_party/Eigen/Eigen/Eigen"
#include "third_party/Eigen/Eigen/Geometry"

using libcgt::core::vecmath::compose;
using libcgt::core::vecmath::EuclideanTransform;
using libcgt::core::vecmath::SimilarityTransform;
using libcgt::camera_wrappers::PoseInputStream;
using libcgt::camera_wrappers::PoseStreamFormat;
using libcgt::camera_wrappers::PoseStreamMetadata;
using libcgt::camera_wrappers::PoseStreamTransformDirection;
using libcgt::camera_wrappers::PoseStreamUnits;
using libcgt::camera_wrappers::PoseOutputStream;

DEFINE_string(nvm, "", "Input .nvm file.");
DEFINE_string(input_pose, "", "Input .pose file.");
DEFINE_string(output_pose, "", "Output .pose file.");

// TODO: consider reading two pose files instead.

// TODO: bool?
std::pair<int32_t, int64_t> parseTimestampFromFilename(
  const std::string& filename) {
  std::string basename = pystring::os::path::basename(filename);
  std::vector<std::string> tokens;
  pystring::split(basename, tokens, "_");
  int32_t frame_index = atoi(tokens[tokens.size() - 2].c_str());
  const std::string& timestamp_str = tokens.back();
  int64_t timestamp = static_cast<int64_t>(
    strtoll(timestamp_str.c_str(), nullptr, 10 /* radix */));
  return std::make_pair(frame_index, timestamp);
}

Matrix4f fromEigen( const Eigen::Matrix4f& e )
{
    Matrix4f m;
    memcpy( m.m_elements, e.data(), 16 * sizeof( float ) );
    return m;
}

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  // TODO: validate input and output

  // Load NVM into memory.
  std::vector<std::string> nvm =
    File::readLines(FLAGS_nvm.c_str() );
  int num_nvm_poses_cfw = atoi(nvm[1].c_str());

  // TODO: read filename to index map
  // extract basename out of nvm file
  // extract frame index and timestamp

  // TODO: nvm only has one component.
  // There might be multiple NVM files, one per component.
  // It's possible to stitch them all together.

  EuclideanTransform rot180(
        Matrix3f::rotateX(static_cast<float>(M_PI)));

  // NVM's direction is camera_from_world.
  std::unordered_map<int64_t, EuclideanTransform> nvm_poses_cfw;

  std::vector<std::pair<int32_t, int64_t>> nvm_timestamps;
  for (int i = 0; i < num_nvm_poses_cfw; ++i) {
    const std::string& line = nvm[2 + i];
    std::vector< std::string > tokens;
    pystring::split(line, tokens, " ");

    // Parse filename.
    // TODO: this is a hack and not very robust.
    auto ft = parseTimestampFromFilename(tokens[0]);
    nvm_timestamps.push_back(ft);

    Quat4f q;
    Vector3f c;

    // Parse orientation.
    for (int j = 0; j < 4; ++j) {
        q[j] = static_cast<float>(atof(tokens[2 + j].c_str()));
    }
    // Parse camera center.
    for (int j = 0; j < 3; ++j) {
        c[j] = static_cast<float>(atof(tokens[6 + j].c_str()));
    }

    // Convert translation from SfM convention (R * (x[0:2] - x[3] * c)) to
    // standard convention: [R, t]: R * x[0:2] + t.
    EuclideanTransform e{Matrix3f::fromQuat(q), -q.rotateVector(c)};

    // Convert from y-down (OpenCV convention) to y-up (OpenGL convention).
    // TODO: is this part necessary? Alignment might be able to figure it out.
    nvm_poses_cfw[ft.second] = rot180 * e * rot180;
  }

  PoseInputStream input_pose_stream(FLAGS_input_pose.c_str());
  // TODO: if (input_stream.isValid)...

  std::unordered_map<int64_t, EuclideanTransform> input_poses_cfw;
  int32_t frame_index;
  int64_t timestamp;
  EuclideanTransform e;

  // TODO: check the right format: it might be a quaternion, etc
  // TODO: check that the read succeeds.
  bool ok = input_pose_stream.read(frame_index, timestamp, e.rotation, e.translation);
  while (ok) {
    if (input_pose_stream.metadata().direction ==
      PoseStreamTransformDirection::WORLD_FROM_CAMERA) {
      input_poses_cfw.insert({timestamp, inverse(e)});
    } else {
      input_poses_cfw.insert({timestamp, e});
    }
    ok = input_pose_stream.read(frame_index, timestamp, e.rotation, e.translation);
  }

  // TODO: iterate over which ever is smaller.
  std::vector<Vector3f> input_world_points;
  std::vector<Vector3f> nvm_world_points;

  const Vector3f origin( 0 );
  const Vector3f basis_x( 1, 0, 0 );
  const Vector3f basis_y( 0, 1, 0 );
  const Vector3f basis_z( 0, 0, 1 );

  for (const auto& kvp : input_poses_cfw) {
    int64_t timestamp = kvp.first;
    const EuclideanTransform& input_pose_cfw = kvp.second;
    auto nvm_itr = nvm_poses_cfw.find(timestamp);
    if (nvm_itr != nvm_poses_cfw.end()) {
      const EuclideanTransform& nvm_pose_cfw = nvm_itr->second;

      input_world_points.push_back(transformPoint(inverse(input_pose_cfw), origin));
      nvm_world_points.push_back(transformPoint(inverse(nvm_pose_cfw), origin));
    }
  }

  int num_points = static_cast<int>(input_world_points.size());
  Eigen::MatrixXf input_world_points_matrix(3, num_points);
  Eigen::MatrixXf nvm_world_points_matrix(3, num_points);
  for (int j = 0; j < num_points; ++j) {
    for (int i = 0; i < 3; ++i) {
      input_world_points_matrix(i, j) = input_world_points[j][i];
      nvm_world_points_matrix(i, j) = nvm_world_points[j][i];
    }
  }

  // Now compute the alignment: find nvm_world_from_input_world.
  Eigen::MatrixXf nvm_world_from_input_world_matrix_eigen = Eigen::umeyama(
    input_world_points_matrix /* src */,
    nvm_world_points_matrix /* dst */ );

  std::cout << "umemaya found transformation:\n" <<
    nvm_world_from_input_world_matrix_eigen << "\n";

  Matrix4f nvm_world_from_input_world_matrix =
    fromEigen(nvm_world_from_input_world_matrix_eigen);
  SimilarityTransform nvm_world_from_input_world =
    SimilarityTransform::fromMatrix(nvm_world_from_input_world_matrix);
  std::cout << "solved scale is: " << nvm_world_from_input_world.scale << "\n";
  std::cout << "solved rotation determinant is: " <<
    nvm_world_from_input_world.rotation().determinant() << "\n";

  PoseStreamMetadata output_metadata;
  output_metadata.format = PoseStreamFormat::ROTATION_MATRIX_3X3_COL_MAJOR_AND_TRANSLATION_VECTOR_FLOAT;
  output_metadata.direction = PoseStreamTransformDirection::CAMERA_FROM_WORLD;
  output_metadata.units = input_pose_stream.metadata().units;
  PoseOutputStream pose_output_stream(
    output_metadata,
    FLAGS_output_pose.c_str());

  // Iterate over the NVM poses.
  for (auto ft : nvm_timestamps) {
    int32_t frame_index = ft.first;
    int64_t timestamp = ft.second;
    const EuclideanTransform& nvm_pose_cfw = nvm_poses_cfw[timestamp];

    // x_{nvm,cam} <-- S * x_{input,world}
    SimilarityTransform cam_from_input_world_sim =
      compose(SimilarityTransform(nvm_pose_cfw), nvm_world_from_input_world);

    EuclideanTransform cam_from_input_world_euc(
      cam_from_input_world_sim.rotation(),
      cam_from_input_world_sim.translation() / cam_from_input_world_sim.scale);

    // TODO: write should check that what's coming is is the same as what the
    // header format says.
    pose_output_stream.write(frame_index, timestamp,
      cam_from_input_world_euc.rotation,
      cam_from_input_world_euc.translation);

    const auto& itr = input_poses_cfw.find(timestamp);
    if (itr != input_poses_cfw.end()) {
      const EuclideanTransform& input_pose_cfw = itr->second;
    }
  }

  return 0;
}
