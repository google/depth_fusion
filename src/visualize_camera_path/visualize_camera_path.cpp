#include <vector>

#include <QApplication>

#include "libcgt/camera_wrappers/PoseStream.h"

#include "main_widget.h"
#include "../rgbd_camera_parameters.h"

using libcgt::core::vecmath::EuclideanTransform;
using libcgt::core::vecmath::lerp;

struct PoseFrame {
  int32_t frameIndex;
  int64_t timestamp;
  EuclideanTransform e;
};

std::vector<PoseFrame> LoadPoses(const std::string& filename) {
  libcgt::camera_wrappers::PoseInputStream inputStream(filename.c_str());
  auto metadata = inputStream.metadata();
  std::vector<PoseFrame> poses;
  PoseFrame f;
  bool ok = inputStream.read(f.frameIndex, f.timestamp,
    f.e.rotation, f.e.translation);
  while(ok)
  {
    poses.push_back(f);
    ok = inputStream.read(f.frameIndex, f.timestamp,
      f.e.rotation, f.e.translation);
  }
  return poses;
}

int main(int argc, char* argv[]) {
  QApplication app(argc, argv);

  MainWidget main_widget;
  main_widget.show();

  RGBDCameraParameters rgbd_params = LoadRGBDCameraParameters(
    "D:/tmp/slf/asus_calib_00000");

  auto aruco_poses = LoadPoses("d:/tmp/slf/still_life_00002/aruco.pose");
  auto sfm_aligned_poses = LoadPoses("d:/tmp/slf/still_life_00002/sfm.aligned.pose");

  Vector4f red{ 1, 0, 0, 1 };
  Vector4f green{ 0, 1, 0, 1 };
  Vector4f blue{ 0, 0, 1, 1 };
  Vector4f yellow{ 1, 1, 0, 1 };
  //main_widget.addCameraPath(aruco_poses, rgbd_params.color, red, red);
  //main_widget.addCameraPath(sfm_aligned_poses, rgbd_params.color, blue, blue);

  std::vector<EuclideanTransform> aruco_transforms;
  aruco_transforms.push_back(aruco_poses[0].e);
  aruco_transforms.push_back(aruco_poses[aruco_poses.size() - 1].e);

  std::vector<EuclideanTransform> lerp_path;
  int kNumSamples = 100;
  for (int i = 0; i < kNumSamples; ++i) {
    float t = static_cast<float>(i) / (kNumSamples - 1);
    lerp_path.push_back(lerp(aruco_transforms[0], aruco_transforms[1], t));
  }

#if 0
  for (const auto& f : aruco_poses) {
    aruco_transforms.push_back
  }
#endif

  main_widget.addCameraPath(aruco_transforms, rgbd_params.color, red, red);
  main_widget.addCameraPath(lerp_path, rgbd_params.color, blue, blue);

  return app.exec();
}

#if 0
void load() {
  // How to read.
  libcgt::camera_wrappers::PoseInputStream inputStream(
    "d:/tmp/chairs_sfm/chairs-00000.pose" );
  auto metadata = inputStream.metadata();
  int32_t frameIndex;
  int64_t timestamp;
  std::vector<EuclideanTransform> poses;
  EuclideanTransform e;
  bool ok = inputStream.read( frameIndex, timestamp, e.rotation, e.translation );
  poses.push_back( e );
  while( ok )
  {
    ok = inputStream.read( frameIndex, timestamp, e.rotation, e.translation );
    poses.push_back( e );
  }

  debug_frusta_ = std::vector< Frustum >( poses.size() );
  CameraParameters p = pipeline->GetCameraParameters().color;
  Vector4f c0{ 1, 0, 0, 1 };
  Vector4f c1{ 0, 0, 1, 1 };
  for( int i = 0; i < poses.size(); ++i )
  {
    PerspectiveCamera camera;
    camera.setFrustum( p.intrinsics, p.resolution,
      0.02f, 0.1f );
    camera.setCameraFromWorld( poses[ i ] );

    float frac = static_cast< float >( i ) / ( poses.size() );
    debug_frusta_[ i ].updateColor( libcgt::core::math::lerp( c0, c1, frac ) );
    debug_frusta_[ i ].updatePositions( camera );
  }
}

void SingleMovingCameraGLState::DrawCameraFrustaAndTSDFGrid()
{
  GLSeparableProgram& vs = programs_[ "drawColorVS" ];
  vs.setUniformMatrix4f( 0, free_camera_.viewProjectionMatrix() );
  draw_color_.bind();

  tracked_rgb_camera_.draw();
  tracked_depth_camera_.draw();
  tsdf_bbox_.draw();

  for( int i = 0; i < debug_frusta_.size(); ++i )
  {
    debug_frusta_[ i ].draw();
  }

  GLProgramPipeline::unbindAll();
}
#endif
