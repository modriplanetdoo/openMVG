// Copyright (c) 2017 Mitja Puzigaća.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/cameras/Camera_With_Shutter.h"

using namespace openMVG;
using namespace openMVG::cameras;

#include "testing/testing.h"


TEST(RollingShutter, getMotionFactor) {
    cameras::RollingShutter shutter(1000, 1000);

    EXPECT_EQ(shutter.getMotionFactor(Vec2(0,0)), -1.0);
    EXPECT_EQ(shutter.getMotionFactor(Vec2(0, 500)), 0);
    EXPECT_EQ(shutter.getMotionFactor(Vec2(0, 1000)), +1.0);
}


TEST(GlobalShutter, getMotionFactor) {
    cameras::GlobalShutter shutter(1000, 1000);

    EXPECT_EQ(shutter.getMotionFactor(Vec2(0,0)), 0);
    EXPECT_EQ(shutter.getMotionFactor(Vec2(750, 500)), 0);
    EXPECT_EQ(shutter.getMotionFactor(Vec2(250, 1000)), 0);
}

TEST(PoseMotion, center) {
    const double epsilon = 1e-5;

    geometry::Pose3 pose;
    geometry::PoseMotion pose_motion(AngleAxis(), (Vec3(0, 0, 1)).eval());


    EXPECT_MATRIX_NEAR(pose_motion.center(pose, 0), pose.center(), epsilon);

    EXPECT_MATRIX_NEAR(pose_motion.center(pose, -1), (pose.center() - Vec3(0, 0, 1)), epsilon);
    EXPECT_MATRIX_NEAR(pose_motion.center(pose, +1), (pose.center() + Vec3(0, 0, 1)), epsilon);
}

TEST(PoseMotion, rotation) {
    const double epsilon = 1e-5;


    for (int i = 0; i < 10; ++i)
    {
      // generate pose with random rotation
      Vec4 random = Vec4::Random();

      geometry::Pose3 pose(AngleAxis(random(0), random.tail<3>()).toRotationMatrix(), Vec3()); // with random rotation
      geometry::PoseMotion pose_motion(AngleAxis(D2R(90), Vec3::UnitZ()), (Vec3(0, 0, 0)).eval()); // rotate 90°

      EXPECT_MATRIX_NEAR(pose_motion.rotation(pose, 0), (pose.rotation() * Mat3::Identity()).eval(), epsilon);

      EXPECT_MATRIX_NEAR(pose_motion.rotation(pose, -1), (pose.rotation() * (Mat3() << -Vec3::UnitY(),  Vec3::UnitX(), Vec3::UnitZ()).finished()), epsilon);
      EXPECT_MATRIX_NEAR(pose_motion.rotation(pose, +1), (pose.rotation() * (Mat3() <<  Vec3::UnitY(), -Vec3::UnitX(), Vec3::UnitZ()).finished()), epsilon);
    }
}


TEST(Shutter_Camera, RollingShutter_Projection) {
  const double epsilon = 1e-4;

  const unsigned int im_w = 6000;
  const unsigned int im_h = 4000;
  const double im_f = im_w;

  Mat3 R; R << 1, 0, 0, 0, -1, 0, 0, 0, -1; // nadir
  Vec3 C; C << 0,0, 100;                    // 100m above ground
  geometry::Pose3 pose(R, C);

  double shutter_read_time = 33.0 / 1000.0;                         // 33 ms
  AngleAxis R_motion(D2R(-5.0) * shutter_read_time, Vec3::UnitX()); // -5 °/s around X axis
  Vec3 C_motion = Vec3(0, 10, 0) * shutter_read_time;               // 10 m/s towards North
  geometry::PoseMotion pose_motion(R_motion, (C_motion).eval());

  std::shared_ptr<cameras::AbstractShutterModel> shutter_model = std::make_shared<cameras::RollingShutter>(im_w, im_h);
  const ShutterCamera cam(shutter_model, pose_motion, im_w, im_h, im_f, im_w / 2.0, im_h / 2.0);


  for (int i = 0; i < 10; ++i)
  {
    // generate random point inside the world domain
    const Vec3 pt3D = Vec3::Random() * 20;

    Vec2 ptImage = cam.project(pose, pt3D);
    double motion_factor = shutter_model->getMotionFactor(ptImage);
    Vec2 ptImage_ = cam.project(pose, pt3D, motion_factor);

    EXPECT_MATRIX_NEAR(ptImage, ptImage_, epsilon);
  }
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
