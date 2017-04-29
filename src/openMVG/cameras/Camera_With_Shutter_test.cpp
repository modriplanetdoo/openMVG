// Copyright (c) 2017 Mitja Puzigaća.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/cameras/Camera_Pinhole_Radial.hpp"
#include "openMVG/cameras/Camera_With_Shutter.h"
#include "openMVG/cameras/Shutter_Model.h"
#include "openMVG/geometry/Similarity3.hpp"

using namespace openMVG;
using namespace openMVG::cameras;

#include "testing/testing.h"


#define EXPECT_POSE_NEAR(a, b, tolerance) \
do { \
  EXPECT_MATRIX_NEAR(a.center(), b.center(), tolerance) \
  EXPECT_MATRIX_NEAR(a.rotation(), b.rotation(), tolerance); \
} while(false);

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

    geometry::Pose3 pose(Mat3::Identity(), Vec3::Zero(),
                         AngleAxis(), (Vec3(0, 0, 1)).eval());

    EXPECT_MATRIX_NEAR(pose.center(0), pose.center(), epsilon);

    EXPECT_MATRIX_NEAR(pose.center(-1), (pose.center() - Vec3(0, 0, 1)), epsilon);
    EXPECT_MATRIX_NEAR(pose.center(+1), (pose.center() + Vec3(0, 0, 1)), epsilon);
}

TEST(PoseMotion, rotation) {
    const double epsilon = 1e-5;

    for (int i = 0; i < 10; ++i)
    {
      // generate pose with random rotation
      Vec4 random = Vec4::Random();

      geometry::Pose3 pose(AngleAxis(random(0), random.tail<3>().normalized()).toRotationMatrix(), Vec3(), // with random rotation
                           AngleAxis(D2R(90), Vec3::UnitZ()), (Vec3(0, 0, 0)).eval()); // rotate 90°

      EXPECT_MATRIX_NEAR(pose.rotation(0), (pose.rotation() * Mat3::Identity()).eval(), epsilon);

      EXPECT_MATRIX_NEAR(pose.rotation(-1), (pose.rotation() * (Mat3() << -Vec3::UnitY(),  Vec3::UnitX(), Vec3::UnitZ()).finished()), epsilon);
      EXPECT_MATRIX_NEAR(pose.rotation(+1), (pose.rotation() * (Mat3() <<  Vec3::UnitY(), -Vec3::UnitX(), Vec3::UnitZ()).finished()), epsilon);
    }
}

// Testing PoseMotion::pose(const Pose3 &, double) function
TEST(PoseMotion, pose) {
    const double epsilon = 1e-5;

    for (int i = 0; i < 10; ++i)
    {
      // generate pose with random rotation
      Vec4 random_1 = Vec4::Random();
      Vec4 random_2 = Vec4::Random();

      geometry::Pose3 pose(AngleAxis(random_1(0), random_1.tail<3>().normalized()).toRotationMatrix(), Vec3(), // with random rotation
                           AngleAxis(D2R(random_2(0) * 180), random_2.tail<3>().normalized()), (Vec3::Random() * 10).eval()); // random pose motion

      // test rotation
      EXPECT_MATRIX_NEAR(pose.pose( 0).rotation(), pose.rotation( 0), epsilon);
      EXPECT_MATRIX_NEAR(pose.pose(-1).rotation(), pose.rotation(-1), epsilon);
      EXPECT_MATRIX_NEAR(pose.pose(+1).rotation(), pose.rotation(+1), epsilon);

      // test center
      EXPECT_MATRIX_NEAR(pose.pose( 0).center(), pose.center( 0), epsilon);
      EXPECT_MATRIX_NEAR(pose.pose(-1).center(), pose.center(-1), epsilon);
      EXPECT_MATRIX_NEAR(pose.pose(+1).center(), pose.center(+1), epsilon);
    }
}

// Testing Similarity::operator () (const PoseMotion &) function
TEST(PoseMotion, transform) {
    const double epsilon = 1e-5;

    for (int i = 0; i < 1; ++i)
    {
      // generate random similarity, pose and pose_motion
      Vec4 random_1 = Vec4::Random();
      Vec4 random_2 = Vec4::Random();
      Vec6 random_3 = Vec6::Random();

      geometry::Similarity3 similarity(geometry::Pose3(AngleAxis(D2R(random_3(0) * 180), random_3.segment<3>(1).normalized()).toRotationMatrix(), (Vec3::Random() * 100).eval()), random_3(4) + 1.01);

      geometry::Pose3 pose(AngleAxis(D2R(random_1(0) * 180), random_1.tail<3>().normalized()).toRotationMatrix(), Vec3::Random() * 300, // with random rotation
                           AngleAxis(D2R(random_2(0) * 180), random_2.tail<3>().normalized()), (Vec3::Random() * 10).eval()); // random pose motion

      // applying pose_motion to a pose first and then transfroming should yield same results as
      // transforming both pose_motion and pose and then applying transformed_pose_motion to transfromed_pose
      EXPECT_POSE_NEAR(similarity(pose.pose( 0)), similarity(pose).pose( 0), epsilon);
      EXPECT_POSE_NEAR(similarity(pose.pose(-1)), similarity(pose).pose(-1), epsilon);
      EXPECT_POSE_NEAR(similarity(pose.pose(+1)), similarity(pose).pose(+1), epsilon);
    }
}

TEST(Shutter_Camera, RollingShutter_Projection) {
  const double epsilon = 1e-4;

  const unsigned int im_w = 6000;
  const unsigned int im_h = 4000;
  const double im_f = im_w;

  Mat3 R; R << 1, 0, 0, 0, -1, 0, 0, 0, -1; // nadir
  Vec3 C; C << 0,0, 100;                    // 100m above ground

  double shutter_read_time = 33.0 / 1000.0;                         // 33 ms
  AngleAxis R_motion(D2R(-5.0) * shutter_read_time, Vec3::UnitX()); // -5 °/s around X axis
  Vec3 C_motion = Vec3(0, 10, 0) * shutter_read_time;               // 10 m/s towards North

  geometry::Pose3 pose(R, C, R_motion, C_motion);

  std::shared_ptr<cameras::AbstractShutterModel> shutter_model = std::make_shared<cameras::RollingShutter>(im_w, im_h);
  const ShutterCamera<cameras::Pinhole_Intrinsic_Radial_K3> cam(shutter_model, im_w, im_h, im_f, im_w / 2.0, im_h / 2.0, 0.01, 0.03, 0.3);


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
