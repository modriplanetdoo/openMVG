// Copyright (c) 2017 Mitja Puzigaća.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef OPENMVG_CAMERAS_SHUTTER_CAMERA_HPP
#define OPENMVG_CAMERAS_SHUTTER_CAMERA_HPP

#include "openMVG/numeric/numeric.h"
#include "openMVG/cameras/Shutter_Model.h"
#include "openMVG/cameras/Camera_Pinhole.hpp"
#include "openMVG/geometry/pose3.hpp"

namespace openMVG
{
namespace cameras
{

// TODO:
// template <class T>
// class ShutterCamera : public T // T should inherit from cameras::Pinhole_Intrinsic
class ShutterCamera : public cameras::Pinhole_Intrinsic
{
  using ShutterModel = std::shared_ptr<cameras::AbstractShutterModel>;

private:
  ShutterModel shutter_model_;

  // TODO: should not be member
  geometry::PoseMotion pose_motion_;

public:
  ShutterCamera(const ShutterModel &shutter_model, const geometry::PoseMotion &pose_motion,
                    unsigned int w = 0, unsigned int h = 0,
                    double focal_length_pix = 0.0,
                    double ppx = 0.0, double ppy = 0.0 )
  : cameras::Pinhole_Intrinsic( w, h, focal_length_pix, ppx, ppy )
  , shutter_model_(shutter_model), pose_motion_(pose_motion)
  {
    // nothing to do
  }

  virtual Vec2 residual(
    const geometry::Pose3 &pose,
    const Vec3 &X,
    const Vec2 &x) const
  {
    const Vec2 proj = this->project(pose, X, shutter_model_->getMotionFactor(x));
    return x - proj;
  }

  Vec2 project(
    const geometry::Pose3 & pose,
    const Vec3 & pt3D,
    double motion_factor) const
  {
      return Pinhole_Intrinsic::project(pose_motion_.pose(pose, motion_factor), pt3D);
  }

  virtual Vec2 project(
    const geometry::Pose3 & pose,
    const Vec3 & pt3D) const
  {
    // find best motion factor such that motion factor from projected point is equal to "guessed" motion factor
    double epsilon = 1e-6; // criteria to stop the bisection (epsilon_time = epsilon*shutter_readout_time)

    double lowerbound = -1, upbound = +1;
    do
    {
      const double mid = .5 * (lowerbound + upbound);
      const double mid_reverse = shutter_model_->getMotionFactor(project(pose, pt3D, mid));

      if (mid_reverse < mid)
      {
        upbound = mid;
      }
      else
      {
        lowerbound = mid;
      }
    }
    while (epsilon < upbound - lowerbound);

    const double motion_factor = .5 * (lowerbound + upbound);

    return project(pose, pt3D, motion_factor);
  }
};

} // namespace cameras
} // namespace openMVG

#endif // #ifndef OPENMVG_CAMERAS_SHUTTER_CAMERA_HPP
