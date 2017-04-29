// Copyright (c) 2017 Mitja Puzigaća.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef OPENMVG_CAMERAS_SHUTTER_CAMERA_HPP
#define OPENMVG_CAMERAS_SHUTTER_CAMERA_HPP

#include "openMVG/cameras/Shutter_Model.h"
#include "openMVG/geometry/pose3.hpp"
#include "openMVG/numeric/numeric.h"

namespace openMVG
{
namespace cameras
{

/**
* @brief Wraps camera type `T` and adds functionality that considers a `AbstractShutterModel` when projecting a point.
*/
template <class T>
class ShutterCamera : public T
{
  using ShutterModel = std::shared_ptr<cameras::AbstractShutterModel>;

private:
  ShutterModel shutter_model_;

public:

  /**
  * @brief Constructor
  * @param shutter_model ShutterModel for camera.
  * @param pose_motion Pose motion for view
  * @param args Arguments to pass to wrapped type T.
  *
  * Example on how `ShutterCamera` can be used.
  * > ShutterCamera<cameras::Pinhole_Intrinsic>(shutter_model, pose_motion, w, h, focal_length_pix, ppx, ppy)
  */
  template <class... Args>
  ShutterCamera(const ShutterModel &shutter_model, Args&&... args)
  : T(std::forward<Args>(args)...)
  , shutter_model_(shutter_model)
  {
    // nothing to do
  }

  virtual Vec2 residual(
    const geometry::Pose3 &pose,
    const Vec3 &X,
    const Vec2 &x) const override
  {
    const Vec2 proj = this->project(pose, X, shutter_model_->getMotionFactor(x));
    return x - proj;
  }

  Vec2 project(
    const geometry::Pose3 & pose,
    const Vec3 & pt3D,
    double motion_factor) const
  {
      return T::project(pose.pose(motion_factor), pt3D);
  }

  virtual Vec2 project(
    const geometry::Pose3 & pose,
    const Vec3 & pt3D) const override
  {
    // find best motion factor such that motion factor from projected point is equal to "guessed" motion factor
    const double epsilon = 1e-6; // criteria to stop the bisection (epsilon_time = epsilon*shutter_readout_time)

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
