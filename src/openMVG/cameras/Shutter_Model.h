// Copyright (c) 2017 Mitja Puzigaća.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef OPENMVG_CAMERAS_SHUTTER_MODEL_HPP
#define OPENMVG_CAMERAS_SHUTTER_MODEL_HPP

#include "openMVG/numeric/numeric.h"

namespace openMVG
{
namespace cameras
{

/**
* @brief `AbstractShutterModel` is a base class for different types of shutter. It's main responsibility is to model how data is read for sensor.
* This is important since not all sensor types read data from every pixel at the same time. This model aims to map from pixel position to time factor when pixel data was read from sensor.
*/
class AbstractShutterModel {

public:
  AbstractShutterModel(unsigned int w, unsigned int h) : w_(w), h_(h) {
    // nothing to do
  }

  /**
  * @brief Get the motion factor for PoseMotion. `-1` represents the time when sensor starts the read-out and `+1` when the sensor is done with reading from sensor.
  * @return Motion factor [-1, +1]
  */
  virtual double getMotionFactor(const openMVG::Vec2 &x) const = 0;

  /// Width of image
  unsigned int w_;
  /// Height of image
  unsigned int h_;
};


/**
* @brief Electronic or mechanical rolling shutter
*/
class RollingShutter : public AbstractShutterModel  {
public:
  RollingShutter(unsigned int w, unsigned int h) : AbstractShutterModel(w, h) {
    // nothing to do
  }

  virtual double getMotionFactor(const openMVG::Vec2 &x) const override {
    const double row = x(1);

    return row / h_ * 2 - 1;
  }
};

/**
* @brief Electronic global shutter
*/
class GlobalShutter : public AbstractShutterModel  {
public:
  GlobalShutter(unsigned int w, unsigned int h) : AbstractShutterModel(w, h) {
    // nothing to do
  }

  virtual double getMotionFactor(const openMVG::Vec2 &x) const override {
    return 0.0;
  }
};
} // namespace cameras
} // namespace openMVG

#endif // #ifndef OPENMVG_CAMERAS_SHUTTER_MODEL_HPP
