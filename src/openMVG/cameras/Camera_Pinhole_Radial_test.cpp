// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/cameras/Camera_Pinhole_Radial.hpp"
#include "openMVG/cameras/Camera_Pinhole_Radial_Rational.hpp"
using namespace openMVG;
using namespace openMVG::cameras;

#include "testing/testing.h"
#include "openMVG/cameras/Camera_Unit_Test.inl"

TEST(Cameras_Radial, disto_undisto_K1) {

  const Pinhole_Intrinsic_Radial_K1 cam(1000, 1000, 1000, 500, 500,
    // K1
    0.1);

  Test_camera(cam);
}

TEST(Cameras_Radial, disto_undisto_K3) {

  const Pinhole_Intrinsic_Radial_K3 cam(1000, 1000, 1000, 500, 500,
    // K1, K2, K3
    -0.245539, 0.255195, 0.163773);

  Test_camera(cam);
}


TEST(Cameras_Radial, disto_undisto_K3_Rational_2) {

  const Pinhole_Intrinsic_Radial_K3_Rational_2 cam(1000, 1000, 1000, 500, 500,
    // K1, K2, K3
    -1.26207, -1.25222, -0.0153429);

  Test_camera(cam);
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
