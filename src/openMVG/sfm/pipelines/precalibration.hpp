// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2018 Mitja Puzigaća

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef OPENMVG_SFM_PIPELINES_PRECALIBRATION_HPP
#define OPENMVG_SFM_PIPELINES_PRECALIBRATION_HPP

#include <memory>

#include "openMVG/types.hpp"

namespace openMVG { namespace sfm { struct Features_Provider; }}
namespace openMVG { namespace sfm { struct Matches_Provider; }}
namespace openMVG { namespace sfm { struct SfM_Data; }}

namespace openMVG {
namespace sfm {

class Intrinsic_Precalibration
{
public:
  explicit Intrinsic_Precalibration() = default;

  bool run(
      SfM_Data & sfm_data,
      const Pair_Set & pairs,
      const std::shared_ptr<Features_Provider> & features_provider,
      const std::shared_ptr<Matches_Provider> & matches_provider);
};

} // namespace sfm
} // namespace openMVG

#endif // OPENMVG_SFM_PIPELINES_PRECALIBRATION_HPP
