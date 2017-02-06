
// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef OPENMVG_SFM_SFM_REGIONS_PROVIDER_HPP
#define OPENMVG_SFM_SFM_REGIONS_PROVIDER_HPP

#include "openMVG/features/image_describer.hpp"
#include "openMVG/features/regions.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/types.hpp"

#include "third_party/progress/progress.hpp"

#include <memory>

namespace openMVG {
namespace sfm {

/// Abstract Regions provider
/// Allow to load and return the regions related to a view
struct Regions_Provider
{
public:

  virtual ~Regions_Provider() = default;

  std::string Type_id()
  {
    if (region_type_)
      return region_type_->Type_id();
    else
    {
      return std::string("initialized regions type");
    }
  }

  bool IsScalar()
  {
    if (region_type_)
      return region_type_->IsScalar();
    else
    {
      std::cerr << "Invalid region type" << std::endl;
      return false;
    }
  }

  bool IsBinary()
  {
    if (region_type_)
      return region_type_->IsBinary();
    else
    {
      std::cerr << "Invalid region type" << std::endl;
      return false;
    }
  }

  const openMVG::features::Regions* getRegionsType() const
  {
    if (region_type_)
      return &(*region_type_);
    else
      return nullptr;
  }

  virtual std::shared_ptr<features::Regions> get(const IndexT x) const
  {
    auto it = cache_.find(x);
    std::shared_ptr<features::Regions> ret;

    if(it == end(cache_))
    {
      // Invalid ressource
    }
    else
    {
      ret = it->second;
    }
    return ret;
  }

  // Load (only for compatibility)
  virtual bool load(
    const SfM_Data & sfm_data,
    const std::string & feat_directory,
    std::unique_ptr<features::Regions> & region_type)
  {
    return load(sfm_data, feat_directory, region_type, false);
  }

  // Load Regions related to a provided SfM_Data View container
  virtual bool load(
    const SfM_Data & sfm_data,
    const std::string & feat_directory,
    std::unique_ptr<features::Regions> & region_type,
    bool features_only,
    C_Progress &my_progress_bar = C_Progress_display(0, std::cout, "\n- Regions Loading -\n"))
  {
    std::set<IndexT> views;

    for (Views::const_iterator iter = sfm_data.views.begin(); iter != sfm_data.views.end(); ++iter)
    {
      views.insert(iter->first);
    }

    return load(sfm_data, views, feat_directory, region_type, features_only, my_progress_bar);
  }

  // Load subset of Regions related to a provided SfM_Data View container
  virtual bool load(
    const SfM_Data & sfm_data,
    const std::set<IndexT> set_ViewIds,
    const std::string & feat_directory,
    std::unique_ptr<features::Regions> & region_type,
    bool features_only = false,
    C_Progress &my_progress_bar = C_Progress_display(0, std::cout, "\n- Regions Loading -\n"))
  {
    std::map<IndexT, std::string> image_names;

    for (std::set<IndexT>::const_iterator iter = set_ViewIds.begin(); iter != set_ViewIds.end(); ++iter)
    {
      const std::string sImageName = stlplus::create_filespec(sfm_data.s_root_path, sfm_data.views.at(*iter)->s_Img_path);
      image_names[*iter] = sImageName;
    }

    return load(image_names, feat_directory, region_type, features_only, my_progress_bar);
  }

  virtual bool load(
    const std::map<IndexT, std::string> & views,
    const std::string & feat_directory,
    std::unique_ptr<features::Regions> & region_type,
    bool features_only = false,
    C_Progress &my_progress_bar = C_Progress_display(0, std::cout, "\n- Regions Loading -\n"))
  {
    my_progress_bar.restart( views.size() );

    region_type_.reset(region_type->EmptyClone());

    // Read for each view the corresponding regions and store them
    bool bContinue = true;
#ifdef OPENMVG_USE_OPENMP
    #pragma omp parallel
#endif
    for (std::map<IndexT, std::string>::const_iterator iter = views.begin();
      iter != views.end() && bContinue; ++iter)
    {
#ifdef OPENMVG_USE_OPENMP
    #pragma omp single nowait
#endif
      {
        const std::string sImageName = iter->second;
        const std::string basename = stlplus::basename_part(sImageName);
        const std::string featFile = stlplus::create_filespec(feat_directory, basename, ".feat");
        const std::string descFile = stlplus::create_filespec(feat_directory, basename, ".desc");

        std::unique_ptr<features::Regions> regions_ptr(region_type->EmptyClone());
        if (!features_only && !regions_ptr->Load(featFile, descFile)
                || features_only && !regions_ptr->LoadFeatures(featFile))
        {
          std::cerr << "Invalid regions files for the view: " << sImageName << std::endl;
#ifdef OPENMVG_USE_OPENMP
        #pragma omp critical
#endif
          bContinue = false;
        }
#ifdef OPENMVG_USE_OPENMP
        #pragma omp critical
#endif
        {
          cache_[iter->first] = std::move(regions_ptr);
          ++my_progress_bar;
        }
      }
    }
    return bContinue;
  }

protected:
  /// Regions per ViewId of the considered SfM_Data container
  mutable Hash_Map<IndexT, std::shared_ptr<features::Regions> > cache_;
  std::unique_ptr<openMVG::features::Regions> region_type_;
}; // Regions_Provider

} // namespace sfm
} // namespace openMVG

#endif // OPENMVG_SFM_SFM_REGIONS_PROVIDER_HPP
