// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2018 Mitja Puzigaća.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/sfm/pipelines/precalibration.hpp"

#include "openMVG/cameras/Camera_Intrinsics.hpp"
#include "openMVG/multiview/triangulation.hpp"
#include "openMVG/sfm/sfm_data_graph_utils.hpp"
#include "openMVG/sfm/sfm_data_BA_ceres.hpp"
#include "openMVG/sfm/pipelines/relative_pose_engine.hpp"
#include "openMVG/sfm/pipelines/sfm_features_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_matches_provider.hpp"

#include "openMVG/sfm/sfm_data.hpp"

#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

void openMVG::sfm::Intrinsic_Precalibration::run(
    SfM_Data & sfm_data,
    const Pair_Set & pairs,
    const std::shared_ptr<Features_Provider> & features_provider,
    const std::shared_ptr<Matches_Provider> & matches_provider)
{

    std::map<IndexT, Pair_Set> minimum_spanning_trees;
    sfm::PairsToMST(pairs, matches_provider->pairWise_matches_, minimum_spanning_trees);

    Pair_Set all_pairs;
    for (const auto & minimum_spanning_trees_it : minimum_spanning_trees )
    {
        const Pair_Set & minimum_spanning_tree = minimum_spanning_trees_it.second;
        all_pairs.insert(minimum_spanning_tree.begin(), minimum_spanning_tree.end());
    }

    std::cout << "pairs:" << "\n" << "---------" << std::endl;
    for (const auto & pair : all_pairs)
    {
        std::cout << "pair:" << pair.first << " " << pair.second << std::endl;
    }

    Relative_Pose_Engine relative_pose_engine;
    relative_pose_engine.Process(all_pairs, sfm_data, matches_provider.get(), features_provider.get());
    Relative_Pose_Engine::Relative_Pair_Poses relative_poses = relative_pose_engine.Get_Relative_Poses();


    SfM_Data tiny_scene;
    // copy intrinsics without any modifications
    for (const auto & intrinsics_it : sfm_data.intrinsics)
    {
        tiny_scene.intrinsics.insert(intrinsics_it);
    }

    // copy generated views with generated structure and poses
    IndexT id_pose = 0, id_view = 0, id_landmark = 0;
    for (const auto & relative_poses_it : relative_poses)
    {
        const Pair & pair = relative_poses_it.first;
        
        std::shared_ptr<View> view_I_original = sfm_data.views.at(pair.first);
        std::shared_ptr<View> view_J_original = sfm_data.views.at(pair.second);

        const std::shared_ptr<cameras::IntrinsicBase>
          cam_I = sfm_data.intrinsics.at(view_I_original->id_intrinsic),
          cam_J = sfm_data.intrinsics.at(view_J_original->id_intrinsic);

        const Pose3 pose_I; // identity
        const Pose3 & pose_J = relative_poses_it.second;

        // add poses
        const IndexT id_pose_I = id_pose++;
        const IndexT id_pose_J = id_pose++;
        tiny_scene.poses[id_pose_I] = pose_I;
        tiny_scene.poses[id_pose_J] = pose_J;
        
        // add views
        const IndexT id_view_I = id_view++;
        const IndexT id_view_J = id_view++;
        std::shared_ptr<View> view_I = std::make_shared<View>(
            view_I_original->s_Img_path,
            id_view_I,
            view_I_original->id_intrinsic,
            id_pose_I,
            view_I_original->ui_width,
            view_I_original->ui_height);
        std::shared_ptr<View> view_J = std::make_shared<View>(
            view_J_original->s_Img_path,
            id_view_J,
            view_J_original->id_intrinsic,
            id_pose_J,
            view_J_original->ui_width,
            view_J_original->ui_height);
        tiny_scene.views[id_view_I] = view_I;
        tiny_scene.views[id_view_J] = view_J;

 
        // init structure
        const Mat34
          P1 = cam_I->get_projective_equivalent(pose_I),
          P2 = cam_J->get_projective_equivalent(pose_J);
        Landmarks & landmarks = tiny_scene.structure;

        const matching::IndMatches & matches = matches_provider->pairWise_matches_.at(pair);
        for (const matching::IndMatch & match : matches)
        {
            const Vec2
                x1_ = features_provider->feats_per_view.at(view_I_original->id_view)[match.i_].coords().cast<double>(),
                x2_ = features_provider->feats_per_view.at(view_J_original->id_view)[match.j_].coords().cast<double>();
            Vec3 X;
            TriangulateDLT(
                P1, cam_I->get_ud_pixel(x1_).homogeneous(),
                P2, cam_J->get_ud_pixel(x2_).homogeneous(),
                &X);
            Observations obs;
            obs[view_I->id_view] = Observation(x1_, match.i_);
            obs[view_J->id_view] = Observation(x2_, match.j_);
            landmarks[id_landmark].obs = obs;
            landmarks[id_landmark].X = X;

            id_landmark++;
        }
    }

    Bundle_Adjustment_Ceres bundle_adjustment_obj;
    // - refine only Structure and translations
    bundle_adjustment_obj.Adjust
      (
        tiny_scene,
        Optimize_Options(
          cameras::Intrinsic_Parameter_Type::ADJUST_FOCAL_LENGTH | cameras::Intrinsic_Parameter_Type::ADJUST_PRINCIPAL_POINT,
          sfm::Extrinsic_Parameter_Type::ADJUST_TRANSLATION, // Rotations are held as constant
          sfm::Structure_Parameter_Type::ADJUST_ALL,
          Control_Point_Parameter(),
          false)
      );
    bundle_adjustment_obj.Adjust
      (
        tiny_scene,
        Optimize_Options(
          cameras::Intrinsic_Parameter_Type::ADJUST_ALL,
          sfm::Extrinsic_Parameter_Type::ADJUST_TRANSLATION, // Rotations are held as constant
          sfm::Structure_Parameter_Type::ADJUST_ALL,
          Control_Point_Parameter(),
          false)
      );
}
