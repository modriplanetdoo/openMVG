// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef OPENMVG_GEOMETRY_POSE3_HPP
#define OPENMVG_GEOMETRY_POSE3_HPP

#include "openMVG/multiview/projection.hpp"
#include <cereal/cereal.hpp> // Serialization

namespace openMVG
{
namespace geometry
{


/**
* @brief Defines a pose in 3d space
* [R|C] t = -RC
*/
class Pose3
{
  protected:

    /// Orientation matrix
    Mat3 rotation_;

    /// Center of rotation
    Vec3 center_;

    /// Motion rotation
    AngleAxis incremental_rotation_;
    /// Motion translation
    Vec3 incremental_translation_;

  public:

    /**
    * @brief Default constructor
    * @note This defines a Null transform (aligned with cartesian frame, centered at origin)
    */
    Pose3()
      : rotation_( Mat3::Identity() ),
        center_( Vec3::Zero() ),
        incremental_rotation_(0, Vec3::UnitX()),
        incremental_translation_(Vec3::Zero())
    {

    }

    /**
    * @brief Constructor
    * @param r Rotation
    * @param c Center
    */
    Pose3( const Mat3& r, const Vec3& c )
        : rotation_( r )
        , center_( c )
        , incremental_rotation_(0, Vec3::UnitX())
        , incremental_translation_(Vec3::Zero())
    {

    }

    /**
    * @brief Constructor
    * @param r Rotation
    * @param c Center
    * @param m_r Motion rotation
    * @param m_t Motion translation
    */
    Pose3( const Mat3& r, const Vec3& c, const AngleAxis& m_r, const Vec3& m_t ) : rotation_( r ), center_( c ), incremental_rotation_( m_r ), incremental_translation_( m_t ) {}

    /**
    * @brief Get Rotation matrix
    * @return Rotation matrix
    */
    const Mat3& rotation() const
    {
      return rotation_;
    }

    /**
    * @brief Get Rotation matrix
    * @return Rotation matrix
    */
    Mat3& rotation()
    {
      return rotation_;
    }

    /**
    * @brief Get center of rotation
    * @return Center of rotation
    */
    const Vec3& center() const
    {
      return center_;
    }

    /**
    * @brief Get center of rotation
    * @return Center of rotation
    */
    Vec3& center()
    {
      return center_;
    }

    /**
    * @brief Get translation vector
    * @return translation vector
    * @note t = -RC
    */
    inline Vec3 translation() const
    {
      return -( rotation_ * center_ );
    }

    Mat3 rotation(double motion_factor) const {
      return rotation() * AngleAxis(motion_factor * incremental_rotation_.angle(), incremental_rotation_.axis()).toRotationMatrix();
    }

    Vec3 center(double motion_factor) const {
      return center() + (incremental_translation_ * motion_factor);
    }

    Vec3 translation(double motion_factor) const {
      return -( rotation(motion_factor) * center(motion_factor) );
    }

    Pose3 pose(double motion_factor) const {
      return Pose3(rotation(motion_factor), center(motion_factor));
    }

    const AngleAxis &getIncrementalRotation() const {
      return incremental_rotation_;
    }

    const Vec3 &getIncrementalTranslation() const {
      return incremental_translation_;
    }

    bool hasMotion() const {
        return !incremental_rotation_.isApprox(AngleAxis()) || !incremental_translation_.isApprox(Vec3::Zero());
    }


    /**
    * @brief Apply pose
    * @param p Point
    * @return transformed point
    */
    inline Mat3X operator () ( const Mat3X& p ) const
    {
      return rotation_ * ( p.colwise() - center_ );
    }

//    // NOT USED ANYWHERE
//    /**
//    * @brief Composition of poses
//    * @param P a Pose
//    * @return Composition of current pose and parameter pose
//    */
//    Pose3 operator * ( const Pose3& P ) const
//    {
//      return Pose3( rotation_ * P.rotation_, P.center_ + P.rotation_.transpose() * center_ );
//    }


    /**
    * @brief Get inverse of the pose
    * @return Inverse of the pose
    *
    * @note inverted pose has no motion
    */
    Pose3 inverse() const
    {
      // inverse for pose motion does not exist so we return plain pose

      return Pose3( rotation_.transpose(),  -( rotation_ * center_ ) );
    }


    /**
    * @brief Return the depth (distance) of a point respect to the camera center
    * @param X Input point
    * @return Distance to center
    */
    double depth( const Vec3 &X ) const
    {
      return ( rotation_ * ( X - center_ ) )[2];
    }

    /**
    * Serialization out
    * @param ar Archive
    */
    template <class Archive>
    void save( Archive & ar ) const
    {
      const std::vector<std::vector<double>> mat =
      {
        { rotation_( 0, 0 ), rotation_( 0, 1 ), rotation_( 0, 2 ) },
        { rotation_( 1, 0 ), rotation_( 1, 1 ), rotation_( 1, 2 ) },
        { rotation_( 2, 0 ), rotation_( 2, 1 ), rotation_( 2, 2 ) }
      };

      ar( cereal::make_nvp( "rotation", mat ) );

      const std::vector<double> vec = { center_( 0 ), center_( 1 ), center_( 2 ) };
      ar( cereal::make_nvp( "center", vec ) );
    }

    /**
    * @brief Serialization in
    * @param ar Archive
    */
    template <class Archive>
    void load( Archive & ar )
    {
      std::vector<std::vector<double>> mat( 3, std::vector<double>( 3 ) );
      ar( cereal::make_nvp( "rotation", mat ) );
      // copy back to the rotation
      rotation_.row( 0 ) = Eigen::Map<const Vec3>( &( mat[0][0] ) );
      rotation_.row( 1 ) = Eigen::Map<const Vec3>( &( mat[1][0] ) );
      rotation_.row( 2 ) = Eigen::Map<const Vec3>( &( mat[2][0] ) );

      std::vector<double> vec( 3 );
      ar( cereal::make_nvp( "center", vec ) );
      center_ = Eigen::Map<const Vec3>( &vec[0] );
    }
};



////------------------
////-- Bibliography --
////------------------
////- [1] "Photogrammetric Accuracy and Modeling of Rolling Shutter Cameras."
////- Authors: Jonas Vautherin, Simon Rutishauser, Klaus Schneider-Zapp, Hon Fai Choi Venera Chovancova, Alexis Glass, Christoph Strecha.
////- Date: June 2016.
////- Publication : ISPRS Annals of Photogrammetry, Remote Sensing and Spatial Information Sciences, Volume III-3, 2016, pp.139-146.
////------

///**
//* @brief Defines a pose motion in 3D space. The amount of motion applied depends on `motion_factor`.
//* `motion_factor` of `0` means that there is no motion to be applied.
//*
//* Well known pinhole camera matrix `[R|C]` is here considere to be more like `[R(t)|C(t)]`.
//* where:
//* > R(t) = R0 · λ ∆R
//* > c(t) = c0 + λ ∆c
//* > λ ∈ [−1, +1]
//*
//* Note:
//* - All rotations and translations are in world frame
//*/
//class PoseMotion {
//private:
//  AngleAxis incremental_rotation_;
//  Vec3 incremental_translation_;

//public:
//  PoseMotion(const AngleAxis &incremental_rotation, const Vec3 &incremental_translation)
//    : incremental_rotation_(incremental_rotation), incremental_translation_(incremental_translation)
//  {
//    // nothing to do
//  }

//  Mat3 rotation(const Pose3 &pose, double motion_factor) const {
//    return pose.rotation() * AngleAxis(motion_factor * incremental_rotation_.angle(), incremental_rotation_.axis()).toRotationMatrix();
//  }

//  Vec3 center(const Pose3 &pose, double motion_factor) const {
//    return pose.center() + (incremental_translation_ * motion_factor);
//  }

//  Pose3 pose(const Pose3 &pose, double motion_factor) const {
//    return Pose3(rotation(pose, motion_factor), center(pose, motion_factor));
//  }

////  PoseMotion inverse() const {
////    return PoseMotion(incremental_rotation_.inverse(), -incremental_translation_);
////  }

//  const AngleAxis &getIncrementalRotation() const {
//      return incremental_rotation_;
//  }

//  const Vec3 &getIncrementalTranslation() const {
//      return incremental_translation_;
//  }

//};

} // namespace geometry
} // namespace openMVG

#endif  // OPENMVG_GEOMETRY_POSE3_HPP
