#ifndef OPENMVG_CAMERAS_CAMERA_PINHOLE_RADIAL_RATIONAL_HPP
#define OPENMVG_CAMERAS_CAMERA_PINHOLE_RADIAL_RATIONAL_HPP

#include "openMVG/cameras/Camera_Pinhole_Radial.hpp"

namespace openMVG
{
namespace cameras
{

/**
* @brief Implement a Pinhole camera with a 3 rational radial distortion coefficients.
* \f$ x_d = x_u (1 + K_1 r) / (1 + K_2 r + K_3 r^2) \f$
*/
class Pinhole_Intrinsic_Radial_K3_Rational_2 : public Pinhole_Intrinsic_Radial_K3
{
  using class_type = Pinhole_Intrinsic_Radial_K3_Rational_2;

  public:

    /**
    * @brief Constructor
    * @param w Width of image
    * @param h Height of image
    * @param focal Focal (in pixel) of the camera
    * @param ppx Principal point on X-Axis
    * @param ppy Principal point on Y-Axis
    * @param k1 First radial distortion coefficient
    * @param k2 Second radial distortion coefficient
    * @param k3 Third radial distortion coefficient
    */
    Pinhole_Intrinsic_Radial_K3_Rational_2(
      int w = 0, int h = 0,
      double focal = 0.0, double ppx = 0, double ppy = 0,
      double k1 = 0.0, double k2 = 0.0, double k3 = 0.0 )
      : Pinhole_Intrinsic_Radial_K3( w, h, focal, ppx, ppy, k1, k2, k3 )
    {
    }

    ~Pinhole_Intrinsic_Radial_K3_Rational_2() override = default;

    /**
    * @brief Tell from which type the embed camera is
    * @retval PINHOLE_CAMERA_RADIAL3_Rational_2
    */
    EINTRINSIC getType() const override
    {
      return PINHOLE_CAMERA_RADIAL3_Rational_2;
    }


    /**
    * @brief Add the distortion field to a point (that is in normalized camera frame)
    * @param p Point before distortion computation (in normalized camera frame)
    * @return point with distortion
    */
    Vec2 add_disto( const Vec2 & p ) const override
    {
      const double & k1 = params_[0], & k2 = params_[1], & k3 = params_[2];

      const double r2 = Square( p( 0 ) ) + Square( p( 1 ) );
      const double r1 = std::sqrt(r2);
      const double r_coeff = ( 1. + k1 * r1) / ( 1. + k2 * r1 + k3 * r2 );

      return ( p * r_coeff );
    }


    /**
     * @brief Add distortion for one coordinate
     * @param x Coordinate on axis
     * @param c Another axis divided by this (c_x = y / x or c_y = x / y)
     * @return Coordinate with distortion
     */
    double add_disto( const double x, const double c ) const
    {
      const double & k1 = params_[0], & k2 = params_[1], & k3 = params_[2];

      if ( !std::isfinite( c ) || x == 0.0 || ( k1 == 0.0 && k2 == 0.0 && k3 == 0.0 ) )
      {
        return x;
      }

      const double p = k1 * ::sqrt( 1. + Square( c ) );
      const double q = k2 * ::sqrt( 1. + Square( c ) );
      const double r = k3 * ( 1. + Square( c ) );
 
      // f(x) = x * (1 + p * abs(x)) / (1 + q * abs(x) + r * x^2)
      return x * ( 1. + p * std::abs( x ) ) / ( 1. + q * std::abs( x ) + r * Square( x ) );
    }

    /**
     * @brief Remove distortion for one coordinate
     * @param x Coordinate on axis
     * @param c Another axis divided by this (c_x = y / x or c_y = x / y)
     * @return Coordinate without distortion
     */
    double remove_disto( const double xd, const double c ) const
    {
      const double & k1 = params_[0], & k2 = params_[1], & k3 = params_[2];

      if ( !std::isfinite( c ) || xd == 0.0 || ( k1 == 0.0 && k2 == 0.0 && k3 == 0.0 ) )
      {
        return xd;
      }

      const double p = k1 * ::sqrt( 1. + Square( c ) );
      const double q = k2 * ::sqrt( 1. + Square( c ) );
      const double r = k3 * ( 1. + Square( c ) );

      // 4 possible solutions
      double x1, x2;
      if ( xd > 0 )
      {
        // solve xd = x * (1 + p * abs(x)) / (1 + q * abs(x) + r * x^2) for x where x > 0
        x1 = ( + ::sqrt( Square( +q * xd - 1. ) - 4. * xd * ( -p + r * xd ) ) - q * xd + 1. ) / ( 2. * ( -p + r * xd ) ); // x > 0
        x2 = ( + ::sqrt( Square( +q * xd - 1. ) - 4. * xd * ( -p + r * xd ) ) + q * xd - 1. ) / ( 2. * ( +p - r * xd ) ); // x > 0
      }
      else
      {
        // solve yd = y * (1 + p * abs(y)) / (1 + q * abs(y) + r * y^2) for y where x < 0
        x1 = ( - ::sqrt( Square( -q * xd - 1. ) - 4. * xd * ( +p + r * xd ) ) + q * xd + 1. ) / ( 2. * ( +p + r * xd ) ); // x < 0
        x2 = ( + ::sqrt( Square( -q * xd - 1. ) - 4. * xd * ( +p + r * xd ) ) + q * xd + 1. ) / ( 2. * ( +p + r * xd ) ); // x < 0
      }

      double x = xd;
      double diff = std::numeric_limits<double>::max();

      // choose x1 or x2, whichever is closest to `xd`
      for ( const double x_ : { x1, x2 } )
      {
        if ( !std::isfinite( x_ ) )
          continue;
        
        const double diff_ = std::abs( xd - x_ );
        if ( diff_ < diff )
        {
          x = x_;
          diff = diff_;
        }
      }

      return x;
    }

    /**
    * @brief Remove the distortion to a camera point (that is in normalized camera frame)
    * @param p Point with distortion
    * @return Point without distortion
    */
    Vec2 remove_disto( const Vec2& pt ) const override
    {
      Vec2 pt_;

      pt_(0) = remove_disto( pt( 0 ), pt( 1 ) / pt( 0 ) );
      pt_(1) = remove_disto( pt( 1 ), pt( 0 ) / pt( 1 ) );

      return pt_; 
    }

    /**
    * @brief Serialization out
    * @param ar Archive
    */
    template <class Archive>
    inline void save( Archive & ar ) const;

    /**
    * @brief  Serialization in
    * @param ar Archive
    */
    template <class Archive>
    inline void load( Archive & ar );

    /**
    * @brief Clone the object
    * @return A clone (copy of the stored object)
    */
    IntrinsicBase * clone( void ) const override
    {
      return new class_type( *this );
    }
};



/**
* @brief Implement a Pinhole camera with a 3 rational radial distortion coefficients.
* \f$ x_d = x_u (1 + K_1 r^2) / (1 + K_2 r + K_3 r^2) \f$
*/
class Pinhole_Intrinsic_Radial_K3_Rational_3 : public Pinhole_Intrinsic_Radial_K3
{
  using class_type = Pinhole_Intrinsic_Radial_K3_Rational_3;

  public:

    /**
    * @brief Constructor
    * @param w Width of image
    * @param h Height of image
    * @param focal Focal (in pixel) of the camera
    * @param ppx Principal point on X-Axis
    * @param ppy Principal point on Y-Axis
    * @param k1 First radial distortion coefficient
    * @param k2 Second radial distortion coefficient
    * @param k3 Third radial distortion coefficient
    */
    Pinhole_Intrinsic_Radial_K3_Rational_3(
      int w = 0, int h = 0,
      double focal = 0.0, double ppx = 0, double ppy = 0,
      double k1 = 0.0, double k2 = 0.0, double k3 = 0.0 )
      : Pinhole_Intrinsic_Radial_K3( w, h, focal, ppx, ppy, k1, k2, k3 )
    {
    }

    ~Pinhole_Intrinsic_Radial_K3_Rational_3() override = default;

    /**
    * @brief Tell from which type the embed camera is
    * @retval PINHOLE_CAMERA_RADIAL3_Rational_3
    */
    EINTRINSIC getType() const override
    {
      return PINHOLE_CAMERA_RADIAL3_Rational_3;
    }


    /**
    * @brief Add the distortion field to a point (that is in normalized camera frame)
    * @param p Point before distortion computation (in normalized camera frame)
    * @return point with distortion
    */
    Vec2 add_disto( const Vec2 & p ) const override
    {
      const double & k1 = params_[0], & k2 = params_[1], & k3 = params_[2];

      const double r2 = Square( p( 0 ) ) + Square( p( 1 ) );
      const double r1 = std::sqrt(r2);
      const double r_coeff = ( 1. + k1 * r2) / ( 1. + k2 * r1 + k3 * r2 );

      return ( p * r_coeff );
    }


    /**
     * @brief Add distortion for one coordinate
     * @param x Coordinate on axis
     * @param c Another axis divided by this (c_x = y / x or c_y = x / y)
     * @return Coordinate with distortion
     */
    double add_disto( const double x, const double c ) const
    {
      const double & k1 = params_[0], & k2 = params_[1], & k3 = params_[2];

      if ( !std::isfinite( c ) || x == 0.0 || ( k1 == 0.0 && k2 == 0.0 && k3 == 0.0 ) )
      {
        return x;
      }

      const double p = k1 * ( 1. + Square( c ) );
      const double q = k2 * ::sqrt( 1. + Square( c ) );
      const double r = k3 * ( 1. + Square( c ) );
 
      // f(x) = x * (1 + p * x^2) / (1 + q * abs(x) + r * x^2)
      return x * ( 1. + p * Square( x ) ) / ( 1. + q * std::abs( x ) + r * Square( x ) );
    }

    /**
     * @brief Remove distortion for one coordinate
     * @param x Coordinate on axis
     * @param c Another axis divided by this (c_x = y / x or c_y = x / y)
     * @return Coordinate without distortion
     */
    double remove_disto( const double xd, const double c ) const
    {
      const double & k1 = params_[0], & k2 = params_[1], & k3 = params_[2];

      if ( !std::isfinite( c ) || xd == 0.0 || ( k1 == 0.0 && k2 == 0.0 && k3 == 0.0 ) )
      {
        return xd;
      }

      const double p = k1 * ( 1. + Square( c ) );
      const double q = k2 * ::sqrt( 1. + Square( c ) );
      const double r = k3 * ( 1. + Square( c ) );

      double x;
      if ( xd < 0 )
      {
        x = std::cbrt( std::sqrt( Square( 27 * p*p * xd - 9 * p * q * r * xd*xd - 9 * p * r * xd + 2 * r*r*r * xd*xd*xd ) + 4 * std::pow( +3 * p * q * xd + 3 * p - r*r * xd*xd, 3 ) ) + 27 * p*p * xd - 9 * p * q * r * xd*xd - 9 * p * r * xd + 2 * r*r*r * xd*xd*xd ) / ( 3 * std::cbrt( 2 ) * p ) - ( std::cbrt( 2 ) * ( +3 * p * q * xd + 3 * p - r*r * xd*xd ) ) / ( 3 * p * std::cbrt( std::sqrt( Square( 27 * p*p * xd - 9 * p * q * r * xd*xd - 9 * p * r * xd + 2 * r*r*r * xd*xd*xd) + 4 * std::pow( +3 * p * q * xd + 3 * p - r*r * xd*xd, 3 ) ) + 27 * p*p * xd - 9 * p * q * r * xd*xd - 9 * p * r * xd + 2 * r*r*r * xd*xd*xd ) ) + ( r * xd ) / ( 3 * p );
      }
      else
      {
        x = std::cbrt( std::sqrt( Square( 27 * p*p * xd + 9 * p * q * r * xd*xd - 9 * p * r * xd + 2 * r*r*r * xd*xd*xd ) + 4 * std::pow( -3 * p * q * xd + 3 * p - r*r * xd*xd, 3 ) ) + 27 * p*p * xd + 9 * p * q * r * xd*xd - 9 * p * r * xd + 2 * r*r*r * xd*xd*xd ) / ( 3 * std::cbrt( 2 ) * p ) - ( std::cbrt( 2 ) * ( -3 * p * q * xd + 3 * p - r*r * xd*xd ) ) / ( 3 * p * std::cbrt( std::sqrt( Square( 27 * p*p * xd + 9 * p * q * r * xd*xd - 9 * p * r * xd + 2 * r*r*r * xd*xd*xd) + 4 * std::pow( -3 * p * q * xd + 3 * p - r*r * xd*xd, 3 ) ) + 27 * p*p * xd + 9 * p * q * r * xd*xd - 9 * p * r * xd + 2 * r*r*r * xd*xd*xd ) ) + ( r * xd ) / ( 3 * p );
      }

      return x;
    }

    /**
    * @brief Remove the distortion to a camera point (that is in normalized camera frame)
    * @param p Point with distortion
    * @return Point without distortion
    */
    Vec2 remove_disto( const Vec2& pt ) const override
    {
      Vec2 pt_;

      pt_(0) = remove_disto( pt( 0 ), pt( 1 ) / pt( 0 ) );
      pt_(1) = remove_disto( pt( 1 ), pt( 0 ) / pt( 1 ) );

      return pt_; 
    }

    /**
    * @brief Serialization out
    * @param ar Archive
    */
    template <class Archive>
    inline void save( Archive & ar ) const;

    /**
    * @brief  Serialization in
    * @param ar Archive
    */
    template <class Archive>
    inline void load( Archive & ar );

    /**
    * @brief Clone the object
    * @return A clone (copy of the stored object)
    */
    IntrinsicBase * clone( void ) const override
    {
      return new class_type( *this );
    }
};

} // namespace cameras
} // namespace openMVG

#endif // #ifndef OPENMVG_CAMERAS_CAMERA_PINHOLE_RADIAL_RATIONAL_HPP
