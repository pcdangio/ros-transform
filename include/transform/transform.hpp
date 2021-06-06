/// \file transform/transform.hpp
/// \brief Defines the transform::transform_t class.
#ifndef TRANSFORM___TRANSFORM_H
#define TRANSFORM___TRANSFORM_H

#include <geometry_msgs_ext/transform.h>

#include <eigen3/Eigen/Dense>

/// \brief Utilities for creating and applying geometric transformations.
namespace transform {

/// \brief Represents a 3D transformation between coordinate frames.
class transform_t
{
public:
    // CONSTRUCTORS
    /// \brief Instantiates a new identity transformation_t object.
    transform_t();
    /// \brief Instantiates a new transformation_t object.
    /// \param translation The translation component of the transformation.
    /// \param rotation The rotation component of the transformation.
    transform_t(const Eigen::Vector3d& translation, const Eigen::Quaterniond& rotation);
    /// \brief Instantiates a new transformation_t object with an identity rotation.
    /// \param translation The translation component of the transformation.
    transform_t(const Eigen::Vector3d& translation);
    /// \brief Instantiates a new transformation_t object with an identity translation.
    /// \param rotation The rotation component of the transformation.
    transform_t(const Eigen::Quaterniond& rotation);
    /// \brief Instantiates a new transformation_t object from a transform message.
    /// \param transform_message The message to instantiate from.
    transform_t(const geometry_msgs_ext::transform& transform_message);

    // MODIFIERS
    /// \brief Calculates the inverse of the transform.
    /// \returns The inverted transform.
    transform_t inverse() const;

    // APPLICATIONS
    /// \brief Performs an in-place chain on top of another transform.
    /// \param transform The transform to chain in place.
    void transform(transform_t& transform) const;
    /// \brief Transforms a 3D point or vector in place.
    /// \param vector The 3D point or vector to transform.
    /// \param rotate_only Indicates if the transform should only perform a rotation.
    void transform(Eigen::Vector3d& vector, bool rotate_only = false) const;
    /// \brief Transforms a 3D pose in place.
    /// \param position The 3D position of the pose.
    /// \param orientation The 3D euler angle representation of the pose orientation.
    void transform(Eigen::Vector3d& position, Eigen::Vector3d& orientation);
    /// \brief Transforms a 3D pose in place.
    /// \param position The 3D position of the pose.
    /// \param orientation The 3D quaternion representation of the pose orientation.
    void transform(Eigen::Vector3d& position, Eigen::Quaterniond& orientation);

    // ACCESS
    /// \brief Gets the translation component of this transform.
    /// \returns A const reference to the translation component.
    const Eigen::Vector3d& translation() const;
    /// \brief Gets the rotation component of this transform.
    /// \returns A const reference to the rotation component.
    const Eigen::Quaterniond& rotation() const;

    // EXPORT
    /// \brief Gets a ROS message representing the transform.
    /// \returns The ROS transform message.
    geometry_msgs_ext::transform to_message() const;
    
private:
    // VARIABLES
    /// \brief The transform's translation component.
    Eigen::Vector3d m_translation;
    /// \brief The transform's rotation component.
    Eigen::Quaterniond m_rotation;
};

}

#endif