#include <transform/transform.hpp>

using namespace transform;

// CONSTRUCTORS
transform_t::transform_t()
{
    transform_t::m_translation.setZero();
    transform_t::m_rotation.setIdentity();
}
transform_t::transform_t(const Eigen::Vector3d& translation, const Eigen::Quaterniond& rotation)
{
    transform_t::m_translation = translation;
    transform_t::m_rotation = rotation;
}
transform_t::transform_t(const Eigen::Vector3d& translation)
{
    transform_t::m_translation = translation;
    transform_t::m_rotation.setIdentity();
}
transform_t::transform_t(const Eigen::Quaterniond& rotation)
{
    transform_t::m_translation.setZero();
    transform_t::m_rotation = rotation;
}
transform_t::transform_t(const geometry_msgs_ext::transform& transform_message)
{
    transform_t::m_translation = {transform_message.translation.x, transform_message.translation.y, transform_message.translation.z};
    transform_t::m_rotation = {transform_message.rotation.qw, transform_message.rotation.qx, transform_message.rotation.qy, transform_message.rotation.qz};
}

// MODIFIERS
transform_t transform_t::inverse() const
{
    // Create output transform.
    // NOTE: This method allows Eigen to do operations in place.
    transform_t inverted;

    // Invert the translation.
    inverted.m_translation = transform_t::m_translation * -1.0;

    // Invert the rotation.
    inverted.m_rotation = transform_t::m_rotation.inverse();

    return inverted;
}

// APPLICATIONS
void transform_t::rotate(Eigen::Vector3d& vector) const
{
    // Apply this transform's rotation to the original vector.
    vector = transform_t::m_rotation * vector;
}
void transform_t::translate(Eigen::Vector3d& vector) const
{
    // Add the transform's translation to the rotated vector.
    vector += transform_t::m_translation;
}
void transform_t::transform(transform_t& transform) const
{
    // Apply this transform's rotation to the original transform.
    transform.m_rotation = transform_t::m_rotation * transform.m_rotation;
    transform.m_translation = transform_t::m_rotation * transform.m_translation;
    // Normalize the transformed rotation for numerical stability.
    transform.m_rotation.normalize();

    // Add this transform's translation to the now rotated original translation.
    transform.m_translation += transform_t::m_translation;
}
void transform_t::transform(Eigen::Vector3d& vector) const
{
    // Apply this transform's rotation to the original vector.
    vector = transform_t::m_rotation * vector;

    // Add the transform's translation to the rotated vector.
    vector += transform_t::m_translation;
}
void transform_t::transform(Eigen::Vector3d& position, Eigen::Vector3d& orientation)
{
    // Convert the orientation to a quaternion.
    Eigen::Quaterniond q = Eigen::AngleAxisd(orientation(0), Eigen::Vector3d::UnitX()) *
                           Eigen::AngleAxisd(orientation(1), Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(orientation(2), Eigen::Vector3d::UnitZ());

    // Transform the pose with it's quaternion form.
    transform_t::transform(position, q);

    // Convert the quaternion back to euler.
    orientation = q.toRotationMatrix().eulerAngles(0, 1, 2);
}
void transform_t::transform(Eigen::Vector3d& position, Eigen::Quaterniond& orientation)
{
    // Apply this transform's rotation to the orientation.
    orientation = transform_t::m_rotation * orientation;
    // Normalize the transformed orientation for numerical stability.
    orientation.normalize();

    // Apply this transform's rotation to the position.
    position = transform_t::m_rotation * position;

    // Add this transform's translation to the position.
    position += transform_t::m_translation;
}

// ACCESS
const Eigen::Vector3d& transform_t::translation() const
{
    return transform_t::m_translation;
}
const Eigen::Quaterniond& transform_t::rotation() const
{
    return transform_t::m_rotation;
}

// EXPORT
geometry_msgs_ext::transform transform_t::to_message() const
{
    geometry_msgs_ext::transform message;
    message.translation.x = transform_t::m_translation.x();
    message.translation.y = transform_t::m_translation.y();
    message.translation.z = transform_t::m_translation.z();
    message.rotation.qw = transform_t::m_rotation.w();
    message.rotation.qx = transform_t::m_rotation.x();
    message.rotation.qy = transform_t::m_rotation.y();
    message.rotation.qz = transform_t::m_rotation.z();

    return message;
}