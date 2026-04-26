#include "jetrover_nav2_bringup/ackermann_motion_model.hpp"

#include <cmath>

#include "nav2_amcl/pf/pf_pdf.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace jetrover_nav2_bringup {

namespace {

double normalizeAngle(double angle)
{
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }

    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }

    return angle;
}

double clampVariance(double variance)
{
    if (variance < 0.0) {
        return 0.0;
    }

    return variance;
}

} // namespace

void AckermannMotionModel::initialize(double alpha1, double alpha2, double alpha3, double alpha4, double alpha5)
{
    alpha1_ = alpha1;
    alpha2_ = alpha2;
    alpha3_ = alpha3;
    alpha4_ = alpha4;
    alpha5_ = alpha5;
}

void AckermannMotionModel::odometryUpdate(pf_t* pf, const pf_vector_t& pose, const pf_vector_t& delta)
{
    /*
     * Ackermann AMCL motion model.
     *
     * This model assumes the robot follows a bicycle-model arc.
     *
     * AMCL only provides odometry delta:
     *   delta.v[0] = odom dx
     *   delta.v[1] = odom dy
     *   delta.v[2] = odom dyaw
     *
     * It does not provide steering angle directly.
     * Therefore, curvature is inferred from odometry:
     *
     *   delta_s     = signed longitudinal distance
     *   delta_theta = yaw change
     *
     * Parameters:
     *   alpha1: yaw noise from yaw motion
     *   alpha2: yaw noise from translation
     *   alpha3: translation noise from translation
     *   alpha4: translation noise from yaw motion
     *   alpha5: lateral slip / lateral odom error
     */

    const double dx_odom = delta.v[0];
    const double dy_odom = delta.v[1];
    const double delta_theta = normalizeAngle(delta.v[2]);

    const double cos_odom_yaw = std::cos(pose.v[2]);
    const double sin_odom_yaw = std::sin(pose.v[2]);

    // Convert odom-frame delta into previous robot-base frame.
    const double dx_base = cos_odom_yaw * dx_odom + sin_odom_yaw * dy_odom;

    const double dy_base = -sin_odom_yaw * dx_odom + cos_odom_yaw * dy_odom;

    // Ackermann robot cannot rotate in place.
    // Longitudinal motion dominates.
    const double delta_s = dx_base;

    // Lateral component is treated as odometry slip/error, not commanded motion.
    const double delta_lateral = dy_base;

    const double trans_var = clampVariance(alpha3_ * delta_s * delta_s + alpha4_ * delta_theta * delta_theta);

    const double yaw_var = clampVariance(alpha1_ * delta_theta * delta_theta + alpha2_ * delta_s * delta_s);

    const double lateral_var = clampVariance(alpha5_ * delta_s * delta_s + alpha5_ * delta_theta * delta_theta);

    pf_sample_set_t* set = pf->sets + pf->current_set;

    for (int i = 0; i < set->sample_count; ++i) {
        pf_sample_t* sample = set->samples + i;

        const double delta_s_hat = delta_s - pf_ran_gaussian(trans_var);

        const double delta_theta_hat = delta_theta - pf_ran_gaussian(yaw_var);

        const double delta_lateral_hat = delta_lateral - pf_ran_gaussian(lateral_var);

        const double theta = sample->pose.v[2];

        if (std::abs(delta_theta_hat) < 1e-6) {
            // Straight Ackermann motion.
            sample->pose.v[0] += delta_s_hat * std::cos(theta);
            sample->pose.v[1] += delta_s_hat * std::sin(theta);
        } else {
            // Circular arc Ackermann integration.
            const double radius = delta_s_hat / delta_theta_hat;

            sample->pose.v[0] += radius * (std::sin(theta + delta_theta_hat) - std::sin(theta));

            sample->pose.v[1] += -radius * (std::cos(theta + delta_theta_hat) - std::cos(theta));
        }

        // Small lateral slip / odom lateral residual.
        sample->pose.v[0] += delta_lateral_hat * std::cos(theta + M_PI_2);

        sample->pose.v[1] += delta_lateral_hat * std::sin(theta + M_PI_2);

        sample->pose.v[2] = normalizeAngle(theta + delta_theta_hat);
    }
}

} // namespace jetrover_nav2_bringup

PLUGINLIB_EXPORT_CLASS(jetrover_nav2_bringup::AckermannMotionModel, nav2_amcl::MotionModel)