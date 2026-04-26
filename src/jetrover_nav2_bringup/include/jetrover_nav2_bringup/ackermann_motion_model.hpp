#ifndef NAV2_AMCL__MOTION_MODEL__ACKERMANN_MOTION_MODEL_HPP_
#define NAV2_AMCL__MOTION_MODEL__ACKERMANN_MOTION_MODEL_HPP_

#include "nav2_amcl/motion_model/motion_model.hpp"

namespace jetrover_nav2_bringup {

class AckermannMotionModel : public nav2_amcl::MotionModel {
public:
    void initialize(double alpha1, double alpha2, double alpha3, double alpha4, double alpha5) override;
    void odometryUpdate(pf_t* pf, const pf_vector_t& pose, const pf_vector_t& delta) override;

private:
    double alpha1_{0.2}; // 旋转运动对旋转估计的噪声系数
    double alpha2_{0.2}; // 平移运动对旋转估计的噪声系数
    double alpha3_{0.2}; // 平移运动对平移估计的噪声系数
    double alpha4_{0.2}; // 旋转运动对平移估计的噪声系数
    double alpha5_{0.2}; // 阿克曼模型转向角相关的噪声系数
};
} // namespace jetrover_nav2_bringup
#endif // NAV2_AMCL__MOTION_MODEL__ACKERMANN_MOTION_MODEL_HPP_
