#ifndef ROBOT_ARM_IK_H
#define ROBOT_ARM_IK_H

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/spatial/se3.hpp>

#include <casadi/casadi.hpp>

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <memory>
#include <fstream>
#include <iostream>
#include <iomanip>

#include "weighted_moving_filter.h"

/**
 * @brief Base configuration structure for robot models
 */
struct RobotConfig {
    std::string asset_file;
    std::string asset_root;
};

/**
 * @brief G1_29_ArmIK - Inverse kinematics solver for G1 robot with 29 DOF
 */
class G1_29_ArmIK {
public:
    G1_29_ArmIK(
        bool unit_test = false, 
        bool visualization = false
    );
    virtual ~G1_29_ArmIK();

    /**
     * @brief Solve inverse kinematics for both arms
     * @param left_wrist Target pose for left wrist (4x4 homogeneous transformation)
     * @param right_wrist Target pose for right wrist (4x4 homogeneous transformation)
     * @param current_lr_arm_motor_q Current joint positions (optional)
     * @param current_lr_arm_motor_dq Current joint velocities (optional)
     * @return Pair of (joint_positions, joint_torques)
     */
    virtual std::pair<Eigen::VectorXd, Eigen::VectorXd> solve_ik(
        const Eigen::Matrix4d& left_wrist,
        const Eigen::Matrix4d& right_wrist,
        const Eigen::VectorXd* current_lr_arm_motor_q = nullptr,
        const Eigen::VectorXd* current_lr_arm_motor_dq = nullptr
    );

    /**
     * @brief Scale arm poses based on arm length ratio
     * @param human_left_pose Human left arm pose
     * @param human_right_pose Human right arm pose
     * @param human_arm_length Human arm length (default: 0.60m)
     * @param robot_arm_length Robot arm length (default: 0.75m)
     * @return Pair of scaled (left_pose, right_pose)
     */
    std::pair<Eigen::Matrix4d, Eigen::Matrix4d> scale_arms(
        const Eigen::Matrix4d& human_left_pose,
        const Eigen::Matrix4d& human_right_pose,
        double human_arm_length = 0.60,
        double robot_arm_length = 0.75
    );

protected:
    void setup_optimization();
    void initialize_joints_to_lock();
    void add_end_effector_frames();

    bool unit_test_;
    bool visualization_;
    std::string urdf_path_;
    std::string model_dir_;

    pinocchio::Model robot_model_;
    pinocchio::Data robot_data_;
    pinocchio::Model reduced_model_;
    pinocchio::Data reduced_data_;

    casadi::Opti opti_;
    casadi::MX var_q_;
    casadi::MX var_q_last_;
    casadi::MX param_tf_l_;
    casadi::MX param_tf_r_;

    casadi::Function translational_error_;
    casadi::Function rotational_error_;

    pinocchio::FrameIndex L_hand_id_;
    pinocchio::FrameIndex R_hand_id_;

    Eigen::VectorXd init_data_;
    std::unique_ptr<WeightedMovingFilter> smooth_filter_;

    std::vector<std::string> mixed_joints_to_lock_ids_;
};
#endif // ROBOT_ARM_IK_H