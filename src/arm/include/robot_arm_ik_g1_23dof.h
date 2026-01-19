#ifndef ROBOT_ARM_IK_G1_23DOF_H
#define ROBOT_ARM_IK_G1_23DOF_H

#include "robot_arm_ik.h"
#include <pinocchio/algorithm/collision.hpp>
#include <pinocchio/parsers/urdf.hpp>

/**
 * @brief G1_29_ArmIK_NoWrists - Extended IK solver without wrist joints
 * 
 * This class extends G1_29_ArmIK by:
 * - Locking wrist joints
 * - Adding self-collision detection
 * - Supporting external forces on end effectors
 */
class G1_29_ArmIK_NoWrists : public G1_29_ArmIK {
public:
    /**
     * @brief Constructor
     * @param unit_test Enable unit test mode
     * @param visualization Enable visualization
     * @param robot_config Robot configuration (asset file and root directory)
     */
    G1_29_ArmIK_NoWrists(bool unit_test = false, 
                         bool visualization = false,
                         const RobotConfig* robot_config = nullptr);
    
    ~G1_29_ArmIK_NoWrists();

    /**
     * @brief Solve inverse kinematics with collision checking and external forces
     * @param left_wrist Target pose for left wrist
     * @param right_wrist Target pose for right wrist
     * @param current_lr_arm_motor_q Current joint positions
     * @param current_lr_arm_motor_dq Current joint velocities
     * @param EE_efrc_L External force on left end effector (6D wrench)
     * @param EE_efrc_R External force on right end effector (6D wrench)
     * @param collision_check Enable self-collision checking
     * @return Pair of (joint_positions, joint_torques)
     */
    std::pair<Eigen::VectorXd, Eigen::VectorXd> solve_ik(
        const Eigen::Matrix4d& left_wrist,
        const Eigen::Matrix4d& right_wrist,
        const Eigen::VectorXd* current_lr_arm_motor_q = nullptr,
        const Eigen::VectorXd* current_lr_arm_motor_dq = nullptr,
        const Eigen::VectorXd* EE_efrc_L = nullptr,
        const Eigen::VectorXd* EE_efrc_R = nullptr,
        bool collision_check = true
    );

    /**
     * @brief Check for self-collision at given configuration
     * @param q Joint configuration to check
     * @return true if collision detected, false otherwise
     */
    bool check_self_collision(const Eigen::VectorXd& q);

private:
    void initialize_collision_model();
    void filter_adjacent_collision_pairs();
    
    // Interpolation attributes (not fully implemented in this version)
    Eigen::Matrix4d current_L_tf_;
    Eigen::Matrix4d current_R_tf_;
    Eigen::Quaterniond current_L_orientation_;
    Eigen::Quaterniond current_R_orientation_;
    double speed_factor_;

    // Geometry model for collision detection
    pinocchio::GeometryModel geom_model_;
    pinocchio::GeometryData geom_data_;
    
    int nq_;
    int nv_;
};

#endif // ROBOT_ARM_IK_G1_23DOF_H
