#include "robot_arm_ik.h"
#include <iostream>
#include <stdexcept>
#include <cmath>

// ============================================================================
// G1_29_ArmIK Implementation
// ============================================================================

G1_29_ArmIK::G1_29_ArmIK(bool unit_test, bool visualization)
    : unit_test_(unit_test), 
      visualization_(visualization), 
      cache_path_("g1_29_model_cache.pkl") {
    
    std::cout << std::fixed << std::setprecision(5);
    
    urdf_path_ = "../assets/g1/g1_body29_hand14.urdf";
    
    // Initialize joints to lock
    initialize_joints_to_lock();
    
    std::cout << "[G1_29_ArmIK] >>> Loading URDF (slow)..." << std::endl;
    
    // Build robot model from URDF
    pinocchio::urdf::buildModel(urdf_path_, robot_model_);
    robot_data_ = pinocchio::Data(robot_model_);
    
    // Build reduced robot by locking specified joints
    std::vector<pinocchio::JointIndex> joints_to_lock;
    for (const auto& joint_name : mixed_joints_to_lock_ids_) {
        if (robot_model_.existJointName(joint_name)) {
            joints_to_lock.push_back(robot_model_.getJointId(joint_name));
        }
    }
    
    Eigen::VectorXd reference_config = Eigen::VectorXd::Zero(robot_model_.nq);
    pinocchio::buildReducedModel(robot_model_, joints_to_lock, 
                                    reference_config, reduced_model_);
    reduced_data_ = pinocchio::Data(reduced_model_);
    
    // Add end-effector frames
    add_end_effector_frames();
    
    // Setup CasADi optimization
    setup_optimization();
    
    // Initialize filter and data
    Eigen::VectorXd weights(4);
    weights << 0.4, 0.3, 0.2, 0.1;
    smooth_filter_ = std::make_unique<WeightedMovingFilter>(weights, 14);
    init_data_ = Eigen::VectorXd::Zero(reduced_model_.nq);
}

G1_29_ArmIK::~G1_29_ArmIK() {}

void G1_29_ArmIK::initialize_joints_to_lock() {
    mixed_joints_to_lock_ids_ = {
        "left_hip_pitch_joint",
        "left_hip_roll_joint", 
        "left_hip_yaw_joint",
        "left_knee_joint", 
        "left_ankle_pitch_joint", 
        "left_ankle_roll_joint",
        "right_hip_pitch_joint", 
        "right_hip_roll_joint", 
        "right_hip_yaw_joint",
        "right_knee_joint", 
        "right_ankle_pitch_joint", 
        "right_ankle_roll_joint",
        "waist_yaw_joint", 
        "waist_roll_joint", 
        "waist_pitch_joint",

        "left_hand_thumb_0_joint", 
        "left_hand_thumb_1_joint", 
        "left_hand_thumb_2_joint",
        "left_hand_middle_0_joint", 
        "left_hand_middle_1_joint",
        "left_hand_index_0_joint", 
        "left_hand_index_1_joint",
        "right_hand_thumb_0_joint", 
        "right_hand_thumb_1_joint", 
        "right_hand_thumb_2_joint",
        "right_hand_index_0_joint", 
        "right_hand_index_1_joint",
        "right_hand_middle_0_joint", 
        "right_hand_middle_1_joint"
    };
}

void G1_29_ArmIK::add_end_effector_frames() {
    // Add left end-effector frame
    pinocchio::JointIndex left_wrist_id = reduced_model_.getJointId("left_wrist_yaw_joint");
    pinocchio::SE3 left_placement(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.05, 0, 0));
    reduced_model_.addFrame(pinocchio::Frame("L_ee", left_wrist_id, left_placement, 
                                            pinocchio::OP_FRAME));
    
    // Add right end-effector frame
    pinocchio::JointIndex right_wrist_id = reduced_model_.getJointId("right_wrist_yaw_joint");
    pinocchio::SE3 right_placement(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.05, 0, 0));
    reduced_model_.addFrame(pinocchio::Frame("R_ee", right_wrist_id, right_placement, 
                                             pinocchio::OP_FRAME));

    // Get frame IDs for end effectors
    L_hand_id_ = reduced_model_.getFrameId("L_ee");
    R_hand_id_ = reduced_model_.getFrameId("R_ee");
}

void G1_29_ArmIK::setup_optimization() {
    // Create optimization variables and parameters
    var_q_ = opti_.variable(reduced_model_.nq, 1);
    var_q_last_ = opti_.parameter(reduced_model_.nq, 1);
    param_tf_l_ = opti_.parameter(4, 4);
    param_tf_r_ = opti_.parameter(4, 4);
    
    // Note: Full CasADi-Pinocchio integration requires more complex setup
    // This includes creating symbolic functions for forward kinematics
    // and defining error functions. This is simplified here.
    
    // Set joint limits as constraints
    casadi::DM lower_limits = casadi::DM(reduced_model_.lowerPositionLimit);
    casadi::DM upper_limits = casadi::DM(reduced_model_.upperPositionLimit);
    opti_.subject_to(opti_.bounded(lower_limits, var_q_, upper_limits));
    
    // Set optimization options
    casadi::Dict opts;
    opts["expand"] = true;
    opts["detect_simple_bounds"] = true;
    opts["calc_lam_p"] = false;
    opts["print_time"] = false;
    opts["ipopt.sb"] = "yes";
    opts["ipopt.print_level"] = 0;
    opts["ipopt.max_iter"] = 30;
    opts["ipopt.tol"] = 1e-4;
    opts["ipopt.acceptable_tol"] = 5e-4;
    opts["ipopt.acceptable_iter"] = 5;
    opts["ipopt.warm_start_init_point"] = "yes";
    opts["ipopt.derivative_test"] = "none";
    opts["ipopt.jacobian_approximation"] = "exact";
    
    opti_.solver("ipopt", opts);
}

std::pair<Eigen::Matrix4d, Eigen::Matrix4d> G1_29_ArmIK::scale_arms(
    const Eigen::Matrix4d& human_left_pose,
    const Eigen::Matrix4d& human_right_pose,
    double human_arm_length,
    double robot_arm_length) {
    
    double scale_factor = robot_arm_length / human_arm_length;
    Eigen::Matrix4d robot_left_pose = human_left_pose;
    Eigen::Matrix4d robot_right_pose = human_right_pose;
    
    robot_left_pose.block<3, 1>(0, 3) *= scale_factor;
    robot_right_pose.block<3, 1>(0, 3) *= scale_factor;
    
    return {robot_left_pose, robot_right_pose};
}

std::pair<Eigen::VectorXd, Eigen::VectorXd> G1_29_ArmIK::solve_ik(
    const Eigen::Matrix4d& left_wrist,
    const Eigen::Matrix4d& right_wrist,
    const Eigen::VectorXd* current_lr_arm_motor_q,
    const Eigen::VectorXd* current_lr_arm_motor_dq) {
    
    // Update initial guess
    if (current_lr_arm_motor_q != nullptr) {
        init_data_ = *current_lr_arm_motor_q;
    }
    
    // Set optimization initial guess and parameters
    opti_.set_initial(var_q_, casadi::DM(init_data_));
    opti_.set_value(param_tf_l_, casadi::DM(left_wrist));
    opti_.set_value(param_tf_r_, casadi::DM(right_wrist));
    opti_.set_value(var_q_last_, casadi::DM(init_data_));
    
    try {
        // Solve optimization problem
        casadi::OptiSol sol = opti_.solve();
        
        // Extract solution
        std::vector<double> sol_q_vec = static_cast<std::vector<double>>(sol.value(var_q_));
        Eigen::VectorXd sol_q = Eigen::Map<Eigen::VectorXd>(sol_q_vec.data(), reduced_model_.nq);
        
        // Apply smoothing filter
        smooth_filter_->add_data(sol_q);
        sol_q = smooth_filter_->filtered_data();
        
        // Compute velocity
        Eigen::VectorXd v;
        if (current_lr_arm_motor_dq != nullptr) {
            v = (*current_lr_arm_motor_dq) * 0.0;
        } else {
            v = (sol_q - init_data_) * 0.0;
        }
        
        init_data_ = sol_q;
        
        // Compute feedforward torques using RNEA
        Eigen::VectorXd sol_tauff = pinocchio::rnea(
            reduced_model_, reduced_data_, sol_q, v,
            Eigen::VectorXd::Zero(reduced_model_.nv)
        );
        
        return {sol_q, sol_tauff};
        
    } catch (const std::exception& e) {
        std::cerr << "ERROR in convergence: " << e.what() << std::endl;
        
        // Get debug solution
        std::vector<double> sol_q_vec = static_cast<std::vector<double>>(
            opti_.debug().value(var_q_));
        Eigen::VectorXd sol_q = Eigen::Map<Eigen::VectorXd>(
            sol_q_vec.data(), reduced_model_.nq);
        
        smooth_filter_->add_data(sol_q);
        sol_q = smooth_filter_->filtered_data();
        
        Eigen::VectorXd v;
        if (current_lr_arm_motor_dq != nullptr) {
            v = (*current_lr_arm_motor_dq) * 0.0;
        } else {
            v = (sol_q - init_data_) * 0.0;
        }
        
        init_data_ = sol_q;
        
        Eigen::VectorXd sol_tauff = pinocchio::rnea(
            reduced_model_, reduced_data_, sol_q, v,
            Eigen::VectorXd::Zero(reduced_model_.nv)
        );
        
        std::cerr << "sol_q: " << sol_q.transpose() << std::endl;
        std::cerr << "left_pose:\n" << left_wrist << std::endl;
        std::cerr << "right_pose:\n" << right_wrist << std::endl;
        
        // Return current motor state or zeros
        if (current_lr_arm_motor_q != nullptr) {
            return {*current_lr_arm_motor_q, Eigen::VectorXd::Zero(reduced_model_.nv)};
        } else {
            return {sol_q, sol_tauff};
        }
    }
}
