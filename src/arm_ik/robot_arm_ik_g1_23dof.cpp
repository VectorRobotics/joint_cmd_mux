#include "include/robot_arm_ik_g1_23dof.h"
#include "include/casadi_eigen_utils.h"
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <set>

G1_29_ArmIK_NoWrists::G1_29_ArmIK_NoWrists(bool unit_test, 
                                           bool visualization,
                                           const RobotConfig* robot_config)
    : G1_29_ArmIK(unit_test, visualization),
      speed_factor_(0.02) {
    
    std::cout << std::fixed << std::setprecision(5);
    
    // Override URDF paths if robot_config is provided
    if (robot_config != nullptr) {
        urdf_path_ = robot_config->asset_file;
        model_dir_ = robot_config->asset_root;
        
        // Rebuild robot model with new paths
        pinocchio::urdf::buildModel(urdf_path_, robot_model_);
        robot_data_ = pinocchio::Data(robot_model_);
    }
    
    // Add wrist joints to the lock list
    mixed_joints_to_lock_ids_.push_back("left_wrist_pitch_joint");
    mixed_joints_to_lock_ids_.push_back("left_wrist_roll_joint");
    mixed_joints_to_lock_ids_.push_back("left_wrist_yaw_joint");
    mixed_joints_to_lock_ids_.push_back("right_wrist_pitch_joint");
    mixed_joints_to_lock_ids_.push_back("right_wrist_roll_joint");
    mixed_joints_to_lock_ids_.push_back("right_wrist_yaw_joint");
    
    // Build reduced robot with wrist joints locked
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
    
    // Add end-effector frames at elbow joints (without wrists)
    pinocchio::JointIndex left_elbow_id = reduced_model_.getJointId("left_elbow_joint");
    pinocchio::SE3 left_placement(Eigen::Matrix3d::Identity(), 
                                   Eigen::Vector3d(0.35, -0.075, 0));
    reduced_model_.addFrame(pinocchio::Frame("L_ee", left_elbow_id, 
                                             left_placement, pinocchio::OP_FRAME));
    
    pinocchio::JointIndex right_elbow_id = reduced_model_.getJointId("right_elbow_joint");
    pinocchio::SE3 right_placement(Eigen::Matrix3d::Identity(), 
                                    Eigen::Vector3d(0.35, 0.075, 0));
    reduced_model_.addFrame(pinocchio::Frame("R_ee", right_elbow_id, 
                                              right_placement, pinocchio::OP_FRAME));
    
    // Get frame IDs
    L_hand_id_ = reduced_model_.getFrameId("L_ee");
    R_hand_id_ = reduced_model_.getFrameId("R_ee");
    
    // Initialize collision model
    initialize_collision_model();
    
    // Setup optimization with modified cost function (no rotation cost)
    setup_optimization();
    
    // Store dimensions
    nq_ = reduced_model_.nq;
    nv_ = reduced_model_.nv;
    
    // Initialize filter and data
    Eigen::VectorXd weights(4);
    weights << 0.4, 0.3, 0.2, 0.1;
    smooth_filter_ = std::make_unique<WeightedMovingFilter>(weights, nq_);
    init_data_ = Eigen::VectorXd::Zero(nq_);
}

G1_29_ArmIK_NoWrists::~G1_29_ArmIK_NoWrists() {}

void G1_29_ArmIK_NoWrists::initialize_collision_model() {
    // Build geometry model for collision detection
    pinocchio::urdf::buildGeom(reduced_model_, urdf_path_, 
                               pinocchio::COLLISION, geom_model_, model_dir_);
    
    // Create geometry data
    geom_data_ = pinocchio::GeometryData(geom_model_);
    
    // Add all collision pairs
    geom_model_.addAllCollisionPairs();
    
    // Filter out adjacent link collisions
    filter_adjacent_collision_pairs();
    
    std::cout << "num collision pairs - initial: " 
              << geom_model_.collisionPairs.size() << std::endl;
    std::cout << "Number of geometry objects: " 
              << geom_model_.geometryObjects.size() << std::endl;
}

void G1_29_ArmIK_NoWrists::filter_adjacent_collision_pairs() {
    // Get kinematic adjacency from the model
    std::set<std::pair<int, int>> adjacent_pairs;
    for (int i = 1; i < reduced_model_.njoints; ++i) {
        adjacent_pairs.insert({reduced_model_.parents[i], i});
        adjacent_pairs.insert({i, reduced_model_.parents[i]});
    }
    
    // Filter out neighboring links
    std::vector<pinocchio::CollisionPair> filtered_pairs;
    for (const auto& cp : geom_model_.collisionPairs) {
        int link1 = geom_model_.geometryObjects[cp.first].parentJoint;
        int link2 = geom_model_.geometryObjects[cp.second].parentJoint;
        
        if (adjacent_pairs.find({link1, link2}) == adjacent_pairs.end()) {
            filtered_pairs.push_back(cp);
        }
    }
    
    geom_model_.collisionPairs = filtered_pairs;
}

bool G1_29_ArmIK_NoWrists::check_self_collision(const Eigen::VectorXd& q) {
    // Update geometry placements
    pinocchio::updateGeometryPlacements(reduced_model_, reduced_data_, 
                                       geom_model_, geom_data_, q);
    
    // Check all collision pairs
    bool collision_detected = pinocchio::computeCollisions(
        geom_model_, geom_data_, false);
    
    return collision_detected;
}

std::pair<Eigen::VectorXd, Eigen::VectorXd> G1_29_ArmIK_NoWrists::solve_ik(
    const Eigen::Matrix4d& left_wrist,
    const Eigen::Matrix4d& right_wrist,
    const Eigen::VectorXd* current_lr_arm_motor_q,
    const Eigen::VectorXd* current_lr_arm_motor_dq,
    const Eigen::VectorXd* EE_efrc_L,
    const Eigen::VectorXd* EE_efrc_R,
    bool collision_check) {
    
    // Update initial guess from current motor state
    if (current_lr_arm_motor_q != nullptr) {
        // Extract last nq_ elements (arm joints only)
        int offset = current_lr_arm_motor_q->size() - nq_;
        init_data_ = current_lr_arm_motor_q->segment(offset, nq_);
    }
    
    // Set optimization initial guess and parameters
    opti_.set_initial(var_q_, eigen_to_casadi(init_data_));
    opti_.set_value(param_tf_l_, eigen_to_casadi(left_wrist));
    opti_.set_value(param_tf_r_, eigen_to_casadi(right_wrist));
    opti_.set_value(var_q_last_, eigen_to_casadi(init_data_));
    
    try {
        // Solve optimization problem
        casadi::OptiSol sol = opti_.solve();
        
        // Extract solution
        Eigen::VectorXd sol_q = casadi_to_eigen_vector(sol.value(var_q_));
        
        // Apply smoothing filter
        smooth_filter_->add_data(sol_q);
        sol_q = smooth_filter_->filtered_data();
        
        // Check for self-collision
        if (collision_check && check_self_collision(sol_q)) {
            std::cout << "Self-collision detected. Rejecting solution." << std::endl;
            return {init_data_, Eigen::VectorXd::Zero(nv_)};
        }
        
        // Compute velocity
        Eigen::VectorXd v;
        if (current_lr_arm_motor_dq != nullptr) {
            int offset = current_lr_arm_motor_dq->size() - nv_;
            v = current_lr_arm_motor_dq->segment(offset, nv_) * 0.0;
        } else {
            v = (sol_q - init_data_) * 0.0;
        }
        
        init_data_ = sol_q;
        
        // Compute feedforward torques using RNEA
        Eigen::VectorXd sol_tauff = pinocchio::rnea(
            reduced_model_, reduced_data_, sol_q, v,
            Eigen::VectorXd::Zero(nv_)
        );
        
        // Add external forces if provided
        if (EE_efrc_L != nullptr && EE_efrc_R != nullptr) {
            // Compute Jacobians for both end effectors
            pinocchio::Data::Matrix6x J_L(6, nv_);
            pinocchio::Data::Matrix6x J_R(6, nv_);
            
            pinocchio::computeFrameJacobian(reduced_model_, reduced_data_, sol_q, 
                                           L_hand_id_, pinocchio::LOCAL_WORLD_ALIGNED, J_L);
            pinocchio::computeFrameJacobian(reduced_model_, reduced_data_, sol_q, 
                                           R_hand_id_, pinocchio::LOCAL_WORLD_ALIGNED, J_R);
            
            // Compute torques from external forces
            Eigen::VectorXd tau_ext_L = J_L.transpose() * (*EE_efrc_L);
            Eigen::VectorXd tau_ext_R = J_R.transpose() * (*EE_efrc_R);
            
            // Combine external torques (first 4 from left, last 4 from right)
            Eigen::VectorXd tau_ext = Eigen::VectorXd::Zero(nv_);
            if (nv_ >= 8) {
                tau_ext.head(4) = tau_ext_L.head(4);
                tau_ext.tail(4) = tau_ext_R.tail(4);
            }
            
            sol_tauff += tau_ext;
        }
        
        return {sol_q, sol_tauff};
        
    } catch (const std::exception& e) {
        std::cerr << "ERROR in convergence: " << e.what() << std::endl;
        
        // Get debug solution
        Eigen::VectorXd sol_q = casadi_to_eigen_vector(opti_.debug().value(var_q_));
        
        if (sol_q.size() == nq_) {
            smooth_filter_->add_data(sol_q);
            sol_q = smooth_filter_->filtered_data();
        }
        
        Eigen::VectorXd v;
        if (current_lr_arm_motor_dq != nullptr) {
            int offset = current_lr_arm_motor_dq->size() - nv_;
            v = current_lr_arm_motor_dq->segment(offset, nv_) * 0.0;
        } else {
            v = (sol_q - init_data_) * 0.0;
        }
        
        init_data_ = sol_q;
        
        Eigen::VectorXd sol_tauff = pinocchio::rnea(
            reduced_model_, reduced_data_, sol_q, v,
            Eigen::VectorXd::Zero(nv_)
        );
        
        std::cerr << "sol_q: " << sol_q.transpose() << std::endl;

        if (current_lr_arm_motor_q != nullptr) {
            std::cerr << "motorstate: " << current_lr_arm_motor_q->transpose() << std::endl;
        } else {
            std::cerr << "motorstate: "
                      << Eigen::VectorXd::Zero(nq_).transpose() << std::endl;
        }
        std::cerr << "left_pose:\n" << left_wrist << std::endl;
        std::cerr << "right_pose:\n" << right_wrist << std::endl;
        
        // Return current motor state or zeros
        if (current_lr_arm_motor_q != nullptr) {
            int offset = current_lr_arm_motor_q->size() - nq_;
            return {current_lr_arm_motor_q->segment(offset, nq_), 
                    Eigen::VectorXd::Zero(nv_)};
        } else {
            return {Eigen::VectorXd::Zero(nq_), Eigen::VectorXd::Zero(nv_)};
        }
    }
}
