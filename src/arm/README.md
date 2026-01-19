# How to use

To use IK, do the following in the package
```cpp
#include "arm/include/robot_arm_ik_g1_23dof.h"


const RobotConfig* robot_config = new RobotConfig("","");
G1_29_ArmIK_NoWrists arm_ik(robot_config=robot_config);

Eigen::Quaterniond quat_identity(1, 0, 0, 0);
Eigen::Matrix4d L_tf_target = create_se3(quat_identity, Eigen::Vector3d(0.25, 0.25, 0.1));
Eigen::Matrix4d R_tf_target = create_se3(quat_identity, Eigen::Vector3d(0.25, -0.25, 0.1));
    
// Solving IK
auto [sol_q, sol_tauff] = arm_ik.solve_ik(L_tf_target, R_tf_target);
```


## Some helper functions
```cpp
// Helper function to create SE3 transformation matrix
Eigen::Matrix4d create_se3(const Eigen::Quaterniond& q, const Eigen::Vector3d& t) {
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    transform.block<3, 1>(0, 3) = t;
    return transform;
}

// Helper function to create SE3 from quaternion components and translation
Eigen::Matrix4d create_se3(double qw, double qx, double qy, double qz, 
                           double tx, double ty, double tz) {
    Eigen::Quaterniond q(qw, qx, qy, qz);
    Eigen::Vector3d t(tx, ty, tz);
    return create_se3(q, t);
}


```