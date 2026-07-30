#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

#include <Eigen/Core>
#include <iostream>

int main()
{
    pinocchio::Model model;
    pinocchio::urdf::buildModel(
        "/home/user/ros2_ws/src/truck_description/urdf/truck.urdf",
        model);

    pinocchio::Data data(model);

    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);

    pinocchio::forwardKinematics(model, data, q);
    pinocchio::computeJointJacobians(model, data);
    pinocchio::updateFramePlacements(model, data);

    Eigen::MatrixXd J(6, model.nv);

    pinocchio::computeFrameJacobian(
        model,
        data,
        q,
        model.getFrameId("ee_link"),
        pinocchio::LOCAL_WORLD_ALIGNED,
        J);

    std::cout << J << std::endl;

    return 0;
}