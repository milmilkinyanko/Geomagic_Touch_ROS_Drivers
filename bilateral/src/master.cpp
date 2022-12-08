#include <array>
#include <boost/shared_ptr.hpp>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "omni_msgs/OmniFeedback.h"
#include "omni_msgs/OmniState.h"

#include "bilateral/bilateral.hpp"

template <typename T>
static bool waitForMyMsg(const std::string& topic_name, ros::NodeHandle& nh)
{
    boost::shared_ptr<const T> ptr = ros::topic::waitForMessage<T>(topic_name, nh, ros::Duration(10));
    if (ptr == nullptr) {
        return false;
    } else {
        return true;
    }
}

void BilateralController::forceControl()
{
    omni_msgs::OmniFeedback force_msg;
    // phantomの場合、forceをかける方向はencの向きと逆 -> joint_gainはマイナス
    // A0Bにおいては各軸について符号あわせる
    // slaveをmasterにあわせる -> ref: slave, th: master
    std::array<double, 3> tauref
        = -this->positionIIRController() - this->forceIIRController();
    // std::cout << "x: " << tau_est.at(0) << std::endl;
    // std::cout << "y: " << tau_est.at(1) << std::endl;
    // std::cout << "z: " << tau_est.at(2) << std::endl;
    force_msg.force.x = tauref.at(0);
    force_msg.force.y = tauref.at(1);
    force_msg.force.z = tauref.at(2);
    // force_msg.force.y = 0.0;
    // force_msg.force.z = 0.0;
    force_msg.position.x = 0.0;
    force_msg.position.y = 0.0;
    force_msg.position.z = 0.0;
    m_pub.publish(force_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bilateral_master");
    ros::NodeHandle nh;

    // slaveのomni_stateが立ち上がるまで待つ
    if (!waitForMyMsg<omni_msgs::BilateralData>("/phantom_slave/phantom/data", nh)) {
        ROS_ERROR("DID NOT RECEIVE SLAVE TOPIC");
        return EXIT_FAILURE;
    }
    // masterのomni_stateが立ち上がるまで待つ

    if (!waitForMyMsg<omni_msgs::BilateralData>("/phantom_master/phantom/data", nh)) {
        ROS_ERROR("DID NOT RECEIVE MASTER TOPIC");
        return EXIT_FAILURE;
    }

    ROS_INFO("Start bilateral master node ...");
    BilateralController bilateral_controller(BilateralController::MS::Master);
    ros::spin();
    ROS_INFO("End bilateral master node ...");
    return EXIT_SUCCESS;
}
