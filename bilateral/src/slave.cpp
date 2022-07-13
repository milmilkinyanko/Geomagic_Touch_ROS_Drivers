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
    boost::shared_ptr<const T> ptr = ros::topic::waitForMessage<T>(topic_name, nh, ros::Duration(1.0));
    if (ptr == nullptr) {
        return false;
    } else {
        return true;
    }
}

void BilateralController::forceControl()
{
    omni_msgs::OmniFeedback force_msg;
    // A0Bにおいては各軸について符号あわせる
    std::array<double, 3> tauref
        = this->positionIIRController() - this->forceIIRController();
    force_msg.force.x = tauref.at(0);
    // force_msg.force.y = tauref.at(1);
    // force_msg.force.z = tauref.at(2);
    force_msg.force.y = 0.0;
    force_msg.force.z = 0.0;
    // for (int i = 0; i < 3; i++) {
    //     this->m_tauref.at(i) = tauref.at(i);
    //     ROS_INFO("%i: %lf", i, tauref.at(i));
    // }
    force_msg.position.x = 0.0;
    force_msg.position.y = 0.0;
    force_msg.position.z = 0.0;
    m_pub.publish(force_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bilateral_slave");
    ros::NodeHandle nh;

    // slaveのomni_stateが立ち上がるまで待つ
    if (!waitForMyMsg<omni_msgs::OmniState>("/phantom_slave/phantom/state", nh)) {
        ROS_ERROR("DID NOT RECEIVE SLAVE TOPIC");
        return EXIT_FAILURE;
    }
    // masterのomni_stateが立ち上がるまで待つ
    if (!waitForMyMsg<omni_msgs::OmniState>("/phantom_master/phantom/state", nh)) {
        ROS_ERROR("DID NOT RECEIVE MASTER TOPIC");
        return EXIT_FAILURE;
    }

    ROS_INFO("Start bilateral slave node ...");
    BilateralController bilateral_controller(BilateralController::MS::Slave);
    ros::spin();
    ROS_INFO("End bilateral slave node ...");
    return EXIT_SUCCESS;
}
