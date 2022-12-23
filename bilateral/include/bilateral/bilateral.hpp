#pragma once

#include <array>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include "omni_msgs/OmniFeedback.h"
#include "omni_msgs/BilateralData.h"
#include "IIR.hpp"

namespace PIDParams
{
static constexpr double a0 = 84.44;
static constexpr double a1 = -84.29;
static constexpr double b1 = 0.9851;
}  // namespace PIDParams
namespace Slave
{
namespace DOBParams
{
namespace LPF
{
static constexpr double a0 = 0.0001533;
static constexpr double a1 = 0.0001533;
static constexpr double b1 = 0.9851;
}  // namespace LPF
namespace MotorInv
{
static constexpr std::array<double, 3> a0 = {0.9679, 0, 0};
static constexpr std::array<double, 3> a1 = {-1.936, 0, 0};
static constexpr std::array<double, 3> a2 = {0.9679, 0, 0};
static constexpr double b1 = 1.979;
static constexpr double b2 = -0.979;
}  // namespace MotorInv
}  // namespace DOBParams
namespace RFOBParams
{
static constexpr double a0 = 14.89;
static constexpr double a1 = -14.89;
static constexpr double b1 = 0.9851;
}  // namespace RFOBParams
}  // namespace Slave

template <typename T, std::size_t N>
std::array<T, N> operator+(const std::array<T, N>& a, const std::array<T, N>& b) noexcept
{
    std::array<T, N> ret;
    for (std::size_t i = 0; i < N; i++) {
        ret.at(i) = a.at(i) + b.at(i);
    }
    return ret;
}
template <typename T, std::size_t N>
std::array<T, N> operator-(const std::array<T, N>& a, const std::array<T, N>& b) noexcept
{
    std::array<T, N> ret;
    for (std::size_t i = 0; i < N; i++) {
        ret.at(i) = a.at(i) - b.at(i);
    }
    return ret;
}
template <typename T, std::size_t N>
std::array<T, N> operator*(const double a, const std::array<T, N>& b) noexcept
{
    std::array<T, N> ret;
    for (std::size_t i = 0; i < N; i++) {
        ret.at(i) = a * b.at(i);
    }
    return ret;
}
template <typename T>
std::vector<T> operator*(const double a, const std::vector<T>& b) noexcept
{
    std::vector<T> ret;
    int N = b.size();
    ret.resize(N);
    for (int i = 0; i < N; i++) {
        ret.at(i) = a * b.at(i);
    }
    return ret;
}
template <typename T, std::size_t N>
std::array<T, N> operator-(const std::array<T, N>& a) noexcept
{
    std::array<T, N> ret;
    for (std::size_t i = 0; i < N; i++) {
        ret.at(i) = -a.at(i);
    }
    return ret;
}

class BilateralController
{
public:
    enum class MS
    {
        Master,
        Slave
    };

private:
    std::string m_topic_name_master;
    std::string m_topic_name_slave;
    std::vector<double> m_joint_gain_list;
    std::vector<double> m_force_gain_list;
    std::vector<double> m_position_scale_gain;
    std::vector<double> m_force_scale_gain;
    std::vector<double> m_rfob_c;
    std::vector<double> m_rfob_fc;
    BilateralController::MS m_master_or_slave;

    ros::NodeHandle m_nh;
    ros::NodeHandle m_pnh;
    ros::Publisher m_pub;
    ros::Subscriber m_sub_master;
    ros::Subscriber m_sub_slave;

    geometry_msgs::Pose m_master_pose;
    geometry_msgs::Pose m_slave_pose;

    std::array<double, 3> m_tauref_slave;
    std::array<double, 3> m_tauref_master;
    std::array<double, 3> m_joint_theta_slave;
    std::array<double, 3> m_joint_theta_master;

    std::vector<IIRFilter> m_position_iir_controller;
    std::vector<IIRFilter> m_force_dob_lpf_master;
    std::vector<IIRFilter> m_force_dob_motor_inv_master;
    std::vector<IIRFilter> m_force_rfob_master;
    std::vector<IIRFilter> m_force_dob_lpf_slave;
    std::vector<IIRFilter> m_force_dob_motor_inv_slave;
    std::vector<IIRFilter> m_force_rfob_slave;

    // 位置にもとづくディジタル制御器
    // tustin変換 (双一次z変換) によりIIR型フィルタとして構成している
    std::array<double, 3> positionIIRController()
    {
        std::array<double, 3> pos_feedback_diff;  // theta_input_diff
        // x, y, zでしかaccessできないので仕方なく...
        for (int i = 0; i < 3; i++) {
            pos_feedback_diff.at(i) = this->m_joint_theta_master.at(i) - this->m_position_scale_gain.at(i) * this->m_joint_theta_slave.at(i);
        }
        std::array<double, 3> ret;
        for (int i = 0; i < 3; i++) {
            ret.at(i) = this->m_position_iir_controller.at(i).control(pos_feedback_diff.at(i));
        }
        return ret;
    }

    std::array<double, 3> forceDOB(std::array<double, 3>& tauref, std::array<double, 3>& theta, BilateralController::MS master_or_slave)
    {
        auto& dob_lpf = (master_or_slave == BilateralController::MS::Master) ? m_force_dob_lpf_master : m_force_dob_lpf_slave;
        auto& dob_motor_inv = (master_or_slave == BilateralController::MS::Master) ? m_force_dob_motor_inv_master : m_force_dob_motor_inv_slave;
        auto& rfob = (master_or_slave == BilateralController::MS::Master) ? m_force_rfob_master : m_force_rfob_slave;

        std::array<double, 3> ret;
        double eps = 0.01;

        for (int i = 0; i < 3; i++) {
            double tmp_tau = dob_lpf.at(i).control(tauref.at(i));
            double tmp_inv = dob_motor_inv.at(i).control(theta.at(i));
            double tmp_omega = rfob.at(i).control(theta.at(i));
            double tmp_rfob;
            if (abs(tmp_omega) < eps) {
                tmp_rfob = m_rfob_c.at(i) * tmp_omega;
            } else if (tmp_omega > 0) {
                tmp_rfob = m_rfob_c.at(i) * tmp_omega + m_rfob_fc.at(i);
            } else {
                tmp_rfob = m_rfob_c.at(i) * tmp_omega - m_rfob_fc.at(i);
            }
            ret.at(i) = tmp_tau - tmp_inv - tmp_rfob;
            if (i == 0) {
                ROS_INFO("tau: %lf, inv: %lf, omega: %lf, rfob: %lf, sum: %lf", tmp_tau, tmp_inv, tmp_omega, tmp_rfob, tmp_tau - tmp_inv - tmp_rfob);
            }
        }
        return ret;
    }

    // TODO: 今はとりあえず定数だが、モータパラメータを使ってDOB、RFOBを構成する
    std::array<double, 3> forceIIRController()
    {
        // std::array<double, 3> master_reaction_force = forceDOB(this->m_tauref_master, this->m_joint_theta_master, BilateralController::MS::Master);
        std::array<double, 3> slave_reaction_force = forceDOB(this->m_tauref_slave, this->m_joint_theta_slave, BilateralController::MS::Slave);
        std::array<double, 3> ret = 0.9 * (slave_reaction_force);
        return ret;
    }

public:
    BilateralController(BilateralController::MS ms = BilateralController::MS::Slave) : m_master_or_slave(ms), m_pnh("~")
    {
        // get parameters
        if (!m_pnh.getParam("/topic_master", m_topic_name_master)) {
            ROS_FATAL("'topic_master' is not set");
        }
        ROS_INFO("topic_master: %s", m_topic_name_master.c_str());
        if (!m_pnh.getParam("/topic_slave", m_topic_name_slave)) {
            ROS_FATAL("'topic_slave' is not set");
        }
        ROS_INFO("topic_slave: %s", m_topic_name_slave.c_str());
        if (!m_pnh.getParam("joint_gain_list", m_joint_gain_list)) {
            ROS_FATAL("'joint_gain_list' is not set");
        }
        ROS_INFO("joint_gain_list: [%lf, %lf, %lf]", m_joint_gain_list.at(0), m_joint_gain_list.at(1), m_joint_gain_list.at(2));
        if (!m_pnh.getParam("force_gain_list", m_force_gain_list)) {
            ROS_FATAL("'force_gain_list' is not set");
        }
        ROS_INFO("force_gain_list: [%lf, %lf, %lf]", m_force_gain_list.at(0), m_force_gain_list.at(1), m_force_gain_list.at(2));
        if (!m_pnh.getParam("/position_scale_gain", m_position_scale_gain)) {
            ROS_FATAL("'position_scale_gain' is not set");
        }
        ROS_INFO("position_scale_gain: [%lf, %lf, %lf]", m_position_scale_gain.at(0), m_position_scale_gain.at(1), m_position_scale_gain.at(2));
        if (!m_pnh.getParam("/force_scale_gain", m_force_scale_gain)) {
            ROS_FATAL("'force_scale_gain' is not set");
        }
        ROS_INFO("force_scale_gain: [%lf, %lf, %lf]", m_force_scale_gain.at(0), m_force_scale_gain.at(1), m_force_scale_gain.at(2));
        if (!m_pnh.getParam("rfob_c", m_rfob_c)) {
            ROS_FATAL("'rfob_c' is not set");
        }
        ROS_INFO("rfob_c: [%lf, %lf, %lf]", m_rfob_c.at(0), m_rfob_c.at(1), m_rfob_c.at(2));
        if (!m_pnh.getParam("rfob_fc", m_rfob_fc)) {
            ROS_FATAL("'rfob_fc' is not set");
        }
        ROS_INFO("rfob_fc: [%lf, %lf, %lf]", m_rfob_fc.at(0), m_rfob_fc.at(1), m_rfob_fc.at(2));

        // set publisher/subscriber
        if (m_master_or_slave == BilateralController::MS::Master) {
            m_pub = m_nh.advertise<omni_msgs::OmniFeedback>(m_topic_name_master + "/force_feedback", 1);
        } else {
            m_pub = m_nh.advertise<omni_msgs::OmniFeedback>(m_topic_name_slave + "/force_feedback", 1);
        }
        m_sub_master = m_nh.subscribe(m_topic_name_master + "/data", 1, &BilateralController::masterCallback, this);
        m_sub_slave = m_nh.subscribe(m_topic_name_slave + "/data", 1, &BilateralController::slaveCallback, this);

        for (int i = 0; i < 3; i++) {
            {
                using namespace PIDParams;
                m_position_iir_controller.push_back(IIRFilter{1, m_joint_gain_list.at(i) * std::vector<double>{a0, a1}, std::vector<double>{b1}});
            }
            {
                using namespace Slave::DOBParams::LPF;
                m_force_dob_lpf_slave.push_back(IIRFilter{1, m_force_gain_list.at(i) * std::vector<double>{a0, a1}, std::vector<double>{b1}});
            }
            {
                using namespace Slave::DOBParams::MotorInv;
                m_force_dob_motor_inv_slave.push_back(IIRFilter{2, m_force_gain_list.at(i) * std::vector<double>{a0.at(i), a1.at(i), a2.at(i)}, std::vector<double>{b1, b2}});
            }
            {
                using namespace Slave::RFOBParams;
                m_force_rfob_slave.push_back(IIRFilter{1, m_force_gain_list.at(i) * std::vector<double>{a0, a1}, std::vector<double>{b1}});
            }
        }
    }

    void updateMasterState(const geometry_msgs::Pose& pose, const sensor_msgs::JointState& joint)
    {
        for (int i = 0; i < 3; i++) {
            m_joint_theta_master.at(i) = joint.position[i];
            m_tauref_master.at(i) = joint.effort[i];
        }
        m_master_pose = pose;
    }

    void updateSlaveState(const geometry_msgs::Pose& pose, const sensor_msgs::JointState& joint)
    {
        for (int i = 0; i < 3; i++) {
            m_joint_theta_slave.at(i) = joint.position[i];
            m_tauref_slave.at(i) = joint.effort[i];
        }
        m_slave_pose = pose;
    }
    void forceControl();
    void masterCallback(const omni_msgs::BilateralData::ConstPtr& master_data)
    {
        this->updateMasterState(master_data->pose, master_data->joint_state);
        // 1kHzにするためにslaveからsubしたときのみcontrol
        // this->forceControl();
    }
    void slaveCallback(const omni_msgs::BilateralData::ConstPtr& slave_data)
    {
        this->updateSlaveState(slave_data->pose, slave_data->joint_state);
        this->forceControl();
    }

    // void hoge() { this->nosition_iir_controller.hoge(); }
};
