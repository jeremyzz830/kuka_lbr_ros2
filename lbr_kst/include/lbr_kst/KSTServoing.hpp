/* *********************** 
* By Yihao Liu, Joshua Liu
* Johns Hopkins University
* Updated 10/30/2021
* 
* MIT licence
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS 
* IN THE SOFTWARE.
* 
************************** */

#pragma once

#include <vector>
#include <boost/asio.hpp>

#include "rclcpp/rclcpp.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "lbr_kst/msg/joint_position.hpp"

using boost::asio::ip::tcp;

struct kKSTServoingRobot{
	// type of the robot
    const int LBR7R800=1;
    const int LBR14R820=2;
    // height of flange (unit meter), taken from iiwa data sheets
    // Manual: 
    // [1] Medien-Flansch
    // Für Produktfamilie LBR iiwa
    // Montage- und Betriebsanleitung
    const double MEDIEN_FLANSCH_ELEKTRISCH= 0.035;
    const double MEDIEN_FLANSCH_PNEUMATISCH= 0.035;
    const double MEDIEN_FLANSCH_IO_PNEUMATISCH= 0.035;
    const double MEDIEN_FLANSCH_TOUCH_PNEUMATISCH= 0.061;
    const double MEDIEN_NONE=0.0;
};

struct KSTServoingDataInertia{
    std::vector<double> m;
    std::vector<std::vector<double>> pcii;
    std::vector<std::vector<std::vector<double>>> I;
};

struct KSTServoingDataDH{
    std::vector<double> alpha;
    std::vector<double> d;
    std::vector<double> a;
};

struct KSTServoingKinematicsForward{

};

struct KSTServoingKinematicsInverse{

};

struct KSTServoingDynamicsForward{

};

struct KSTServoingDynamicsInverse{

};

class KSTServoing{

private:

    // connection
    std::string ip_;
    tcp::socket tcp_sock_; // tcpip connection object
    boost::array<char, 128> buf_;

    //robot data
    KSTServoingDataInertia data_I_; // inertial data of the robot
    KSTServoingDataDH data_dh_; // DH parameters of the robot, combination
    std::string robot_type_;
    int flange_type_;

    tf2::Transform teftool_ = tf2::Transform( // end effector tranform
    	tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(0.0, 0.0, 0.0)); 

public:

    // constructor
    KSTServoing(
        std::string robot_ip, 
        int robot_type, 
        double h_flange, 
        boost::asio::io_context& io_context,
        );
    
    // PTP motion
    bool PTP_joint_space(std::vector<double> jpos , double relVel);
    bool PTP_line_EEF(std::vector<double> epos, double vel); // vel: mm/sec

    // Smart servo
    void servo_direct_cartesian_start();
    void servo_direct_joint_start();
    void servo_smart_cartesian_start();
    void servo_stop();

    void servo_send_joints(std::vector<double> jp);
    std::vector<double> servo_send_joints_getfeedback(std::vector<double> jp);
    void servo_send_EEF(std::vector<double> eef); // x y z rz ry rx
    std::vector<double> servo_send_EEF_getfeedback(std::vector<double> eef); // x y z rz ry rx

    // getters
    lbr_kst::msg::JointPosition get_joint_position();
    geometry_msgs::msg::TransformStamped get_EEF_position();
        
    // networking
    bool net_establish_connection();
    void net_turnoff_server();

    // utility/member methods
    // std::vector<std::vector<double>> utl_centrifugal_matrix();
    // std::vector<std::vector<double>> utl_coriolis_matrix();
    // std::vector<std::vector<double>> utl_mass_matrix();
    // std::vector<std::vector<double>> utl_nullspace_matrix();
    // std::vector<std::vector<double>> utl_jacobian_matrix();
    // std::vector<std::vector<double>> utl_dh_matrix();
    // std::vector<double> utl_gravity_vector();
    // KSTServoingDynamicsForward utl_forward_dynamics();
    // KSTServoingDynamicsInverse utl_inverse_dynamics();
    // KSTServoingKinematicsForward utl_forward_kinematics();
    // KSTServoingKinematicsInverse utl_inverse_kinematics();
    KSTServoingDataDH utl_dh_parameters(int robot_type);
    KSTServoingDataInertia utl_inertial_parameters(int robot_type);

    // flags
    bool flg_realtimectl_ = false;
};
