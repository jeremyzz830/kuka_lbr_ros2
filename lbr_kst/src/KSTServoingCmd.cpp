/* *********************** 
* By Yihao Liu, Joshua Liu
* Johns Hopkins University
* Updated 11/1/2021
* 
* MIT licence
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS 
* IN THE SOFTWARE.
* 
************************** */

#include "lbr_kst/KSTServoingCmd.hpp"
// #include "lbr_kst/KSTServoing.hpp"


#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "lbr_kst/msg/joint_position.hpp"
#include "lbr_kst/msg/eef_cartesian.hpp"

#include "lbr_kst/srv/get_joints.hpp"
#include "lbr_kst/srv/get_eef.hpp"
#include "lbr_kst/srv/smt_servo_start_eef.hpp"
#include "lbr_kst/srv/smt_servo_stop.hpp"

#include "lbr_kst/action/ptp_joint_space.hpp"
#include "lbr_kst/action/ptp_line_eef.hpp"
#include "lbr_kst/action/netiiwa_close.hpp"

KSTServoingCmd::KSTServoingCmd(KSTServoing& servo) : 
	servo_(servo) ,
	srv_action_PTPjs_("/TMSKuka/movePTPJointSpace", boost::bind(&KSTServoingCmd::move_PTPJointSpace, this, _1), false) , // spawn up PTP_joint_space action server
	srv_action_PTPeef_("/TMSKuka/movePTPLineEEF", boost::bind(&KSTServoingCmd::move_PTPLineEEF, this, _1), false) ,
	srv_action_Netcl_("/TMSKuka/rqstNetiiwaClose", boost::bind(&KSTServoingCmd::rqst_NetiiwaClose, this, _1), false)
{
	srv_srvc_getjp_ = n_.advertiseService("/TMSKuka/GetJoints", &KSTServoingCmd::get_joints, this); // spawn up get_joints ros service server
	srv_srvc_geteef_ = n_.advertiseService("/TMSKuka/GetEEF", &KSTServoingCmd::get_EEF, this);
	srv_srvc_SmtSrvoStartEEF_ = n_.advertiseService("/TMSKuka/SmtSrvoStartEEF", &KSTServoingCmd::smtSrvo_startEEF, this);
	srv_srvc_SmtSrvoStop_ = n_.advertiseService("/TMSKuka/SmtSrvoStop", &KSTServoingCmd::smtSrvo_stop, this);

	pub_flagRealtime_ = n_.advertise<std_msgs::Int32>("/TMSKuka/realtimeModeChange", 100);
	pub_oldeef_ = n_.advertise<geometry_msgs::TransformStamped>("/TMSKuka/realtimeOldEEF", 100);

	srv_action_PTPjs_.start();
	srv_action_PTPeef_.start();
	srv_action_Netcl_.start();
	ROS_INFO("Servers initialized");
}

bool KSTServoingCmd::get_joints(lbr_kst::GetJoints::Request &req, lbr_kst::GetJoints::Response &res)
{
	lbr_kst::JointPosition jp = servo_.get_joint_position();
	res.jp = jp;
	return true;
}


bool KSTServoingCmd::get_EEF(lbr_kst::GetEEF::Request &req, lbr_kst::GetEEF::Response &res)
{
	geometry_msgs::TransformStamped eef = servo_.get_EEF_position();
	res.eef = eef;
	return true;
}

bool KSTServoingCmd::smtSrvo_startEEF(lbr_kst::SmtSrvoStartEEF::Request &req, lbr_kst::SmtSrvoStartEEF::Response &res)
{
	
	servo_.servo_smart_cartesian_start();

	servo_.flg_realtimectl_ = true;
	flg_TMSrealtimectl_ = true;

	std_msgs::Int32 msg;
	msg.data = 1;
	pub_flagRealtime_.publish(msg);

	return true;
}

bool KSTServoingCmd::smtSrvo_stop(lbr_kst::SmtSrvoStop::Request &req, lbr_kst::SmtSrvoStop::Response &res)
{
	servo_.servo_stop();

	servo_.flg_realtimectl_ = false;
	flg_TMSrealtimectl_ = false;

	std_msgs::Int32 msg;
	msg.data = 0;
	pub_flagRealtime_.publish(msg);

	return true;
}

void KSTServoingCmd::move_PTPJointSpace(const lbr_kst::PTPJointSpaceGoalConstPtr& goal)
{
	lbr_kst::JointPosition jp = goal->jp;
	std::vector<double> jpos{
		jp.a1, 
		jp.a2, 
		jp.a3, 
		jp.a4, 
		jp.a5, 
		jp.a6, 
		jp.a7 
	};
	double relVel = 0.02;
	servo_.PTP_joint_space(jpos , relVel);
	result_PTPjs_.success = true;
	srv_action_PTPjs_.setSucceeded(result_PTPjs_);
}

void KSTServoingCmd::move_PTPLineEEF(const lbr_kst::PTPLineEEFGoalConstPtr& goal)
{
	lbr_kst::EEFCartesian eef = goal->eef;
	std::vector<double> eefpos{
		eef.x,
		eef.y,
		eef.z,
		eef.rz,
		eef.ry,
		eef.rx
	};
	double vel = 12.0; // mm/sec
	servo_.PTP_line_EEF(eefpos, vel);
	result_PTPeef_.success = true;
	srv_action_PTPeef_.setSucceeded(result_PTPeef_);
}


void KSTServoingCmd::rqst_NetiiwaClose(const lbr_kst::NetiiwaCloseGoalConstPtr& goal)
{

	ROS_INFO("Turning off requested.");
	servo_.net_turnoff_server();
	result_Netclos_.success = true;
	srv_action_Netcl_.setSucceeded(result_Netclos_);
}


bool KSTServoingCmd::getflg_TMSrealtimectl()
{
	return flg_TMSrealtimectl_;
}