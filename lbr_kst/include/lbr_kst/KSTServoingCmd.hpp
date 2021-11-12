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


#pragma once

#include "lbr_kst/KSTServoing.hpp"

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "lbr_kst/srv/get_joints.hpp"
#include "lbr_kst/srv/get_eef.hpp"
#include "lbr_kst/srv/smt_servo_start_eef.hpp"
#include "lbr_kst/srv/smt_servo_stop.hpp"

#include "lbr_kst/action/ptp_joint_space.hpp"
#include "lbr_kst/action/ptp_line_eef.hpp"
#include "lbr_kst/action/netiiwa_close.hpp"


class KSTServoingCmd
{

public:

	KSTServoingCmd(KSTServoing& servo);
	
	void move_PTPJointSpace(const lbr_kst::action::PTPJointSpace_Goal::ConstPtr &goal);
	void move_PTPLineEEF(const lbr_kst::action::PTPLineEEF_Goal::ConstPtr &goal);
	void rqst_NetiiwaClose(const lbr_kst::action::NetiiwaClose_Goal::ConstPtr &goal);

	bool get_joints(lbr_kst::srv::GetJoints_Request &req, lbr_kst::srv::GetJoints_Response &res);
	bool get_EEF(lbr_kst::srv::GetEEF_Request &req, lbr_kst::srv::GetEEF_Response &res);
	bool smtSrvo_startEEF(lbr_kst::srv::SmtServoStartEEF_Request &req, lbr_kst::srv::SmtServoStartEEF_Response &res);
	bool smtSrvo_stop(lbr_kst::srv::SmtServoStop_Request &req, lbr_kst::srv::SmtServoStop_Response &res);

	rclcpp::Publisher pub_oldeef_;

private:

	KSTServoing& servo_;

	rclcpp_action::Server<lbr_kst::action::PTPJointSpace> srv_action_PTPjs_;
	rclcpp_action::Server<lbr_kst::action::PTPLineEEF> srv_action_PTPeef_;
	rclcpp_action::Server<lbr_kst::action::NetiiwaClose> srv_action_Netcl_;
	
	lbr_kst::action::PTPJointSpace_Result result_PTPjs_;
	lbr_kst::action::PTPLineEEF_Result result_PTPeef_;
	lbr_kst::action::NetiiwaClose_Result result_Netclos_;

	rclcpp::Service<lbr_kst::srv::GetJoints>::SharedPtr srv_srvc_getjp_;
	rclcpp::Service<lbr_kst::srv::GetEEF>::SharedPtr srv_srvc_geteef_;
	rclcpp::Service<lbr_kst::srv::SmtServoStartEEF>::SharedPtr srv_srvc_SmtSrvoStartEEF_;
	rclcpp::Service<lbr_kst::srv::SmtServoStop>::SharedPtr srv_srvc_SmtSrvoStop_;

	rclcpp::Publisher pub_flagRealtime_;
};