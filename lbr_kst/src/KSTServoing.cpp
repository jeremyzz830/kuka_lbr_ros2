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

#include <math.h>
#include <string>
#include <iostream>
#include <vector>
#include <unistd.h>
#include <boost/lexical_cast.hpp>

#include "lbr_kst/KSTServoing.hpp"
#include "lbr_kst/UtlFunctions.hpp"
#include "lbr_kst/UtlCalculations.hpp"

// Check acknowledgement from the robot controller side. Usually used after sending an
// encoded command to the robot. Not yet used.
inline bool check_acknowledgement(std::string msg)
{	
	//TODO
	// std::string ak = "done";
	// std::string nak = "nak";
	return true;
}

// Class constructor
KSTServoing::KSTServoing(std::string robot_ip, int robot_type, double h_flange, boost::asio::io_context& io_context) :
	tcp_sock_(io_context)
{
	// Robot parameters
	ip_ = robot_ip;
	data_dh_ = KSTServoing::utl_dh_parameters(robot_type);
	data_I_ = KSTServoing::utl_inertial_parameters(robot_type);

	// Robot type
	if(robot_type==1){
		robot_type_ = "LBR7R800";
		data_dh_.d[6] = data_dh_.d[6] + h_flange;
		// transfer center of mass of last link from elbow frame "convention used by cited studies" to flange frame "convention used in KST"
		data_I_.pcii[2][6] = data_I_.pcii[2][6] - data_dh_.d[6];                    
	}
	else if(robot_type==2){
		//TODO
	}
	else{
		//TODO
	}

	// Robot controller communications
	// tcp::endpoint remote_endpoint = tcp::endpoint(tcp::v4(), 9877); // Test endpoint using localhost
	tcp::endpoint remote_endpoint = tcp::endpoint(boost::asio::ip::address_v4::from_string(robot_ip), 30001); // Controller endpoint
	tcp_sock_.connect(remote_endpoint);

	// Wait a bit to secure the connection
	sleep(1); // C++ sleep from package unistd.h

	// Send connection request
	boost::system::error_code error;
	const std::string msg1 = "TFtrans_0.0_0.0_0.0_0.0_0.0_0.0\n";
	boost::asio::write(tcp_sock_, boost::asio::buffer(msg1), error);
	std::cout << "Tool sent";

	// Receive response
	size_t lenmsgr1 = tcp_sock_.read_some(boost::asio::buffer(buf_));
	
	// Wait a bit to secure the connection
	sleep(0.5); // C++ sleep from package unistd.h

	std::cout << "Kuka Servo Connection Established.";
}

// Point to point command in joint space
bool KSTServoing::PTP_joint_space(std::vector<double> jpos , double relVel)
{
	try
	{
		boost::system::error_code error;

		const std::string msg1 = "jRelVel_" + boost::lexical_cast<std::string>(relVel) + "_\n";
		boost::asio::write(tcp_sock_, boost::asio::buffer(msg1), error);
		size_t lenmsgr1 = tcp_sock_.read_some(boost::asio::buffer(buf_), error);

		const std::string msgjs = "jp_" + 
			UtlFunctions::formated_double_2_string(jpos[0], 5) + "_" +
			UtlFunctions::formated_double_2_string(jpos[1], 5) + "_" +
			UtlFunctions::formated_double_2_string(jpos[2], 5) + "_" +
			UtlFunctions::formated_double_2_string(jpos[3], 5) + "_" +
			UtlFunctions::formated_double_2_string(jpos[4], 5) + "_" +
			UtlFunctions::formated_double_2_string(jpos[5], 5) + "_" +
			UtlFunctions::formated_double_2_string(jpos[6], 5) + "_\n";
		boost::asio::write(tcp_sock_, boost::asio::buffer(msgjs), error);
		size_t lenmsgrjs = tcp_sock_.read_some(boost::asio::buffer(buf_), error);

		const std::string msg2 = "doPTPinJS\n";
		boost::asio::write(tcp_sock_, boost::asio::buffer(msg2), error);
		size_t lenmsgr2 = tcp_sock_.read_some(boost::asio::buffer(buf_), error);

		size_t lenmsgr3 = tcp_sock_.read_some(boost::asio::buffer(buf_), error);

		// bool flag_reading = false;
		// while (flag_reading == false)
		// {
		// 	size_t lenmsgr2 = tcp_sock_.read_some(boost::asio::buffer(buf_), error)ddddd;
		// 	check_acknowledgement
		// }

		return true;
	}
	catch(std::exception& e)
	{
		std::cout << "Command sending encountered problem";
		throw;
	}
}

// Point to point command in end-effector Cartesian space
bool KSTServoing::PTP_line_EEF(std::vector<double> epos, double vel) // vel: mm/sec
{ 
	try
	{
		boost::system::error_code error;
		const std::string msg1 = "jRelVel_" + boost::lexical_cast<std::string>(vel) + "_\n";
		boost::asio::write(tcp_sock_, boost::asio::buffer(msg1), error);
		size_t lenmsgr1 = tcp_sock_.read_some(boost::asio::buffer(buf_), error);

		const std::string msgjs = "cArtixanPosition_" +
			std::to_string(epos[0]) + "_" +
			std::to_string(epos[1]) + "_" +
			std::to_string(epos[2]) + "_" +
			std::to_string(epos[3]) + "_" +
			std::to_string(epos[4]) + "_" +
			std::to_string(epos[5]) + "_\n";
		boost::asio::write(tcp_sock_, boost::asio::buffer(msgjs), error);
		size_t lenmsgrjs = tcp_sock_.read_some(boost::asio::buffer(buf_), error);

		const std::string msg2 = "doPTPinCS\n"; 
		boost::asio::write(tcp_sock_, boost::asio::buffer(msg2), error);
		size_t lenmsgr2 = tcp_sock_.read_some(boost::asio::buffer(buf_), error);

		size_t lenmsgr3 = tcp_sock_.read_some(boost::asio::buffer(buf_), error);

		return true;

	}
	catch(std::exception& e)
	{
		std::cout << "Command sending encountered problem";
		throw;
	}
} 

// Direct servoing in EEF cartesian space. Start command (controller receives the command and expects EEF position streams)
void KSTServoing::servo_direct_cartesian_start()
{
	boost::system::error_code error;
	const std::string msg1 = "stDcEEf_\n";
	boost::asio::write(tcp_sock_, boost::asio::buffer(msg1), error);
	size_t lenmsgr = tcp_sock_.read_some(boost::asio::buffer(buf_), error);
	usleep(1500);
}

// Direct servoing in joint space. Start command (controller receives the command and expects EEF position streams)
void KSTServoing::servo_direct_joint_start()
{
	boost::system::error_code error;
	const std::string msg1 = "startDirectServoJoints\n";
	boost::asio::write(tcp_sock_, boost::asio::buffer(msg1), error);
	size_t lenmsgr = tcp_sock_.read_some(boost::asio::buffer(buf_), error);
	usleep(1500);
}

// Smart servoing in EEF cartesian space. Start command (controller receives the command and expects EEF position streams)
void KSTServoing::servo_smart_cartesian_start()
{
	boost::system::error_code error;
	const std::string msg1 = "stSmtSEEf_\n";
	boost::asio::write(tcp_sock_, boost::asio::buffer(msg1), error);
	size_t lenmsgr = tcp_sock_.read_some(boost::asio::buffer(buf_), error);
	usleep(1500);
}

// Stop servoing
void KSTServoing::servo_stop()
{
	boost::system::error_code error;
	const std::string msg1 = "stopDirectServoJoints\n"; // the same to stop direct servo based on KST
	boost::asio::write(tcp_sock_, boost::asio::buffer(msg1), error);
	size_t lenmsgr = tcp_sock_.read_some(boost::asio::buffer(buf_), error);
}

// Sending the streaming joints to the controller (this need to be after spawning direct/smart servo)
void KSTServoing::servo_send_joints(std::vector<double> jp)
{
	boost::system::error_code error;
	const std::string msg1 = "jf_"+ 
		UtlFunctions::formated_double_2_string(jp[0], 5) + "_" +
		UtlFunctions::formated_double_2_string(jp[1], 5) + "_" +
		UtlFunctions::formated_double_2_string(jp[2], 5) + "_" +
		UtlFunctions::formated_double_2_string(jp[3], 5) + "_" +
		UtlFunctions::formated_double_2_string(jp[4], 5) + "_" +
		UtlFunctions::formated_double_2_string(jp[5], 5) + "_" +
		UtlFunctions::formated_double_2_string(jp[6], 5) + "_\n";
	boost::asio::write(tcp_sock_, boost::asio::buffer(msg1), error);
}

// Sending the streaming joints and get current joints to/from the controller (this need to be after spawning direct/smart servo)
std::vector<double> KSTServoing::servo_send_joints_getfeedback(std::vector<double> jp)
{
	boost::system::error_code error;
	const std::string msg1 = "jpJP_"+ 
		UtlFunctions::formated_double_2_string(jp[0], 5) + "_" +
		UtlFunctions::formated_double_2_string(jp[1], 5) + "_" +
		UtlFunctions::formated_double_2_string(jp[2], 5) + "_" +
		UtlFunctions::formated_double_2_string(jp[3], 5) + "_" +
		UtlFunctions::formated_double_2_string(jp[4], 5) + "_" +
		UtlFunctions::formated_double_2_string(jp[5], 5) + "_" +
		UtlFunctions::formated_double_2_string(jp[6], 5) + "_\n";
	boost::asio::write(tcp_sock_, boost::asio::buffer(msg1), error);

	size_t lenmsgr = tcp_sock_.read_some(boost::asio::buffer(buf_), error);
	std::stringstream ssmsgr;
	std::string strmsgr;
	for(int i=0;i<lenmsgr;i++)
		ssmsgr << buf_[i];
	strmsgr = ssmsgr.str();

	std::vector<double> jpbk = UtlFunctions::parseString2DoubleVec(strmsgr);

	return jpbk;
}

// Sending the streaming EEF cartesian position to the controller (this need to be after spawning direct/smart servo)
void KSTServoing::servo_send_EEF(std::vector<double> eef) // x y z rz ry rx
{
	boost::system::error_code error;
	const std::string msg1 = "DcSeCar_"+ 
		UtlFunctions::formated_double_2_string(eef[0], 5) + "_" +
		UtlFunctions::formated_double_2_string(eef[1], 5) + "_" +
		UtlFunctions::formated_double_2_string(eef[2], 5) + "_" +
		UtlFunctions::formated_double_2_string(eef[3], 5) + "_" +
		UtlFunctions::formated_double_2_string(eef[4], 5) + "_" +
		UtlFunctions::formated_double_2_string(eef[5], 5) + "_\n";
	boost::asio::write(tcp_sock_, boost::asio::buffer(msg1), error);
}

// Sending the streaming EEF cartesian positions and get current position from/to the controller (this need to be after spawning direct/smart servo)
std::vector<double> KSTServoing::servo_send_EEF_getfeedback(std::vector<double> eef)  // x y z rz ry rx
{
	boost::system::error_code error;
	const std::string msg1 = "DcSeCarEEfP_"+ 
		UtlFunctions::formated_double_2_string(eef[0], 5) + "_" +
		UtlFunctions::formated_double_2_string(eef[1], 5) + "_" +
		UtlFunctions::formated_double_2_string(eef[2], 5) + "_" +
		UtlFunctions::formated_double_2_string(eef[3], 5) + "_" +
		UtlFunctions::formated_double_2_string(eef[4], 5) + "_" +
		UtlFunctions::formated_double_2_string(eef[5], 5) + "_\n";
	boost::asio::write(tcp_sock_, boost::asio::buffer(msg1), error);

	size_t lenmsgr = tcp_sock_.read_some(boost::asio::buffer(buf_), error);
	std::stringstream ssmsgr;
	std::string strmsgr;
	for(int i=0;i<lenmsgr;i++)
		ssmsgr << buf_[i];
	strmsgr = ssmsgr.str();

	std::vector<double> eefbk = UtlFunctions::parseString2DoubleVec(strmsgr);

	return eefbk;
}

// get current joint position
std::vector<double> KSTServoing::get_joint_position()
{
	std::string strmsgr;
	try
	{
		boost::system::error_code error;
		const std::string msg = "getJointsPositions\n";
		boost::asio::write(tcp_sock_, boost::asio::buffer(msg), error);
		size_t lenmsgr = tcp_sock_.read_some(boost::asio::buffer(buf_), error);
		std::stringstream ssmsgr;
		for(int i=0;i<lenmsgr;i++)
			ssmsgr << buf_[i];
		ssmsgr << "_"; // this is for getting jpos only, because the kuka java side miss a _
		strmsgr = ssmsgr.str();
	}
	catch(std::exception& e)
	{
		std::cout << "Command sending encountered problem";
		throw;
	}
	// std::cout << strmsgr.c_str());
	std::vector<double> vec = UtlFunctions::parseString2DoubleVec(strmsgr);
	return vec;
}

// get current EEF position
std::vector<double> KSTServoing::get_EEF_position() // x y z rz ry rx
{
	std::string strmsgr;
	try
	{
		boost::system::error_code error;
		const std::string msg = "Eef_pos\n";
		boost::asio::write(tcp_sock_, boost::asio::buffer(msg), error);
		size_t lenmsgr = tcp_sock_.read_some(boost::asio::buffer(buf_), error);
		std::stringstream ssmsgr;
		for(int i=0;i<lenmsgr;i++)
			ssmsgr << buf_[i];
		strmsgr = ssmsgr.str();
	}
	catch(std::exception& e)
	{
		std::cout << "Command sending encountered problem";
		throw;
	}
	std::vector<double> vec = UtlFunctions::parseString2DoubleVec(strmsgr);
	return vec;
}

// stop communication and turnoff the controller java application
void KSTServoing::net_turnoff_server()
{
	boost::system::error_code error;
	const std::string msg1 = "end\n";
	boost::asio::write(tcp_sock_, boost::asio::buffer(msg1), error);
	tcp_sock_.close();
	std::cout << "Robot server shuts down.";
}


KSTServoingDataDH KSTServoing::utl_dh_parameters(int robot_type)
{
	KSTServoingDataDH dh;
	if(robot_type == 1){
		dh.alpha = {0.0, -M_PI/2.0, M_PI/2.0, M_PI/2.0, -M_PI/2.0, -M_PI/2.0, M_PI/2.0};
		dh.d={0.34, 0.0, 0.4, 0.0, 0.4, 0.0, 0.126};
		dh.a={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	}
	else if(robot_type == 2){
		//TODO
	}
	else{
		//TODO
	}
	return dh;
}

KSTServoingDataInertia KSTServoing::utl_inertial_parameters(int robot_type)
{
	KSTServoingDataInertia inrt;
	if(robot_type == 1){
		inrt.m = {3.4525, 3.4821, 4.05623, 3.4822, 2.1633, 2.3466, 3.129};
		inrt.pcii = {   
			{0.0, -0.03441, -0.02, 0.0, 0.0, 0.000001, 0.0000237},
			{0.06949, 0.0, -0.089, -0.034412, 0.14, 0.000485, 0.063866},
			{-0.03423, 0.06733, -0.02906, 0.067329, -0.02137, 0.002115, 0.01464}
		};
        inrt.I = {}; //TODO
    }
    else if(robot_type == 2){
		//TODO
    }
    else{
		//TODO
    }
    return inrt;
}




      

// std::vector<std::vector<double>> KSTServoing::utl_centrifugal_matrix(){

// }

// std::vector<std::vector<double>> KSTServoing::utl_coriolis_matrix(){

// }

// std::vector<std::vector<double>> KSTServoing::utl_mass_matrix(){

// }

// std::vector<std::vector<double>> KSTServoing::utl_nullspace_matrix(){

// }

// std::vector<std::vector<double>> KSTServoing::utl_jacobian_matrix(){

// }

// std::vector<std::vector<double>> KSTServoing::utl_get_dh_matrix(){

// }

// std::vector<double> KSTServoing::utl_gravity_vector(){

// }

// DynamicsForward KSTServoing::utl_forward_dynamics(){

// }

// DynamicsInverse KSTServoing::utl_inverse_dynamics(){

// }

// KinematicsForward KSTServoing::utl_forward_kinematics(){

// }

// KinematicsInverse KSTServoing::utl_inverse_kinematics(){

// }

//         
//         %% Interaction functions
//         function moveWaitForDTWhenInterrupted( this ,...
//                 Pos, VEL,joints_indices,max_torque,min_torque, w)
//             moveWaitForDTWhenInterrupted( this.t_Kuka ,...
//                 Pos, VEL,joints_indices,max_torque,min_torque, w);
//         end

//         function performEventFunctionAtDoubleHit(this) 
//             performEventFunctionAtDoubleHit(this.t_Kuka);
//         end

//         function startHandGuiding( this )
//             startHandGuiding( this.t_Kuka );
//         end

//         function startPreciseHandGuiding( this,wightOfTool,COMofTool )
//             if strcmp(this.RobotType,'LBR7R800')
//                 startPreciseHandGuiding1( this.t_Kuka,wightOfTool,COMofTool );
//             elseif strcmp(this.RobotType,'LBR14R820')
//                 startPreciseHandGuiding2( this.t_Kuka,wightOfTool,COMofTool );
//             end
//         end

//         %% Nonblocking
//         function [ret]=nonBlocking_isGoalReached(this)
//             [ret]=nonBlocking_isGoalReached(this.t_Kuka);
//         end

//         function [flag,feedBack]=nonBlockingCheck_WithFeedback(this, param)
//             [flag,feedBack]=nonBlockingCheck_WithFeedback(this.t_Kuka, param);
//         end 

//         function nonBlocking_movePTPArcXY_AC(this,theta,c,vel)
//             nonBlocking_movePTPArcXY_AC(this.t_Kuka,theta,c,vel);
//         end

//         function nonBlocking_movePTPArcXZ_AC(this,theta,c,vel)
//             nonBlocking_movePTPArcXZ_AC(this.t_Kuka,theta,c,vel);
//         end

//         function nonBlocking_movePTPArcYZ_AC(this,theta,c,vel)
//             nonBlocking_movePTPArcYZ_AC(this.t_Kuka,theta,c,vel);
//         end

//         function nonBlocking_movePTPArc_AC(this,theta,c,k,relVel)
//             nonBlocking_movePTPArc_AC(this.t_Kuka,theta,c,k,relVel);
//         end

//         function nonBlocking_movePTPCirc1OrintationInter( this , f1,f2, relVel)
//             nonBlocking_movePTPCirc1OrintationInter( this.t_Kuka , f1,f2, relVel);
//         end

//         function  nonBlocking_movePTPHomeJointSpace( this , relVel)
//             nonBlocking_movePTPHomeJointSpace( this.t_Kuka , relVel);
//         end

//         function nonBlocking_movePTPJointSpace( this , jPos, relVel)
//             nonBlocking_movePTPJointSpace( this.t_Kuka , jPos, relVel);
//         end

//         function nonBlocking_movePTPLineEEF( this , Pos, relVel)
//             nonBlocking_movePTPLineEEF( this.t_Kuka , Pos, relVel);
//         end

//         function nonBlocking_movePTPTransportPositionJointSpace( this , relVel)
//             nonBlocking_movePTPTransportPositionJointSpace( this.t_Kuka , relVel)
//         end

//         %% PTP motion functions  
//         function [ret]=movePTPArc_AC(this,theta,c,k,relVel)
//             [ret]=movePTPArc_AC(this.t_Kuka,theta,c,k,relVel);
//         end

//         function [ret]=movePTPArcXY_AC(this,theta,c,vel)
//             [ret]=movePTPArcXY_AC(this.t_Kuka,theta,c,vel);
//         end

//         function [ret]=movePTPArcXZ_AC(this,theta,c,vel)
//             [ret]=movePTPArcXZ_AC(this.t_Kuka,theta,c,vel);
//         end

//         function [ret]=movePTPArcYZ_AC(this,theta,c,vel)
//             [ret]=movePTPArcYZ_AC(this.t_Kuka,theta,c,vel);
//         end

//         function [ret]=movePTPCirc1OrintationInter( this , f1,f2, relVel)
//             [ret]=movePTPCirc1OrintationInter( this.t_Kuka , f1,f2, relVel);
//         end

//         function [ state ] = movePTPCirc1OrintationInterCheck( this , f1,f2, relVel)
//             [ state ] = movePTPCirc1OrintationInterCheck( this.t_Kuka , f1,f2, relVel);
//         end 

//         function [ret]=movePTPHomeJointSpace( this , relVel)
//             [ret]=movePTPHomeJointSpace( this.t_Kuka , relVel);
//         end

//         function [ret]=movePTPJointSpace( this , jPos, relVel)
//             [ret]=movePTPJointSpace( this.t_Kuka , jPos, relVel);
//         end

//         function [ret]=movePTPLineEEF( this , Pos, relVel)
//             [ret]=movePTPLineEEF( this.t_Kuka, Pos, relVel);
//         end

//         function [ret]=movePTPLineEefRelBase( this , Pos, relVel)
//             [ret]=movePTPLineEefRelBase( this.t_Kuka , Pos, relVel);
//         end

//         function [ret]=movePTPLineEefRelEef( this , Pos, relVel)
//             [ret]=movePTPLineEefRelEef( this.t_Kuka , Pos, relVel);
//         end

//         function [ret]=movePTPTransportPositionJointSpace( this , relVel)
//             [ret]=movePTPTransportPositionJointSpace( this.t_Kuka , relVel);
//         end

//         % PTP ellipse

//         function movePTPEllipse(this,c,dir,ratio,theta,velocity,accel,TefTool)
//             movePTPEllipse_1(this,c,dir,ratio,theta,velocity,accel,TefTool);
//         end

//         function movePTPEllipse_XY(this,c,ratio,theta,velocity,accel,TefTool)
//             movePTPEllipse_XY_1(this,c,ratio,theta,velocity,accel,TefTool);
//         end

//         function movePTPEllipse_XZ(this,c,ratio,theta,velocity,accel,TefTool)
//             movePTPEllipse_XZ_1(this,c,ratio,theta,velocity,accel,TefTool);
//         end

//         function movePTPEllipse_YZ(this,c,ratio,theta,velocity,accel,TefTool)
//             movePTPEllipse_YZ_1(this,c,ratio,theta,velocity,accel,TefTool);
//         end

//         % conditional
//         function [res]=movePTP_ConditionalTorque_ArcXY_AC(this,theta,c,VEL,joints_indices,max_torque,min_torque)
//             [res]=movePTP_ConditionalTorque_ArcXY_AC(this.t_Kuka,theta,c,VEL,joints_indices,max_torque,min_torque);
//         end

//         function [res]=movePTP_ConditionalTorque_ArcXZ_AC(this,theta,c,VEL,joints_indices,max_torque,min_torque)
//             [res]=movePTP_ConditionalTorque_ArcXZ_AC(this.t_Kuka,theta,c,VEL,joints_indices,max_torque,min_torque);
//         end

//         function [res]=movePTP_ConditionalTorque_ArcYZ_AC(this,theta,c,VEL,joints_indices,max_torque,min_torque)
//             [res]=movePTP_ConditionalTorque_ArcYZ_AC(this.t_Kuka,theta,c,VEL,joints_indices,max_torque,min_torque);
//         end

//         function [res]=movePTP_ConditionalTorque_Arc_AC(this,theta,c,k,VEL,joints_indices,max_torque,min_torque)
//             [res]=movePTP_ConditionalTorque_Arc_AC(this.t_Kuka,theta,c,k,VEL,joints_indices,max_torque,min_torque);
//         end

//         function [res]=movePTP_ConditionalTorque_Circ1OrintationInter( this , f1,f2, VEL,joints_indices,max_torque,min_torque)
//             [res]=movePTP_ConditionalTorque_Circ1OrintationInter( this.t_Kuka , f1,f2, VEL,joints_indices,max_torque,min_torque);
//         end

//         function [res]=movePTP_ConditionalTorque_HomeJointSpace( this , relVel,joints_indices,max_torque,min_torque)
//             [res]=movePTP_ConditionalTorque_HomeJointSpace( this.t_Kuka , relVel,joints_indices,max_torque,min_torque);
//         end

//         function [res]=movePTP_ConditionalTorque_JointSpace( this , jPos, relVel,joints_indices,max_torque,min_torque)
//             [res]=movePTP_ConditionalTorque_JointSpace( this.t_Kuka , jPos, relVel,joints_indices,max_torque,min_torque);
//         end

//         function [res]=movePTP_ConditionalTorque_LineEEF( this , Pos, VEL,joints_indices,max_torque,min_torque)
//             [res]=movePTP_ConditionalTorque_LineEEF( this.t_Kuka , Pos, VEL,joints_indices,max_torque,min_torque);
//         end

//         function [res]=movePTP_ConditionalTorque_LineEefRelBase( this , Pos, VEL,joints_indices,max_torque,min_torque)
//             [res]=movePTP_ConditionalTorque_LineEefRelBase( this.t_Kuka , Pos, VEL,joints_indices,max_torque,min_torque);
//         end

//         function [res]=movePTP_ConditionalTorque_TransportPositionJointSpace( this , relVel,joints_indices,max_torque,min_torque)
//             [res]=movePTP_ConditionalTorque_TransportPositionJointSpace( this.t_Kuka , relVel,joints_indices,max_torque,min_torque);
//         end


//         %% Getters methods
//         function [ Pos ] = getEEFCartesianOrientation( this )
//             [ Pos ] = getEEFCartesianOrientation( this.t_Kuka );
//         end

//         function [ Pos ] = getEEFCartesianPosition( this )
//              [ Pos ] = getEEFCartesianPosition( this.t_Kuka );
//         end

//         function quatrinion= getEEFOrientationQuat(this)
//             quatrinion= getEEFOrientationQuat(this.t_Kuka);
//         end

//         function rMat= getEEFOrientationR(this)
//             rMat= getEEFOrientationR(this.t_Kuka);
//         end

//         function [ Pos ] = getEEFPos( this )
//             [ Pos ] = getEEFPos( this.t_Kuka );
//         end

//         function [ f ] = getEEF_Force( this )
//             [ f ] = getEEF_Force( this.t_Kuka );
//         end

//         function [ f ] = getEEF_Moment( this )
//             [ f ] = getEEF_Moment( this.t_Kuka );
//         end

//         function [ torque ] = getExternalTorqueAtJoint( this,k )
//             [ torque ] = getExternalTorqueAtJoint( this.t_Kuka,k );
//         end

//         function [ torques ] = getJointsExternalTorques( this )
//             [ torques ] = getJointsExternalTorques( this.t_Kuka );
//         end

//         function [ torques ] = getJointsMeasuredTorques( this )
//             [ torques ] = getJointsMeasuredTorques( this.t_Kuka );
//         end

//         function [ jPos ] = getJointsPos( this )
//             [ jPos ] = getJointsPos( this.t_Kuka );
//         end

//         function [ torque ] = getMeasuredTorqueAtJoint( this,k )
//             [ torque ] = getMeasuredTorqueAtJoint( this.t_Kuka,k );
//         end

//         function [ state ] = getPin10State( this )
//             [ state ] = getPin10State( this.t_Kuka );
//         end

//         function [ state ] = getPin13State( this )
//             [ state ] = getPin13State( this.t_Kuka );
//         end

//         function [ state ] = getPin16State( this )
//             [ state ] = getPin16State( this.t_Kuka );
//         end

//         function [ state ] = getPin3State( this )
//             [ state ] = getPin3State( this.t_Kuka );
//         end

//         function [ state ] = getPin4State( this )
//             [ state ] = getPin4State( this.t_Kuka );
//         end




//         %% Soft realtime
//         function [ret]=realTime_moveOnPathInJointSpace( this , trajectory,delayTime)
//             [ret]=realTime_moveOnPathInJointSpace( this.t_Kuka , trajectory,delayTime);
//         end


//         function [ret]=realTime_startImpedanceJoints(this,weightOfTool,cOMx,cOMy,cOMz,...
// cStiness,rStifness,nStifness)
//             [ret]=realTime_startImpedanceJoints(this.t_Kuka,weightOfTool,cOMx,cOMy,cOMz,...
// cStiness,rStifness,nStifness);
//         end

//         function [ret]=realTime_startVelControlJoints( this )
//             [ret]=realTime_startVelControlJoints( this.t_Kuka );
//         end



//         function [ output_args ] = realTime_stopImpedanceJoints( this )
//             [ output_args ] = realTime_stopImpedanceJoints( this.t_Kuka );
//         end

//         function [ret]=realTime_stopVelControlJoints( this )
//             [ret]=realTime_stopVelControlJoints( this.t_Kuka );
//         end       


//         %% Senders methods
//         function [ ret ] = sendEEfPosition( this ,EEEFpos)
//             [ ret ] = sendEEfPosition( this.t_Kuka ,EEEFpos);
//         end        

//         function [ ExTorque ] = sendEEfPositionExTorque( this ,EEEFpos)
//             [ ExTorque ] = sendEEfPositionExTorque( this.t_Kuka ,EEEFpos);
//         end

//         function [ EEFpos ] = sendEEfPositionGetActualEEFpos( this ,EEEFpos)
//             [ EEFpos ] = sendEEfPositionGetActualEEFpos( this.t_Kuka ,EEEFpos);
//         end

//         function [ EEF_force ] = sendEEfPositionGetEEF_Force_rel_EEF( this ,EEEFpos)
//             % "NEW METHOD, ADDED IN THE 9TH OF MAY 2019"
//             [ EEF_force ] = sendEEfPositionGetEEF_Force_rel_EEF( this.t_Kuka ,EEEFpos);
//         end

//         function [ JPOS ] = sendEEfPositionGetActualJpos( this ,EEEFpos)
//             [ JPOS ] = sendEEfPositionGetActualJpos( this.t_Kuka ,EEEFpos);
//         end

//         function [ MT ] = sendEEfPositionMTorque( this ,EEEFpos)
//             [ MT ] = sendEEfPositionMTorque( this.t_Kuka ,EEEFpos);
//         end

//         function sendEEfPositionf( this ,EEEFpos)
//             sendEEfPositionf( this.t_Kuka ,EEEFpos);
//         end

//         function [ output_args ] = sendEEfPositions( this ,jPos)
//             [ output_args ] = sendEEfPositions( this.t_Kuka ,jPos);
//         end

//         function [ ret ] = sendJointsPositions( this ,jPos)
//             [ ret ] = sendJointsPositions( this.t_Kuka ,jPos);
//         end

//         function [ torques ] = sendJointsPositionsExTorque( this ,jPos)
//             [ torques ] = sendJointsPositionsExTorque( this.t_Kuka ,jPos);
//         end

//         function [ EEF_POS ] = sendJointsPositionsGetActualEEFpos( this ,jPos) 
//              [ EEF_POS ] = sendJointsPositionsGetActualEEFpos( this.t_Kuka ,jPos);
//         end

//         function [ EEF_force ] = sendJointsPositionsGetEEF_Force_rel_EEF( this ,jPos) 
//             % "NEW METHOD, ADDED IN THE 9TH OF MAY 2019"
//              [ EEF_force ] = sendJointsPositionsGetEEF_Force_rel_EEF( this.t_Kuka ,jPos);
//         end

//         function [ JPOS ] = sendJointsPositionsGetActualJpos( this ,jPos) 
//             [ JPOS ] = sendJointsPositionsGetActualJpos( this.t_Kuka ,jPos) ;
//         end

//         function [ torques ] = sendJointsPositionsMTorque( this ,jPos) 
//             [ torques ] = sendJointsPositionsMTorque( this.t_Kuka ,jPos) ;
//         end

//         function sendJointsPositionsf( this ,jPos)
//             sendJointsPositionsf( this.t_Kuka ,jPos);
//         end

//         function [ ret ] = sendJointsVelocities( this ,jvel)
//             [ ret ] = sendJointsVelocities( this.t_Kuka ,jvel);
//         end

//         function [ ExT ] = sendJointsVelocitiesExTorques( this ,jvel)
//             [ ExT ] = sendJointsVelocitiesExTorques( this.t_Kuka ,jvel);
//         end

//         function [ EEfPos ] = sendJointsVelocitiesGetActualEEfPos( this ,jvel)
//             [ EEfPos ] = sendJointsVelocitiesGetActualEEfPos( this.t_Kuka ,jvel);
//         end

//         function [ JPoS ] = sendJointsVelocitiesGetActualJpos( this ,jvel)
//             [ JPoS ] = sendJointsVelocitiesGetActualJpos( this.t_Kuka ,jvel);
//         end

//         function [ MT ] = sendJointsVelocitiesMTorques( this ,jvel)
//             [ MT ] = sendJointsVelocitiesMTorques( this.t_Kuka ,jvel);
//         end

//         %% Setters methods
//         function [ ret ] = setBlueOff( this )
//             ret=setBlueOff(this.t_Kuka);
//         end

//         function [ ret ] = setBlueOn( this )
//             ret= setBlueOn( this.t_Kuka );     
//         end    

//         function [ ret ] = setPin11Off( this )
//             ret= setPin11Off( this.t_Kuka );
//         end

//         function [ ret ] = setPin11On( this )
//             ret=setPin11On( this.t_Kuka );
//         end

//         function [ ret ] = setPin12Off( this )
//             ret= setPin12Off( this.t_Kuka );
//         end

//         function [ ret ] = setPin12On( this )
//             ret=setPin12On( this.t_Kuka );
//         end

//         function [ ret ] = setPin1Off( this )
//             ret= setPin1Off( this.t_Kuka );
//         end

//         function [ ret ] = setPin1On( this )
//             ret=setPin1On( this.t_Kuka );
//         end

//         function [ ret ] = setPin2Off( this )
//             ret= setPin2Off( this.t_Kuka );
//         end

//         function [ ret ] = setPin2On( this )
//             ret=setPin2On( this.t_Kuka );
//         end

//        %% networking methods
//         function [flag]=net_establishConnection( this )
//             IP=this.ip;
//             [ t ] = net_establishConnection( IP );
//             flag=0;
//             if ~exist('t','var') || isempty(t) || strcmp(t.Status,'closed')
//               error('Connection could not be establised, script aborted');
//             else
//                 disp('Connection established')
//                 this.t_Kuka=t;
//                 T=this.Teftool;
//                 transForm(1)=T(1,4)*1000;
//                 transForm(2)=T(2,4)*1000;
//                 transForm(3)=T(3,4)*1000; % convert to (mm)
//                 rot_angles_vec=rot2eulZYX(T(1:3,1:3));
//                 transForm(4)=rot_angles_vec(1);
//                 transForm(5)=rot_angles_vec(2);
//                 transForm(6)=rot_angles_vec(3);
//                 [y]=attachToolToFlange(t,transForm);
//                 pause(0.4);
//                 if(y==true)
//                     flag=1;
//                 else
//                     flag=0;
//                 end
//             end
//         end

//         function net_pingIIWA(this)
//             IP=this.ip;
//             net_pingIIWA(IP);
//         end

//         function net_turnOffServer( this )
//             t_kuka=this.t_Kuka;
//             net_turnOffServer( t_kuka );
//         end

//         function [time_stamps,time_delay]=net_updateDelay(this)
//             t_kuka=this.t_Kuka;
//             [time_stamps,time_delay]=net_updateDelay(t_kuka);
//         end
//        %% General porpuse methods
//         function [C]=gen_CentrifugalMatrix(this,q,dq)
//             mcii=this.I_data.m;
//             Pcii=this.I_data.pcii;
//             Icii=this.I_data.I;
//             dh=this.dh_data;
//             [T]=kukaGetKenimaticModelAccelerated_1(q,dh);
//             C=GetCentrifugalMatrix_2(T,Pcii,Icii,mcii,dq);
//         end

//         function [B]=gen_CoriolisMatrix(this,q,dq)
//             mcii=this.I_data.m;
//             Pcii=this.I_data.pcii;
//             Icii=this.I_data.I;
//             dh=this.dh_data;
//             T=kukaGetKenimaticModelAccelerated_1(q,dh);
//             B=GetCoriolisMatrix1(T,Pcii,Icii,mcii,dq);
//         end

//         function [ d2q ] = gen_DirectDynamics(this,q,dq,taw)
//             [M]=this.gen_MassMatrix(q);
//             [B]=this.gen_CoriolisMatrix(q,dq);
//             [ G ] = this.gen_GravityVector(q);
//             % convert angular velocity/acceleration into column vectors
//             dq=this.columnVec(dq);
//             taw=this.columnVec(taw);
//             i=this.I_data;
//             d2q=M\(taw-B*dq-G-i.Fiv.*dq-i.Fis.*sign(dq));
//         end

//         function [eef_transform,J]=directKinematics(this,q)
//             [eef_transform,J]=directKinematics_1(q,this.Teftool,this.dh_data);
//         end

//         function [eef_transform,J]=gen_DirectKinematics(this,q)
//             [eef_transform,J]=directKinematics_1(q,this.Teftool,this.dh_data);
//         end

//         function [ G ] = gen_GravityVector(this,q)
//             mcii=this.I_data.m;
//             Pcii=this.I_data.pcii;
//             G=zeros(7,1);
//             g=[0;0;9.81]; % gravity acceleration vector described in base frame of the robot
//             dh=this.dh_data;
//             T=kukaGetKenimaticModelAccelerated_1(q,dh);

//             % loop through the joints
//             for j=1:7
//                 mj=zeros(3,1);
//                 pj=T(1:3,4,j);
//                 % calculate the moment
//                 for i=7:-1:j
//                     pci=T(1:3,4,i)+T(1:3,1:3,i)*Pcii(:,i);
//                     pcij=pci-pj;
//                     mj=mj+cross(pcij,mcii(i)*g);
//                 end
//                 G(j)=T(1:3,3,j)'*mj;
//             end
//         end

//         function [ taw ] = gen_InverseDynamics(this,q,dq,d2q)
//             [M]=this.gen_MassMatrix(q);
//             [B]=this.gen_CoriolisMatrix(q,dq);
//             [ G ] = this.gen_GravityVector(q);
//             % convert angular velocity/acceleration into column vectors
//             dq=this.columnVec(dq);
//             d2q=this.columnVec(d2q);
//             i=this.I_data;
//             taw=M*d2q+B*dq+G+i.Fiv.*dq+i.Fis.*sign(dq);
//         end

//         function y=columnVec(this,x)
//             if(size(x,2)==1)
//                 y=x;
//             else
//                 y=x';
//             end
//         end

//         function [ qs ]=gen_InverseKinematics(this, qin, Tt,n,lambda )
//             dh=this.dh_data;
//             Teftool=this.Teftool;
//             [ qs ] = kukaDLSSolver_1( qin, Tt, Teftool,n,lambda,dh );
//         end

//         function [M]=gen_MassMatrix(this,q)
//            mcii=this.I_data.m;
//             Pcii=this.I_data.pcii;
//             Icii=this.I_data.I;
//             dh=this.dh_data;
//             T=kukaGetKenimaticModelAccelerated_1(q,dh);
//             [M]=GetInertiaMatrix5(T,Pcii,Icii,mcii);
//         end

//         function [ N ] = gen_NullSpaceMatrix(this,q)
//             [eef_transform,J]=this.gen_DirectKinematics(q);
//             N=eye(7)-J'*inv(J*J')*J;
//         end

//          function [J]=gen_partialJacobean(this,q,linkNum,Pos)
//             if linkNum>7
//                 disp('Error linkNum shall be less than 8');
//                 j=[];
//                 return;
//             end
//             if linkNum<1
//                 disp('Error linkNum shall be more than 0');
//                 j=[];
//                 return;
//             end

//             alfa=this.dh_data.alfa;
//             d=this.dh_data.d; 
//             a=this.dh_data.a;

//             T=zeros(4,4,7);
//             i=1;
//             T(:,:,i)=this.getDHMatrix(alfa{i},q(i),d{i},a{i});
//                 for i=2:7
//                     T(:,:,i)=T(:,:,i-1)*this.getDHMatrix(alfa{i},q(i),d{i},a{i});
//                     T(:,:,i)=this.normalizeColumns(T(:,:,i));
//                 end
//                     T(:,:,7)=T(:,:,7)*this.Teftool;
//                     T(:,:,7)=this.normalizeColumns(T(:,:,7));

//             % parital jacobean
//                 J=zeros(6,linkNum);
//                 pef=T(1:3,1:3,linkNum)*Pos+T(1:3,4,linkNum);
//                 for i=1:linkNum
//                     k=T(1:3,3,i);
//                     pij=pef-T(1:3,4,i);
//                     J(1:3,i)=cross(k,pij);
//                     J(4:6,i)=k;
//                 end
//          end

//     	function T=getDHMatrix(this,alfa,theta,d,a)
//                 T=zeros(4,4);

//                 calpha=cos(alfa);
//                 sinalpha=sin(alfa);
//                 coshteta=cos(theta);
//                 sintheta=sin(theta);

//                 T(1,1)=coshteta;
//                 T(2,1)=sintheta*calpha;
//                 T(3,1)=sintheta*sinalpha;				

//                 T(1,2)=-sintheta;
//                 T(2,2)=coshteta*calpha;
//                 T(3,2)=coshteta*sinalpha;

//                 T(2,3)=-sinalpha;
//                 T(3,3)=calpha;

//                 T(1,4)=a;
//                 T(2,4)=-sinalpha*d;
//                 T(3,4)=calpha*d;
//                 T(4,4)=1;

//             end

//         function normalizedT=normalizeColumns(this,T)
//             r=zeros(4,3); 
//             for j=1:3
//                 r(1:3,j)=T(1:3,j)/norm(T(1:3,j));
//             end
//             normalizedT=[r,T(:,4)];
//         end       

// end
