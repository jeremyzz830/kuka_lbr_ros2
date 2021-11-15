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


#include <string>
#include <boost/asio.hpp>

#include "rclcpp/rclcpp.hpp"

#include "lbr_kst/KSTServoing.hpp"
// #include "lbr_kst/KSTServoingCmd.hpp"
// #include "lbr_kst/UtlCalculations.hpp"


class TestKSTServoing : public rclcpp::Node
{
  public:
    TestKSTServoing()
    : Node("test_KST_servoing")
    {
      std::cout << "Hello World, KST Servoing test node!";
    }
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  kKSTServoingRobot r_param;
	std::string robot_ip = "172.31.1.147";
	// std::string robot_ip = "127.0.0.1";
	int robot_type = r_param.LBR7R800;
	double h_flange = r_param.MEDIEN_FLANSCH_ELEKTRISCH;
	boost::asio::io_context io_context;
  KSTServoing servo(robot_ip, robot_type, h_flange, io_context);

  rclcpp::spin(std::make_shared<TestKSTServoing>());
  rclcpp::shutdown();
  return 0;
}
