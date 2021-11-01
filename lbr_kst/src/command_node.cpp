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
#include "lbr_kst/KSTServoingCmd.hpp"
#include "lbr_kst/UtlCalculations.hpp"



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin();
  rclcpp::shutdown();
  return 0;
}
