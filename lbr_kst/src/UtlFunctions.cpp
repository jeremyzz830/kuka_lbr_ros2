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

#include "lbr_kst/UtlFunctions.hpp"

#include <iomanip>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>


// utility functions

std::string UtlFunctions::formated_double_2_string(double a, int dec)
{
	std::stringstream stream;
    stream << std::fixed << std::setprecision(dec) << a;
    std::string s = stream.str();
    return s;
}

std::vector<double> UtlFunctions::parseString2DoubleVec(std::string s)
{
	std::vector<double> vec;
	std::string delimiter = "_";
	size_t pos = 0;
	while ((pos = s.find(delimiter)) != std::string::npos) 
	{
	    vec.push_back(std::stod(s.substr(0, pos)));
	    s.erase(0, pos + delimiter.length());
	}
	return vec;
}
