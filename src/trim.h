/*********************************************************************
 *  Software License Agreement (CC BY-SA 3.0)
 *  Author: Evan Teran / evan-teran
 *  URL: https://stackoverflow.com/a/217605
 *
 *  String-Trimming Lambdas (c) by Evan Teran
 *
 *  String-Trimming Lambdas is licensed under a
 *  Creative Commons Attribution-ShareAlike 3.0 Unported License.
 *
 *  You should have received a copy of the license along with this
 *  work. If not, see <http://creativecommons.org/licenses/by-sa/3.0/>.
 *********************************************************************/

#ifndef SRDF_TRIM_
#define SRDF_TRIM_

#include <string>

namespace srdf
{

// trim from start (in place)
static inline void ltrim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](int ch) {
        return !std::isspace(ch);
    }));
}
// trim from end (in place)
static inline void rtrim(std::string &s) {
    s.erase(std::find_if(s.rbegin(), s.rend(), [](int ch) {
        return !std::isspace(ch);
    }).base(), s.end());
}
// trim from both ends (in place)
static inline void trim(std::string &s) {
    ltrim(s);
    rtrim(s);
}
// trim from both ends (copying)
static inline std::string trim_copy(std::string s) {
    trim(s);
    return s;
}

}
#endif
