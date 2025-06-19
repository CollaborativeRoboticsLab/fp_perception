#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>

namespace perception
{

enum class RESTOptionType
{
  STRING = 0,
  BOOL,
  INT,
  DOUBLE
};

// prompt option
struct RESTOption
{
  std::string key;
  std::string value;
  RESTOptionType type;
};

// prompt provider request
struct RESTRequest
{
  std::string prompt;
  std::string file_type;
  std::vector<char> file_stream;
  std::vector<RESTOption> options;
};

// prompt provider response
struct RESTResponse
{
  std::string response;
  std::vector<RESTOption> options;
  std::vector<int16_t> audio_stream;  // For audio responses
  bool success{ false };
  double accuracy{ 0.0 };
  double confidence{ 0.0 };
  double risk{ 0.0 };
};

}  // namespace perception
