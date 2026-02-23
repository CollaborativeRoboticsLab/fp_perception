#pragma once

#include <string>
#include <curl/curl.h>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <perception_base/driver_base.hpp>
#include <perception_base/exceptions.hpp>
#include <perception_base/rest/structs.hpp>
#include <perception_base/rest/file_part_source.hpp>

namespace perception
{
/**
 * @brief RestBase
 *
 * This class implements a base class for RESTful perception plugins.
 * It provides a common interface for sending requests and receiving responses.
 */

class RestBase : public DriverBase
{
public:
  /**
   * @brief Constructor
   *
   * Initializes the RestBase with default values.
   */
  RestBase()
  {
  }

  /**
   * @brief Destructor
   *
   * Cleans up resources used by the RestBase.
   */
  virtual ~RestBase()
  {
  }

  /**
   * @brief Initialize the REST base class
   *
   * This method initializes the REST base class with parameters from the ROS parameter server.
   *
   * @param node rclcpp::Node::SharedPtr ROS node
   * @throws perception_exception if there is an error during initialization
   */
  virtual void initialize_rest_base(const rclcpp::Node::SharedPtr& node, std::string plugin_name = "RestBase",
                                    std::string api_key_name = "")
  {
    // Declare the plugin name parameter
    plugin_name_ = plugin_name;

    // Initialize driver base
    initialize_base(node);

    // Declare parameters
    node_->declare_parameter(plugin_name_ + ".rest.uri", "http://localhost:8000/api/v1/perception");
    node_->declare_parameter(plugin_name_ + ".rest.method", "POST");
    node_->declare_parameter(plugin_name_ + ".rest.ssl_verify", true);
    node_->declare_parameter(plugin_name_ + ".rest.auth_type", "Bearer");

    // Get parameters from the parameter server

    uri_ = node_->get_parameter(plugin_name_ + ".rest.uri").as_string();
    method_ = node_->get_parameter(plugin_name_ + ".rest.method").as_string();
    ssl_verify_ = node_->get_parameter(plugin_name_ + ".rest.ssl_verify").as_bool();
    auth_type_ = node_->get_parameter(plugin_name_ + ".rest.auth_type").as_string();

    // Log the parameters
    RCLCPP_INFO(node_->get_logger(), "Assigned driver URI: %s", uri_.c_str());
    RCLCPP_INFO(node_->get_logger(), "Assigned driver Method: %s", method_.c_str());
    RCLCPP_INFO(node_->get_logger(), "Assigned driver SSL Verify: %s", ssl_verify_ ? "true" : "false");
    RCLCPP_INFO(node_->get_logger(), "Assigned driver Auth Type: %s", auth_type_.c_str());

    // Load api key from environment
    if (!api_key_name.empty())
    {
      const char* api_key_env = std::getenv(api_key_name.c_str());
      if (api_key_env)
      {
        api_key_ = api_key_env;
        RCLCPP_INFO(node_->get_logger(), "API key loaded from environment variables: %s", api_key_name.c_str());
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "missing env variable: %s", api_key_name.c_str());
        throw perception::perception_exception("missing env variable: " + api_key_name);
        api_key_ = "";
      }
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "An API key is not used for this plugin, using empty string");
      api_key_ = "";
    }
  }

  /**
   * @brief Request data from the REST API
   *
   * This method sends a RESTRequest to the REST API and returns the response.
   *
   * @param req The REST request containing the prompt and options.
   * @return A RESTResponse containing the response data.
   * @throws perception_exception if there is an error during the request.
   */
  virtual perception::RESTResponse call(const perception::RESTRequest& req)
  {
    // Build JSON body from RESTRequest
    nlohmann::json body_json = toJson(req);  // <- Custom method returning nlohmann::json

    std::string json_body = body_json.dump();
    std::string response_data;

    CURL* curl = curl_easy_init();
    if (!curl)
      throw perception::perception_exception("Failed to initialize libcurl");

    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");

    if (auth_type_ == "Bearer")
    {
      std::string auth_header = "Authorization: Bearer " + api_key_;
      headers = curl_slist_append(headers, auth_header.c_str());
    }
    else
    {
      curl_slist_free_all(headers);
      curl_easy_cleanup(curl);
      throw perception::perception_exception("Unsupported auth type: " + auth_type_);
    }

    curl_easy_setopt(curl, CURLOPT_URL, uri_.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_body.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, json_body.size());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_data);

    if (!ssl_verify_)
    {
      curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0L);
      curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0L);
    }

    CURLcode res = curl_easy_perform(curl);
    if (res != CURLE_OK)
    {
      curl_slist_free_all(headers);
      curl_easy_cleanup(curl);
      throw perception::perception_exception("cURL error: " + std::string(curl_easy_strerror(res)));
    }

    long http_code = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);

    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);

    if (http_code != 200)
    {
      std::string details;

      // Try to extract a useful error message from JSON error payloads.
      try
      {
        auto err_json = nlohmann::json::parse(response_data);
        if (err_json.contains("error"))
        {
          const auto& err = err_json["error"];
          if (err.is_object() && err.contains("message"))
          {
            if (err["message"].is_string())
              details = err["message"].get<std::string>();
            else
              details = err["message"].dump();
          }
          else
          {
            details = err.dump();
          }
        }
      }
      catch (...) {}

      // Fall back to a truncated raw body if parsing didn't work.
      if (details.empty() && !response_data.empty())
      {
        constexpr size_t kMaxLen = 1024;
        details = response_data.substr(0, std::min(kMaxLen, response_data.size()));
      }

      std::string msg = "HTTP error: " + std::to_string(http_code);
      if (!details.empty())
        msg += " - " + details;

      throw perception::perception_exception(msg);
    }

    // Parse the JSON response
    try
    {
      nlohmann::json response_json = nlohmann::json::parse(response_data);
      return fromJson(response_json);  // <- Custom method converting json to RESTResponse
    }
    catch (const std::exception& e)
    {
      throw perception::perception_exception("JSON parse error: " + std::string(e.what()));
    }
  }

  /**
   * @brief Request audio data from the REST API
   *
   * This method sends an audio file to the REST API and returns the response.
   *
   * @param req The REST request containing the audio file and options.
   * @return A RESTResponse containing the response data.
   * @throws perception_exception if there is an error during the request.
   */
  virtual perception::RESTResponse call_audio(const perception::RESTRequest& req)
  {
    CURL* curl = curl_easy_init();
    if (!curl)
      throw perception_exception("Failed to initialize libcurl");

    struct curl_httppost* form = nullptr;
    struct curl_httppost* last = nullptr;

    // Add options as form fields
    for (const auto& opt : req.options)
    {
      curl_formadd(&form, &last, CURLFORM_COPYNAME, opt.key.c_str(), CURLFORM_COPYCONTENTS, opt.value.c_str(),
                   CURLFORM_END);
    }

    // Add audio data as a file
    curl_formadd(&form, &last, CURLFORM_COPYNAME, "file", CURLFORM_BUFFER, "audio.wav", CURLFORM_BUFFERPTR,
                 reinterpret_cast<const void*>(req.file_stream.data()), CURLFORM_BUFFERLENGTH, req.file_stream.size(),
                 CURLFORM_CONTENTTYPE, req.file_type.c_str(), CURLFORM_END);

    std::string response_data;

    // Set curl options
    curl_easy_setopt(curl, CURLOPT_URL, uri_.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPPOST, form);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_data);

    // Handle SSL verification
    if (!ssl_verify_)
    {
      curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0L);
      curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0L);
    }

    // Set authorization header
    struct curl_slist* headers = nullptr;
    if (auth_type_ == "Bearer")
    {
      std::string auth_header = "Authorization: Bearer " + api_key_;
      headers = curl_slist_append(headers, auth_header.c_str());
      curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    }
    else
    {
      curl_formfree(form);
      curl_easy_cleanup(curl);
      throw perception_exception("Unsupported auth type: " + auth_type_);
    }

    // Perform request
    CURLcode res = curl_easy_perform(curl);

    // Get HTTP response code
    long http_code = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);

    // Cleanup
    curl_slist_free_all(headers);
    curl_formfree(form);
    curl_easy_cleanup(curl);

    // Error handling
    if (res != CURLE_OK)
      throw perception_exception("cURL error: " + std::string(curl_easy_strerror(res)));

    if (http_code != 200)
      throw perception_exception("HTTP error: " + std::to_string(http_code));

    // Parse and return response
    try
    {
      nlohmann::json json = nlohmann::json::parse(response_data);
      return fromJson(json);  // Assumes fromJson(const nlohmann::json&) is implemented
    }
    catch (const std::exception& e)
    {
      throw perception_exception("JSON parse error: " + std::string(e.what()));
    }
  }

  virtual perception::RESTResponse call_tts(const perception::RESTRequest& req)
  {
    CURL* curl = curl_easy_init();
    if (!curl)
      throw perception_exception("Failed to initialize libcurl");

    // Prepare the JSON body
    nlohmann::json json_body;
    for (const auto& opt : req.options)
    {
      json_body[opt.key] = opt.value;
    }
    json_body["input"] = req.prompt;  // Set input text explicitly

    std::string body = json_body.dump();
    std::vector<uint8_t> audio_binary;

    // Set headers
    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    if (auth_type_ == "Bearer")
    {
      std::string auth_header = "Authorization: Bearer " + api_key_;
      headers = curl_slist_append(headers, auth_header.c_str());
    }
    else
    {
      curl_easy_cleanup(curl);
      curl_slist_free_all(headers);
      throw perception_exception("Unsupported auth type: " + auth_type_);
    }

    curl_easy_setopt(curl, CURLOPT_URL, uri_.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, body.size());

    // Write binary audio data to vector
    curl_easy_setopt(
        curl, CURLOPT_WRITEFUNCTION, +[](void* ptr, size_t size, size_t nmemb, void* userdata) -> size_t {
          auto* vec = reinterpret_cast<std::vector<uint8_t>*>(userdata);
          size_t total_size = size * nmemb;
          vec->insert(vec->end(), (uint8_t*)ptr, (uint8_t*)ptr + total_size);
          return total_size;
        });
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &audio_binary);

    if (!ssl_verify_)
    {
      curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0L);
      curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0L);
    }

    // Perform request
    CURLcode res = curl_easy_perform(curl);
    long http_code = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);

    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);

    if (res != CURLE_OK)
      throw perception_exception("cURL error: " + std::string(curl_easy_strerror(res)));

    if (http_code != 200)
      throw perception_exception("HTTP error: " + std::to_string(http_code));

    // Convert raw PCM to int16_t
    std::vector<int16_t> samples(audio_binary.size() / 2);
    std::memcpy(samples.data(), audio_binary.data(), audio_binary.size());

    perception::RESTResponse response;
    response.audio_stream = samples;  // Assuming audio_stream is a vector<int16_t>

    return response;
  }

protected:
  static size_t write_callback(void* contents, size_t size, size_t nmemb, std::string* userp)
  {
    userp->append(static_cast<char*>(contents), size * nmemb);
    return size * nmemb;
  }

  /**
   * @brief Convert a prompt request to a JSON object
   *
   * This method converts the perception request to a JSON object that can be sent to the perception plugin.
   * It includes the prompt text and options for the JSON object.
   *
   * @param prompt The perception request to convert
   * @return A JSON object representing the prompt request
   */
  virtual nlohmann::json toJson(const perception::RESTRequest& request) = 0;

  /**
   * @brief Convert a JSON object to a perception response
   *
   * This method converts a JSON object received from the perception plugin into a perception response.
   * It extracts the relevant fields from the JSON object and returns a perception response object.
   *
   * @param object The JSON object to convert
   * @return A perception response object containing the response data
   */
  virtual perception::RESTResponse fromJson(const nlohmann::json& object) = 0;

  std::string plugin_name_;  // Name of the plugin, used for parameter names
  std::string uri_;          // URI for the REST API
  std::string method_;       // HTTP method (GET, POST, etc.)
  bool ssl_verify_;          // Flag for SSL verification
  std::string auth_type_;    // Type of authentication (e.g., Bearer)
  std::string api_key_;      // API key for authentication
};

}  // namespace perception
