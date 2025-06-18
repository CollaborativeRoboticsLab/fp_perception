#pragma once

#include <string>

// include poco json and net/netssl
#include <Poco/JSON/Object.h>
#include <Poco/JSON/Parser.h>
#include <Poco/Net/Context.h>
#include <Poco/Net/HTMLForm.h>
#include <Poco/Net/HTTPRequest.h>
#include <Poco/Net/HTTPResponse.h>
#include <Poco/Net/HTTPSClientSession.h>
#include <Poco/Net/NetException.h>
#include <Poco/Net/StringPartSource.h>
#include <Poco/Net/StreamSocket.h>
#include <Poco/URI.h>

#include <rclcpp/rclcpp.hpp>
#include <perception_events/event_client.hpp>
#include <perception_base/driver_base.hpp>
#include <perception_base/utils/exceptions.hpp>
#include <perception_base/utils/rest/structs.hpp>
#include <perception_base/utils/rest/file_part_source.hpp>

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
    event_->info("Assigned driver URI: " + uri_);
    event_->info("Assigned driver Method: " + method_);
    event_->info("Assigned driver SSL Verify: " + std::string(ssl_verify_ ? "true" : "false"));
    event_->info("Assigned driver Auth Type: " + auth_type_);

    // Load api key from environment
    if (!api_key_name.empty())
    {
      const char* api_key_env = std::getenv(api_key_name.c_str());
      if (api_key_env)
      {
        api_key_ = api_key_env;
        event_->info("API key loaded from environment variables: " + api_key_name);
      }
      else
      {
        event_->error("missing env variable: " + api_key_name);
        throw perception::perception_exception("missing env variable: " + api_key_name);
        api_key_ = "";
      }
    }
    else
    {
      event_->info("An API key is not used for this plugin, using empty string");
      api_key_ = "";
    }
  }

  /**
   * @brief sendPrompt send a prompt to a prompt provider using REST
   *
   * Typical providers offer stream based responses which is also supported
   *
   * @param req The prompt request containing the prompt and options.
   * @return const PromptResponse
   */
  virtual perception::RESTResponse request(const perception::RESTRequest& req)
  {
    // uri
    Poco::URI uri(uri_);

    // prepare request body
    Poco::JSON::Object body_json = toJson(req);

    // calculate body length
    std::ostringstream body_stream;
    body_json.stringify(body_stream);

    // create request object
    Poco::Net::HTTPRequest request(method_, uri.getPath());
    // set headers
    request.setContentType("application/json");
    request.setContentLength(body_stream.str().size());

    // if bearer token
    if (auth_type_ == "Bearer")
    {
      request.setCredentials(auth_type_, api_key_);
    }

    // Todo: support other auth types
    // else if (auth_type_ == "Token")
    // {
    //   request.setCredentials(auth_type_, api_key_);
    // }
    else
    {
      event_->error("unsupported auth type: " + auth_type_);
    }

    // RCLCPP_INFO(node_->get_logger(), "Port %d", uri.getPort());

    std::unique_ptr<Poco::Net::HTTPClientSession> session_ptr;

    // is the session secure?
    if (uri.getScheme() == "https")
    {
      // context without certificate verification
      Poco::Net::Context::Params params;

      if (!ssl_verify_)
      {
        params.verificationMode = Poco::Net::Context::VERIFY_NONE;
      }
      else
      {
        params.verificationMode = Poco::Net::Context::VERIFY_STRICT;
        params.caLocation = "/etc/ssl/certs";  // Update this path if needed
      }

      Poco::Net::Context::Ptr context = new Poco::Net::Context(Poco::Net::Context::CLIENT_USE, params);

      // create secure session
      // Poco::Net::HTTPSClientSession session(uri.getHost(), uri.getPort(), context);
      session_ptr = std::make_unique<Poco::Net::HTTPSClientSession>(uri.getHost(), uri.getPort(), context);
      event_->info("secure session created");
    }
    else
    {
      // create insecure session
      // Poco::Net::HTTPClientSession session(uri.getHost(), uri.getPort());
      session_ptr = std::make_unique<Poco::Net::HTTPClientSession>(uri.getHost(), uri.getPort());
      event_->info("insecure session created");
    }

    try
    {
      // send request
      std::ostream& os = session_ptr->sendRequest(request);
      // complete request body
      body_json.stringify(os);
    }
    catch (const Poco::Net::NetException& e)
    {
      event_->error("network error: " + std::string(e.what()));
      throw perception::perception_exception("network error: " + std::string(e.what()));
    }

    // get response
    Poco::Net::HTTPResponse response;
    std::istream& rs = session_ptr->receiveResponse(response);

    // check for errors
    if (response.getStatus() != Poco::Net::HTTPResponse::HTTP_OK)
    {
      event_->error("HTTP Error: " + std::to_string(response.getStatus()) + " " + response.getReason());
      throw perception::perception_exception("HTTP Error: " + std::to_string(response.getStatus()) + " " +
                                             response.getReason());
    }

    // check content type is 'text/event-stream' or 'application/x-ndjson'
    // https://developer.mozilla.org/en-US/docs/Web/API/Server-sent_events
    // these types are used for server-sent events or newline delimited JSON
    if (response.getContentType() == "text/event-stream" || response.getContentType() == "application/x-ndjson")
    {
      // TODO: handle event stream
      // pass to stream parsing
      // SinglePromptProvider::handle_event_stream(rs, chunck_cb);
      event_->error("HTTP streaming not supported");
      throw perception::perception_exception("HTTP stream not supported");
    }

    // is the response chunked even though it is not server-sent event?
    if (response.getChunkedTransferEncoding())
    {
      // TODO: handle chunked responses
      event_->error("HTTP Chunked Transfer Encoding not supported");
      throw perception::perception_exception("HTTP Chunked Transfer Encoding not supported");
    }

    // parse response
    Poco::JSON::Parser parser;
    Poco::Dynamic::Var result = parser.parse(rs);
    Poco::JSON::Object::Ptr object = result.extract<Poco::JSON::Object::Ptr>();

    // create prompt provider response container
    perception::RESTResponse res = fromJson(object);

    return res;
  }

  /**
   * @brief Convert a REST request that contains audio data to a RESTResponse
   *  This function handles audio data requests, typically for transcription services.
   * It supports both JSON and multipart form data requests.
   *
   * @param req The REST request to convert
   * @return const Poco::JSON::Object
   */
  virtual perception::RESTResponse request_audio(const perception::RESTRequest& req)
  {
    Poco::URI uri(uri_);

    // Create HTTP(S) session
    std::unique_ptr<Poco::Net::HTTPClientSession> session_ptr;

    if (uri.getScheme() == "https")
    {
      Poco::Net::Context::Params params;
      params.verificationMode = ssl_verify_ ? Poco::Net::Context::VERIFY_STRICT : Poco::Net::Context::VERIFY_NONE;
      params.caLocation = "/etc/ssl/certs";

      Poco::Net::Context::Ptr context = new Poco::Net::Context(Poco::Net::Context::CLIENT_USE, params);
      session_ptr = std::make_unique<Poco::Net::HTTPSClientSession>(uri.getHost(), uri.getPort(), context);
      event_->info("Secure session created");
    }
    else
    {
      session_ptr = std::make_unique<Poco::Net::HTTPClientSession>(uri.getHost(), uri.getPort());
      event_->info("Insecure session created");
    }

    // Prepare request
    Poco::Net::HTTPRequest request(method_, uri.getPathAndQuery(), Poco::Net::HTTPMessage::HTTP_1_0);

    if (auth_type_ == "Bearer")
      request.set("Authorization", "Bearer " + api_key_);
    else
      event_->error("Unsupported auth type: " + auth_type_);

    try
    {
      // Use multipart form for audio buffer
      Poco::Net::HTMLForm form;
      form.setEncoding(Poco::Net::HTMLForm::ENCODING_MULTIPART);

      for (const auto& opt : req.options)
      {
        form.add(opt.key, opt.value);
        event_->info("Form option: " + opt.key + " = " + opt.value);
      }

      form.addPart("file", new FilePartSource(req.file_type, req.file_stream));
      form.prepareSubmit(request);

      request.setContentType("multipart/form-data; boundary=" + form.boundary());

      std::ostream& os = session_ptr->sendRequest(request);
      form.write(os);
    }
    catch (const Poco::Exception& e)
    {
      event_->error("Request error: " + std::string(e.what()));
      throw perception::perception_exception("Request error: " + std::string(e.what()));
    }

    // Receive response
    Poco::Net::HTTPResponse response;
    std::istream& rs = session_ptr->receiveResponse(response);

    // Log the echoed result
    std::ostringstream oss;
    Poco::StreamCopier::copyStream(rs, oss);
    event_->info("Echoed response:\n" + oss.str());

    if (response.getStatus() != Poco::Net::HTTPResponse::HTTP_OK)
    {
      // Log the error
      event_->error("HTTP Error: " + std::to_string(response.getStatus()) + " " + response.getReason());
      throw perception::perception_exception("HTTP Error: " + std::to_string(response.getStatus()) + " " +
                                             response.getReason());
    }

    if (response.getContentType() == "text/event-stream" || response.getContentType() == "application/x-ndjson")
    {
      event_->error("HTTP streaming not supported");
      throw perception::perception_exception("HTTP stream not supported");
    }

    if (response.getChunkedTransferEncoding())
    {
      event_->error("HTTP Chunked Transfer Encoding not supported");
      throw perception::perception_exception("HTTP Chunked Transfer Encoding not supported");
    }

    Poco::JSON::Parser parser;
    Poco::Dynamic::Var result = parser.parse(rs);
    Poco::JSON::Object::Ptr object = result.extract<Poco::JSON::Object::Ptr>();

    return fromJson(object);
  }

  /**
   * @brief Process options from the prompt request
   *
   * This method processes the options from the prompt request and returns a JSON object
   * containing the options. It flattens the options into a JSON object, converting types
   * based on the type hint provided in the option.
   *
   * @param req The prompt request containing options to be processed
   */
  virtual const Poco::JSON::Object handle_options(const perception::RESTRequest& req)
  {
    // flatten options into object
    Poco::JSON::Object result;

    for (const perception::RESTOption& option : req.options)
    {
      // try cast the value if there is a type hint
      if (option.type == perception::RESTOptionType::STRING)
      {
        result.set(option.key, option.value);
        continue;
      }

      // try cast the value if there is a type hint
      if (option.type == perception::RESTOptionType::BOOL)
      {
        result.set(option.key, (option.value == "true") ? true : false);
        continue;
      }

      // try cast the value if there is a type hint
      if (option.type == perception::RESTOptionType::INT)
      {
        result.set(option.key, std::stoi(option.value));
        continue;
      }

      // try cast the value if there is a type hint
      if (option.type == perception::RESTOptionType::DOUBLE)
      {
        result.set(option.key, std::stod(option.value));
        continue;
      }

      // just set the value if there is no type hint
      result.set(option.key, option.value);
    }

    return result;
  }

protected:
  /**
   * @brief Convert a prompt request to a JSON object
   *
   * This method converts the perception request to a JSON object that can be sent to the perception plugin.
   * It includes the prompt text and options for the JSON object.
   *
   * @param prompt The perception request to convert
   * @return A JSON object representing the prompt request
   */
  virtual Poco::JSON::Object toJson(const perception::RESTRequest& prompt) = 0;

  /**
   * @brief Convert a JSON object to a perception response
   *
   * This method converts a JSON object received from the perception plugin into a perception response.
   * It extracts the relevant fields from the JSON object and returns a perception response object.
   *
   * @param object The JSON object to convert
   * @return A perception response object containing the response data
   */
  virtual perception::RESTResponse fromJson(const Poco::JSON::Object::Ptr object) = 0;

  std::string plugin_name_;  // Name of the plugin, used for parameter names
  std::string uri_;          // URI for the REST API
  std::string method_;       // HTTP method (GET, POST, etc.)
  bool ssl_verify_;          // Flag for SSL verification
  std::string auth_type_;    // Type of authentication (e.g., Bearer)
  std::string api_key_;      // API key for authentication
};

}  // namespace perception
