#pragma once

#include <perception_base/audio/structs.hpp>
#include <perception_base/audio/wav.hpp>
#include <perception_base/exceptions.hpp>
#include <perception_base/rest_base.hpp>
#include <perception_base/speech/speech_synthesis_driver.hpp>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cctype>
#include <future>
#include <sstream>
#include <string>
#include <vector>

namespace perception
{
class OpenAISpeechDriver : public RestBase, public SpeechSynthesisDriver
{
public:
  /**
   * @brief Construct a new OpenAI Speech Driver object
   */
  OpenAISpeechDriver()
  {
  }

  /**
   * @brief Destroy the OpenAI Speech Driver object
   */
  ~OpenAISpeechDriver() override
  {
    deinitialize();
  }
  /**
   * @brief Initialize the driver
   *
   * This function should be overridden in derived classes to provide specific initialization.
   *
   * @param node Shared pointer to the ROS node
   */
  void initialize(const rclcpp::Node::SharedPtr& node) override
  {
    // Confirm parameters for the node
    node->declare_parameter("driver.speech.OpenAISpeechDriver.name", "OpenAISpeechDriver");
    node->declare_parameter("driver.speech.OpenAISpeechDriver.model", "gpt-4o-mini-tts");
    node->declare_parameter("driver.speech.OpenAISpeechDriver.test_text", "Hello this is a test speech for ROS2 "
                                                                          "perception speech driver.");
    node->declare_parameter("driver.speech.OpenAISpeechDriver.test_file_path", "test/speech.wav");
    node->declare_parameter("driver.speech.OpenAISpeechDriver.voice", "coral");
    node->declare_parameter("driver.speech.OpenAISpeechDriver.instructions", "Please speak clearly and slowly.");
    node->declare_parameter("driver.speech.OpenAISpeechDriver.chunking_enabled", true);
    node->declare_parameter("driver.speech.OpenAISpeechDriver.chunk_max_words", 100);
    node->declare_parameter("driver.speech.OpenAISpeechDriver.chunk_parallel_requests", 3);

    // Get parameters from the node
    name_ = node->get_parameter("driver.speech.OpenAISpeechDriver.name").as_string();
    model_name_ = node->get_parameter("driver.speech.OpenAISpeechDriver.model").as_string();
    test_text_ = node->get_parameter("driver.speech.OpenAISpeechDriver.test_text").as_string();
    test_file_path_ = node->get_parameter("driver.speech.OpenAISpeechDriver.test_file_path").as_string();
    voice_ = node->get_parameter("driver.speech.OpenAISpeechDriver.voice").as_string();
    instructions_ = node->get_parameter("driver.speech.OpenAISpeechDriver.instructions").as_string();
    chunking_enabled_ = node->get_parameter("driver.speech.OpenAISpeechDriver.chunking_enabled").as_bool();
    chunk_max_words_ =
      std::max<size_t>(1,
               static_cast<size_t>(node->get_parameter("driver.speech.OpenAISpeechDriver.chunk_max_words")
                           .as_int()));
    chunk_parallel_requests_ =
      std::max<size_t>(1,
               static_cast<size_t>(node->get_parameter("driver.speech.OpenAISpeechDriver.chunk_parallel_requests")
                           .as_int()));

    // Initialize the REST base class
    initialize_rest_base(node, "driver.speech.OpenAISpeechDriver", "OPENAI_API_KEY");

    // log the parameters
    RCLCPP_INFO(node_->get_logger(), "Assigned driver name: %s", name_.c_str());
    RCLCPP_INFO(node_->get_logger(), "Assigned driver model: %s", model_name_.c_str());
    RCLCPP_INFO(node_->get_logger(), "Assigned driver test text: %s", test_text_.c_str());
    RCLCPP_INFO(node_->get_logger(), "Assigned driver test file path: %s", test_file_path_.c_str());
    RCLCPP_INFO(node_->get_logger(), "Assigned driver voice: %s", voice_.c_str());
    RCLCPP_INFO(node_->get_logger(), "Assigned driver instructions: %s", instructions_.c_str());
    RCLCPP_INFO(node_->get_logger(), "Assigned driver chunking enabled: %s", chunking_enabled_ ? "true" : "false");
    RCLCPP_INFO(node_->get_logger(), "Assigned driver chunk max words: %zu", chunk_max_words_);
    RCLCPP_INFO(node_->get_logger(), "Assigned driver chunk parallel requests: %zu", chunk_parallel_requests_);

    if (diagnostics_enabled())
    {
      enable_diagnostics("rest-speech-" + model_name_, name_ + " status",
                         [this](diagnostic_updater::DiagnosticStatusWrapper& status) {
                           produce_diagnostics(status);
                         });
    }

    // Log the driver initialization
    RCLCPP_INFO(node_->get_logger(), "Initialized");
  }

  /**
   * @brief Deinitialize the driver
   *
   * Required by DriverBase. This driver holds no long-running resources beyond
   * cached state and the ROS node pointer.
   */
  void deinitialize() override
  {
    disable_diagnostics();
    response_ = perception::RESTResponse{};
    model_name_.clear();
    test_text_.clear();
    voice_.clear();
    instructions_.clear();
    test_file_path_.clear();
    name_.clear();
    chunking_enabled_ = true;
    chunk_max_words_ = 200;
    chunk_parallel_requests_ = 3;
    node_.reset();
  }

  audio_data synthesize(const text_data& new_text) override
  {
    audio_data data;

    if (new_text.text.empty())
    {
      last_audio_ = data;
      return last_audio_;
    }

    const auto voice = resolve_voice(new_text);
    const auto instructions = resolve_instructions(new_text);
    const auto chunks = split_text_into_chunks(new_text.text);

    RCLCPP_INFO(node_->get_logger(), "Synthesizing speech using %zu chunk(s).", chunks.size());

    std::vector<int16_t> combined_samples;
    combined_samples.reserve(new_text.text.size() * 24);

    if (chunks.size() == 1)
    {
      const auto response = synthesize_chunk(chunks.front(), voice, instructions);
      combined_samples = response.audio_stream;
      response_ = response;
    }
    else
    {
      for (size_t begin = 0; begin < chunks.size(); begin += chunk_parallel_requests_)
      {
        const size_t end = std::min(chunks.size(), begin + chunk_parallel_requests_);
        std::vector<std::future<perception::RESTResponse>> futures;
        futures.reserve(end - begin);

        for (size_t index = begin; index < end; ++index)
        {
          futures.push_back(std::async(std::launch::async, [this, &chunks, &voice, &instructions, index]() {
            return synthesize_chunk(chunks[index], voice, instructions);
          }));
        }

        for (size_t index = 0; index < futures.size(); ++index)
        {
          auto response = futures[index].get();
          combined_samples.insert(combined_samples.end(), response.audio_stream.begin(), response.audio_stream.end());
          if (begin + index + 1 == chunks.size())
          {
            response_ = std::move(response);
          }
        }
      }
    }

    if (!combined_samples.empty())
    {
      data.samples = std::move(combined_samples);
      data.sample_rate = 24000;
      data.channels = 1;
      data.chunk_count = static_cast<int>((data.samples.size() + static_cast<size_t>(data.chunk_size) - 1) /
                                          static_cast<size_t>(data.chunk_size));
    }

    last_audio_ = data;
    return last_audio_;
  }
  /**
   * @brief Test method for the driver
   *
   * This function can be overridden in derived classes to implement specific test logic.
   */
  void test() override
  {
    // Implement test logic if needed
    RCLCPP_INFO(node_->get_logger(), "Testing with model: %s", model_name_.c_str());
    RCLCPP_INFO(node_->get_logger(), "Testing by speeching : %s", test_text_.c_str());

    perception::text_data new_text;
    new_text.text = test_text_;
    new_text.voice = voice_;
    new_text.instructions = instructions_;

    // Convert the audio data to the expected format
    const auto data = synthesize(new_text);

    RCLCPP_INFO(node_->get_logger(), "Speech service called with test text data. waiting for response...");

    if (!data.samples.empty())
    {
      writeWavFile(test_file_path_, data);
      RCLCPP_INFO(node_->get_logger(), "Speech synthesis result received and saved to %s", test_file_path_.c_str());
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "No speech synthesis result received.");
      throw perception_exception("No speech synthesis result received.");
    }

    RCLCPP_INFO(node_->get_logger(), "Test completed.");
  }

protected:
  std::string resolve_voice(const text_data& new_text) const
  {
    if (new_text.voice.empty())
    {
      RCLCPP_ERROR(node_->get_logger(), "Voice is empty, using default voice.");
      return voice_;
    }

    RCLCPP_INFO(node_->get_logger(), "Using voice: %s", new_text.voice.c_str());
    return new_text.voice;
  }

  std::string resolve_instructions(const text_data& new_text) const
  {
    if (new_text.instructions.empty())
    {
      RCLCPP_ERROR(node_->get_logger(), "Instructions are empty, using default instructions.");
      return instructions_;
    }

    RCLCPP_INFO(node_->get_logger(), "Using instructions: %s", new_text.instructions.c_str());
    return new_text.instructions;
  }

  perception::RESTResponse synthesize_chunk(const std::string& chunk,
                                            const std::string& voice,
                                            const std::string& instructions)
  {
    perception::RESTRequest request;
    request.prompt = chunk;
    request.options.clear();

    perception::RESTOption model_option1;
    model_option1.key = "model";
    model_option1.value = model_name_;
    model_option1.type = perception::RESTOptionType::STRING;
    request.options.push_back(model_option1);

    perception::RESTOption model_option2;
    model_option2.key = "response_format";
    model_option2.value = "pcm";
    model_option2.type = perception::RESTOptionType::STRING;
    request.options.push_back(model_option2);

    perception::RESTOption model_option3;
    model_option3.key = "voice";
    model_option3.value = voice;
    model_option3.type = perception::RESTOptionType::STRING;
    request.options.push_back(model_option3);

    perception::RESTOption model_option4;
    model_option4.key = "instructions";
    model_option4.value = instructions;
    model_option4.type = perception::RESTOptionType::STRING;
    request.options.push_back(model_option4);

    return call_tts(request);
  }

  static std::string trim_copy(const std::string& text)
  {
    const auto begin = text.find_first_not_of(" \t\n\r");
    if (begin == std::string::npos)
      return "";

    const auto end = text.find_last_not_of(" \t\n\r");
    return text.substr(begin, end - begin + 1);
  }

  static size_t count_words(const std::string& text)
  {
    std::istringstream stream(text);
    size_t count = 0;
    std::string word;

    while (stream >> word)
    {
      ++count;
    }

    return count;
  }

  static std::vector<std::string> split_long_segment(const std::string& segment, size_t max_words)
  {
    std::vector<std::string> chunks;
    std::istringstream stream(segment);
    std::string word;
    std::string current;
    size_t current_words = 0;

    while (stream >> word)
    {
      if (!current.empty())
      {
        current.push_back(' ');
      }

      current += word;
      ++current_words;

      if (current_words >= max_words)
      {
        chunks.push_back(current);
        current.clear();
        current_words = 0;
      }
    }

    if (!current.empty())
    {
      chunks.push_back(current);
    }

    return chunks;
  }

  std::vector<std::string> split_text_into_chunks(const std::string& text) const
  {
    const std::string trimmed_text = trim_copy(text);
    if (trimmed_text.empty())
    {
      return {};
    }

    if (!chunking_enabled_ || count_words(trimmed_text) <= chunk_max_words_)
    {
      return {trimmed_text};
    }

    std::vector<std::string> chunks;
    std::string current;
    std::string sentence;

    auto flush_sentence = [&]() {
      std::string trimmed_sentence = trim_copy(sentence);
      sentence.clear();

      if (trimmed_sentence.empty())
      {
        return;
      }

      const size_t sentence_word_count = count_words(trimmed_sentence);
      if (sentence_word_count > chunk_max_words_)
      {
        if (!current.empty())
        {
          chunks.push_back(trim_copy(current));
          current.clear();
        }

        const auto split_segments = split_long_segment(trimmed_sentence, chunk_max_words_);
        chunks.insert(chunks.end(), split_segments.begin(), split_segments.end());
        return;
      }

      const std::string candidate = current.empty() ? trimmed_sentence : current + " " + trimmed_sentence;
      if (count_words(candidate) <= chunk_max_words_)
      {
        current = candidate;
      }
      else
      {
        chunks.push_back(trim_copy(current));
        current = trimmed_sentence;
      }
    };

    for (char character : trimmed_text)
    {
      sentence.push_back(character);
      if (character == '.' || character == '!' || character == '?' || character == '\n')
      {
        flush_sentence();
      }
    }

    flush_sentence();

    if (!current.empty())
    {
      chunks.push_back(trim_copy(current));
    }

    chunks.erase(std::remove_if(chunks.begin(), chunks.end(), [](const std::string& chunk) { return chunk.empty(); }),
                 chunks.end());
    return chunks;
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
  virtual nlohmann::json toJson(const perception::RESTRequest& request)
  {
    nlohmann::json result;

    // Add options
    for (const auto& option : request.options)
    {
      result[option.key] = option.value;
    }

    // Add prompt
    result["input"] = request.prompt;

    return result;
  }

  /**
   * @brief Convert a JSON object to a perception response
   *
   * This method converts a JSON object received from the perception plugin into a perception response.
   * It extracts the relevant fields from the JSON object and returns a perception response object.
   *
   * @param object The JSON object to convert
   * @return A perception response object containing the response data
   */
  virtual perception::RESTResponse fromJson(const nlohmann::json& object)
  {
    (void)object;
    throw perception_exception("fromJson() not implemented for this driver.");
  }

  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    std::string last_error;
    {
      std::lock_guard<std::mutex> lock(rest_status_mutex_);
      last_error = last_rest_error_;
    }

    if (rest_request_count_.load() == 0)
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Speech driver idle");
    else if (last_rest_success_.load())
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Last speech request succeeded");
    else
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Last speech request failed");

    status.add("model", model_name_);
    status.add("uri", uri_);
    status.add("request_count", rest_request_count_.load());
    status.add("failure_count", rest_failure_count_.load());
    status.add("last_http_code", last_rest_http_code_.load());
    status.add("last_audio_samples", last_audio_.samples.size());
    status.add("last_audio_sample_rate", last_audio_.sample_rate);
    status.add("default_voice", voice_);
    status.add("last_error", last_error.empty() ? std::string("none") : last_error);
  }

  std::string model_name_;
  std::string test_text_;
  std::string voice_;
  std::string instructions_;
  std::string test_file_path_ = "test_audio.wav";  // Path to the test audio file
  bool chunking_enabled_ = true;
  size_t chunk_max_words_ = 200;
  size_t chunk_parallel_requests_ = 3;

  perception::RESTResponse response_;
  audio_data last_audio_;
};
}  // namespace perception