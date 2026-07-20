#pragma once

#include <thread>
#include <functional>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <fp_perception/audio_buffer.hpp>
#include <fp_perception/driver_manager.hpp>
#include <fp_perception/image_analysis_pipeline.hpp>
#include <fp_perception/sentiment_pipeline.hpp>
#include <fp_perception/speech_pipeline.hpp>
#include <fp_perception/transcription_pipeline.hpp>
#include <fp_perception_base/audio/audio_sink_driver.hpp>
#include <fp_perception_base/audio/audio_source_driver.hpp>
#include <fp_perception_base/driver_base.hpp>
#include <fp_perception_base/transcription/transcription_driver.hpp>
#include <fp_perception_base/speech/speech_synthesis_driver.hpp>
#include <fp_perception_base/sentiment/sentiment_analysis_driver.hpp>
#include <fp_perception_base/vision/vision_source_driver.hpp>
#include <fp_perception_base/audio/structs.hpp>
#include <fp_perception_base/transcription/structs.hpp>
#include <fp_perception_base/sentiment/structs.hpp>
#include <fp_perception_base/vision/structs.hpp>
#include <fp_perception_base/image_analysis/image_analysis_driver.hpp>
#include <fp_perception_base/image_analysis/structs.hpp>

#include <fp_perception_msgs/msg/perception_audio.hpp>
#include <fp_perception_msgs/msg/perception_text.hpp>
#include <fp_perception_msgs/srv/perception_speech.hpp>
#include <fp_perception_msgs/srv/perception_transcribe.hpp>
#include <fp_perception_msgs/srv/perception_sentiment.hpp>
#include <fp_perception_msgs/srv/perception_image_analysis.hpp>

namespace fp_perception
{

class PerceptionServer : public rclcpp::Node
{
public:
  struct DriverSpec
  {
    const char* parameter_name;
    const char* default_plugin_name;
    const char* driver_description;
    const char* interface_description;
  };

  using Audio = fp_perception_msgs::msg::PerceptionAudio;
  using Speech = fp_perception_msgs::srv::PerceptionSpeech;
  using Transcribe = fp_perception_msgs::srv::PerceptionTranscribe;
  using Sentiment = fp_perception_msgs::srv::PerceptionSentiment;
  using ImageAnalysis = fp_perception_msgs::srv::PerceptionImageAnalysis;

  inline static constexpr DriverSpec kRosVisionDriverSpec{ "ros_vision_driver", "fp_perception::DefaultDriver",
                                                           "vision driver", "VisionSourceDriver" };
  inline static constexpr DriverSpec kNonRosVisionDriverSpec{ "non_ros_vision_driver", "fp_perception::OpenCVDriver",
                                                              "vision driver", "VisionSourceDriver" };
  inline static constexpr DriverSpec kMicrophoneDriverSpec{ "microphone_driver", "fp_perception::MicrophoneAudioDriver",
                                                            "microphone driver", "AudioSourceDriver" };
  inline static constexpr DriverSpec kSpeakerDriverSpec{ "speaker_driver", "fp_perception::SpeakerAudioDriver",
                                                         "speaker driver", "AudioSinkDriver" };
  inline static constexpr DriverSpec kTranscriptionDriverSpec{ "transcription_driver", "fp_perception::OpenAIDriver",
                                                               "transcription driver", "TranscriptionDriver" };
  inline static constexpr DriverSpec kSpeechDriverSpec{ "speech_synthesis_driver", "fp_perception::OpenAISpeechDriver",
                                                        "speech synthesis driver", "SpeechSynthesisDriver" };
  inline static constexpr DriverSpec kSentimentDriverSpec{ "sentiment_driver", "fp_perception::SentimentDriver",
                                                           "sentiment driver", "SentimentAnalysisDriver" };
  inline static constexpr DriverSpec kImageAnalysisDriverSpec{ "image_analysis_driver",
                                                               "fp_perception::OpenAIImageAnalysisDriver",
                                                               "image analysis driver", "ImageAnalysisDriver" };

  /**
   * @brief Construct a new Perception Server object
   *
   * @param options Node options for the server
   */
  PerceptionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("perception_server", options), driver_loader_("fp_perception", "fp_perception::DriverBase")
  {
    schedule_deferred_initialization();
  }

  ~PerceptionServer()
  {
    stop_background_tasks();
  }

  void initialize()
  {
    std::scoped_lock lock(initialization_mutex_);
    if (initialized_)
      return;

    RCLCPP_INFO(this->get_logger(), "Initializing PerceptionServer...");
    DriverManager driver_manager(shared_from_this(), driver_loader_);

    // ROS Interface
    configure_driver_selection();
    configure_interfaces();
    load_drivers(driver_manager);
    initialize_pipelines();

    if (run_tests_)
      run_tests();

    if (audio_input_publish_)
      audio_publisher_ = this->create_publisher<Audio>(audio_input_topic_, 10);

    if (audio_output_subscribe_)
      audio_subscriber_ = this->create_subscription<Audio>(
          audio_output_topic_, 10, std::bind(&PerceptionServer::audioCallback, this, std::placeholders::_1));

    if (transcription_enabled_)
      transcription_ = this->create_service<Transcribe>(
          transcription_service_,
          std::bind(&PerceptionServer::transcribe_callback, this, std::placeholders::_1, std::placeholders::_2));

    if (speech_enabled_)
      speech_service_ =
          this->create_service<Speech>(speech_service_name_, std::bind(&PerceptionServer::speech_callback, this,
                                                                       std::placeholders::_1, std::placeholders::_2));

    if (sentiment_enabled_)
      sentiment_service_ = this->create_service<Sentiment>(
          sentiment_service_name_,
          std::bind(&PerceptionServer::sentiment_callback, this, std::placeholders::_1, std::placeholders::_2));

    if (vision_input_publish_)
      image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(vision_input_topic_, 10);

    if (image_analysis_enabled_)
      image_analysis_service_ = this->create_service<ImageAnalysis>(
          image_analysis_service_name_,
          std::bind(&PerceptionServer::image_analysis_callback, this, std::placeholders::_1, std::placeholders::_2));

    start_background_tasks();

    initialized_ = true;
  }

protected:
  void schedule_deferred_initialization()
  {
    deferred_initialize_timer_ = this->create_wall_timer(std::chrono::milliseconds(1), [this]() {
      deferred_initialize_timer_->cancel();
      deferred_initialize_timer_.reset();

      try
      {
        initialize();
      }
      catch (const std::exception& e)
      {
        RCLCPP_ERROR(this->get_logger(), "Deferred initialization failed: %s", e.what());
      }
    });
  }

  void start_background_tasks()
  {
    if (use_microphone_driver_ && microphone_driver_ && !audio_publish_thread_.joinable())
    {
      audio_publish_thread_ = std::thread([this]() { publishAudio(); });
    }
  }

  void stop_background_tasks()
  {
    if (microphone_driver_)
      microphone_driver_->deinitialize();

    if (audio_publish_thread_.joinable())
      audio_publish_thread_.join();
  }

  void configure_driver_selection()
  {
    this->declare_parameter("use_ros_vision_driver", false);
    this->declare_parameter("use_non_ros_vision_driver", false);
    this->declare_parameter("use_microphone_driver", false);
    this->declare_parameter("use_speaker_driver", false);
    this->declare_parameter("use_transcription_driver", false);
    this->declare_parameter("use_sentiment_driver", false);
    this->declare_parameter("use_speech_driver", false);
    this->declare_parameter("use_image_analysis_driver", false);
    this->declare_parameter("use_diagnostics", false);
    this->declare_parameter("run_tests", false);

    use_ros_vision_driver_ = this->get_parameter("use_ros_vision_driver").as_bool();
    use_non_ros_vision_driver_ = this->get_parameter("use_non_ros_vision_driver").as_bool();
    use_microphone_driver_ = this->get_parameter("use_microphone_driver").as_bool();
    use_speaker_driver_ = this->get_parameter("use_speaker_driver").as_bool();
    use_transcription_driver_ = this->get_parameter("use_transcription_driver").as_bool();
    use_sentiment_driver_ = this->get_parameter("use_sentiment_driver").as_bool();
    use_speech_driver_ = this->get_parameter("use_speech_driver").as_bool();
    use_image_analysis_driver_ = this->get_parameter("use_image_analysis_driver").as_bool();
    run_tests_ = this->get_parameter("run_tests").as_bool();
  }

  void configure_interfaces()
  {
    this->declare_parameter("interface.audio_input.topic", "perception/microphone");
    this->declare_parameter("interface.audio_input.frame_id", "microphone_frame");
    this->declare_parameter("interface.audio_input.publish", false);
    this->declare_parameter("interface.audio_input.frequency", 10);
    this->declare_parameter("interface.audio_input.audio_retention_window", 10);
    this->declare_parameter("interface.audio_input.default_audio_request_window", 10);

    this->declare_parameter("interface.audio_output.topic", "perception/speaker");
    this->declare_parameter("interface.audio_output.subscribe", false);

    this->declare_parameter("interface.vision_input.topic", "perception/camera");
    this->declare_parameter("interface.vision_input.frame_id", "camera_frame");
    this->declare_parameter("interface.vision_input.publish", false);
    this->declare_parameter("interface.vision_input.frequency", 10);

    this->declare_parameter("interface.transcription.service", "perception/transcription");
    this->declare_parameter("interface.transcription.provide_service", false);

    this->declare_parameter("interface.speech.service_name", "perception/speech");
    this->declare_parameter("interface.speech.provide_service", false);

    this->declare_parameter("interface.sentiment.service_name", "perception/sentiment_analysis");
    this->declare_parameter("interface.sentiment.provide_service", false);

    this->declare_parameter("interface.image_analysis.service_name", "perception/image_analysis");
    this->declare_parameter("interface.image_analysis.provide_service", false);

    audio_input_publish_ = this->get_parameter("interface.audio_input.publish").as_bool();
    audio_input_topic_ = this->get_parameter("interface.audio_input.topic").as_string();
    audio_input_frame_id_ = this->get_parameter("interface.audio_input.frame_id").as_string();
    audio_input_frequency_ = this->get_parameter("interface.audio_input.frequency").as_int();
    audio_input_retention_window_ = this->get_parameter("interface.audio_input.audio_retention_window").as_int();
    default_audio_request_window_ = this->get_parameter("interface.audio_input.default_audio_request_window").as_int();

    audio_output_subscribe_ = this->get_parameter("interface.audio_output.subscribe").as_bool();
    audio_output_topic_ = this->get_parameter("interface.audio_output.topic").as_string();

    vision_input_publish_ = this->get_parameter("interface.vision_input.publish").as_bool();
    vision_input_topic_ = this->get_parameter("interface.vision_input.topic").as_string();
    vision_input_frame_id_ = this->get_parameter("interface.vision_input.frame_id").as_string();
    vision_input_frequency_ = this->get_parameter("interface.vision_input.frequency").as_int();

    transcription_service_ = this->get_parameter("interface.transcription.service").as_string();
    transcription_enabled_ = this->get_parameter("interface.transcription.provide_service").as_bool();

    speech_service_name_ = this->get_parameter("interface.speech.service_name").as_string();
    speech_enabled_ = this->get_parameter("interface.speech.provide_service").as_bool();

    sentiment_service_name_ = this->get_parameter("interface.sentiment.service_name").as_string();
    sentiment_enabled_ = this->get_parameter("interface.sentiment.provide_service").as_bool();

    image_analysis_service_name_ = this->get_parameter("interface.image_analysis.service_name").as_string();
    image_analysis_enabled_ = this->get_parameter("interface.image_analysis.provide_service").as_bool();

    RCLCPP_INFO(this->get_logger(), "Audio input publish: %s", audio_input_publish_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Audio input topic: %s", audio_input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Audio input frame_id: %s", audio_input_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "Audio input frequency: %d", audio_input_frequency_);
    RCLCPP_INFO(this->get_logger(), "Audio retention window: %d seconds", audio_input_retention_window_);
    RCLCPP_INFO(this->get_logger(), "Default audio request window: %d seconds", default_audio_request_window_);

    RCLCPP_INFO(this->get_logger(), "Audio output subscribe: %s", audio_output_subscribe_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Audio output topic: %s", audio_output_topic_.c_str());

    RCLCPP_INFO(this->get_logger(), "Vision input publish: %s", vision_input_publish_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Vision input topic: %s", vision_input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Vision input frame_id: %s", vision_input_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "Vision input frequency: %d", vision_input_frequency_);

    RCLCPP_INFO(this->get_logger(), "Transcription enabled: %s", transcription_enabled_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Transcription service: %s", transcription_service_.c_str());
    RCLCPP_INFO(this->get_logger(), "Speech synthesis enabled: %s", speech_enabled_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Speech synthesis service name: %s", speech_service_name_.c_str());

    RCLCPP_INFO(this->get_logger(), "Sentiment analysis enabled: %s", sentiment_enabled_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Sentiment analysis service name: %s", sentiment_service_name_.c_str());

    RCLCPP_INFO(this->get_logger(), "Image analysis enabled: %s", image_analysis_enabled_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Image analysis service name: %s", image_analysis_service_name_.c_str());
  }

  template <typename DriverType>
  std::shared_ptr<DriverType> load_driver_if_enabled(const bool enabled, const DriverSpec& spec,
                                                     const DriverManager& driver_manager)
  {
    if (!enabled)
      return nullptr;

    return driver_manager.loadDriver<DriverType>(spec.parameter_name, spec.default_plugin_name, spec.driver_description,
                                                 spec.interface_description);
  }

  void load_drivers(const DriverManager& driver_manager)
  {
    ros_vision_driver_ = load_driver_if_enabled<fp_perception::VisionSourceDriver>(
        use_ros_vision_driver_, kRosVisionDriverSpec, driver_manager);
    non_ros_vision_driver_ = load_driver_if_enabled<fp_perception::VisionSourceDriver>(
        use_non_ros_vision_driver_, kNonRosVisionDriverSpec, driver_manager);
    microphone_driver_ = load_driver_if_enabled<fp_perception::AudioSourceDriver>(
        use_microphone_driver_, kMicrophoneDriverSpec, driver_manager);
    speaker_driver_ =
        load_driver_if_enabled<fp_perception::AudioSinkDriver>(use_speaker_driver_, kSpeakerDriverSpec, driver_manager);
    transcription_driver_ = load_driver_if_enabled<fp_perception::TranscriptionDriver>(
        use_transcription_driver_, kTranscriptionDriverSpec, driver_manager);
    speech_driver_ = load_driver_if_enabled<fp_perception::SpeechSynthesisDriver>(use_speech_driver_, kSpeechDriverSpec,
                                                                                  driver_manager);
    sentiment_driver_ = load_driver_if_enabled<fp_perception::SentimentAnalysisDriver>(
        use_sentiment_driver_, kSentimentDriverSpec, driver_manager);
    image_analysis_driver_ = load_driver_if_enabled<fp_perception::ImageAnalysisDriver>(
        use_image_analysis_driver_, kImageAnalysisDriverSpec, driver_manager);
  }

  void initialize_pipelines()
  {
    transcription_pipeline_ = std::make_unique<TranscriptionPipeline>(
        transcription_driver_, [this](const audio_buffer_request& request) { return read_public_audio(request); });

    speech_pipeline_ = std::make_unique<SpeechPipeline>(speech_driver_, speaker_driver_);

    sentiment_pipeline_ = std::make_unique<SentimentPipeline>(
        sentiment_driver_, transcription_driver_,
        [this](const audio_buffer_request& request) { return read_public_audio(request); });

    image_analysis_pipeline_ =
        std::make_unique<ImageAnalysisPipeline>(image_analysis_driver_, ros_vision_driver_, non_ros_vision_driver_);
  }

  void run_tests()
  {
    RCLCPP_INFO(this->get_logger(), "Running tests for loaded plugins...");

    DriverManager::testDriver(this->get_logger(), ros_vision_driver_, "ros vision driver");
    DriverManager::testDriver(this->get_logger(), non_ros_vision_driver_, "non-ros vision driver");
    DriverManager::testDriver(this->get_logger(), microphone_driver_, "microphone driver");
    DriverManager::testDriver(this->get_logger(), speaker_driver_, "speaker driver");
    DriverManager::testDriver(this->get_logger(), transcription_driver_, "transcription driver");
    DriverManager::testDriver(this->get_logger(), speech_driver_, "speech synthesis driver");
    DriverManager::testDriver(this->get_logger(), sentiment_driver_, "sentiment driver");
    DriverManager::testDriver(this->get_logger(), image_analysis_driver_, "image analysis driver");
  }

  virtual void publishAudio()
  {
    while (rclcpp::ok())
    {
      try
      {
        audio_data data;

        if (use_microphone_driver_ && microphone_driver_)
          data = microphone_driver_->readChunk();

        const auto audio_stamp = this->now();

        if (use_microphone_driver_ && microphone_driver_ && !data.samples.empty())
          public_audio_buffer_.append(data, audio_input_retention_window_, audio_stamp);

        if (audio_input_publish_ && audio_publisher_)
        {
          // If publishing is enabled, publish the audio data
          Audio msg;
          msg.header.stamp = audio_stamp;
          msg.header.frame_id = audio_input_frame_id_;
          msg.sample_rate = data.sample_rate;
          msg.channels = data.channels;
          msg.chunk_size = data.chunk_size;
          msg.chunk_count = data.chunk_count;
          msg.samples = data.samples;

          audio_publisher_->publish(msg);
        }
      }
      catch (const std::exception& e)
      {
        if (rclcpp::ok())
          RCLCPP_WARN(this->get_logger(), "Audio publish loop skipped a cycle: %s", e.what());
      }

      if (!(use_microphone_driver_ && microphone_driver_))
        std::this_thread::sleep_for(std::chrono::milliseconds(int(1000 / std::max(1, audio_input_frequency_))));
    }
  }

  void publishVideo()
  {
    while (rclcpp::ok())
    {
      if (use_ros_vision_driver_)
      {
        auto frame = ros_vision_driver_->captureFrame();
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame.image).toImageMsg();
        msg->header.stamp = frame.stamp;
        msg->header.frame_id = frame.frame_id;
        image_publisher_->publish(*msg);
      }

      if (use_non_ros_vision_driver_)
      {
        auto frame = non_ros_vision_driver_->captureFrame();

        // If publishing is enabled, publish the image
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame.image).toImageMsg();
        msg->header.stamp = frame.stamp;
        msg->header.frame_id = frame.frame_id;
        image_publisher_->publish(*msg);
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(int(1000 / vision_input_frequency_)));
    }
  }

  /**
   * @brief Set the latest audio data to the driver from topic subscription.
   * This function is called when new audio data is received from the subscribed topic.
   *
   * @param msg The audio data message received from the topic.
   * @throws fp_perception_exception if the stream is not active
   */
  void audioCallback(const Audio& msg)
  {
    auto data = fp_perception::msg_to_audio_data(msg);
    speaker_driver_->play(data);
  }

  transcription_request make_transcription_request(const Transcribe::Request& request, std_msgs::msg::Header& header)
  {
    transcription_request internal_request;
    internal_request.audio_request_window =
        request.audio_request_window > 0 ? request.audio_request_window : default_audio_request_window_;
    internal_request.use_device_audio = request.use_device_audio;
    internal_request.use_device_audio_time_window = false;

    if (!request.use_device_audio)
    {
      internal_request.audio = fp_perception::msg_to_audio_data(request.audio);
      header = request.audio.header;
    }
    else if (request.audio.header.stamp.sec != 0 || request.audio.header.stamp.nanosec != 0)
    {
      internal_request.use_device_audio_time_window = true;
      internal_request.device_audio_start_time = rclcpp::Time(request.audio.header.stamp);
      header = request.audio.header;
    }

    return internal_request;
  }

  void write_transcription_response(const transcription_result& result, Transcribe::Response& response)
  {
    if (result.success)
    {
      response.transcription = result.text;
      response.success = true;
      return;
    }

    response.success = false;
    response.transcription = result.error.empty() ? "No transcription result received." : result.error;
  }

  sentiment_request make_sentiment_request(const Sentiment::Request& request)
  {
    sentiment_request internal_request;
    internal_request.audio_request_window =
        request.audio_request_window > 0 ? request.audio_request_window : default_audio_request_window_;
    internal_request.use_device_audio = request.use_device_audio;
    internal_request.use_device_audio_time_window = false;

    if (!request.use_device_audio)
      internal_request.text = request.text;
    else if (request.header.stamp.sec != 0 || request.header.stamp.nanosec != 0)
    {
      internal_request.use_device_audio_time_window = true;
      internal_request.device_audio_start_time = rclcpp::Time(request.header.stamp);
    }

    return internal_request;
  }

  void write_sentiment_response(const sentiment_result& result, Sentiment::Response& response)
  {
    response.analyzed_text = result.analyzed_text;

    if (result.success)
    {
      response.label = result.label;
      response.score = result.score;
      return;
    }

    response.label = result.error.empty() ? "Error: No sentiment analysis result available" : result.error;
    response.score = 0.0;
  }

  image_analysis_request make_image_analysis_request(const ImageAnalysis::Request& request)
  {
    image_analysis_request internal_request;
    internal_request.use_device_vision = request.use_device_vision;
    internal_request.prompt = request.prompt.empty() ? std::string("What's in this image?") : request.prompt;

    if (!request.use_device_vision)
    {
      const auto& image_msg = request.image;
      auto cv_ptr = cv_bridge::toCvCopy(image_msg, image_msg.encoding);
      internal_request.frame.image = cv_ptr->image;
      internal_request.frame.frame_id = image_msg.header.frame_id;
      internal_request.frame.stamp = image_msg.header.stamp;
    }

    return internal_request;
  }

  void write_image_analysis_response(const image_analysis_result& result, ImageAnalysis::Response& response)
  {
    response.response =
        result.success ? result.response : (result.error.empty() ? "No image analysis result received." : result.error);
  }

  /**
   * @brief Service callback for transcription requests
   *
   * This method handles incoming transcription requests and processes them.
   *
   * @param request The incoming transcription request
   * @param response The response to be sent back
   */
  void transcribe_callback(const std::shared_ptr<Transcribe::Request> request,
                           std::shared_ptr<Transcribe::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Received transcription request.");

    if (!transcription_driver_)
    {
      response->success = false;
      response->transcription = "Transcription driver is not loaded.";
      response->header.stamp = this->now();
      response->header.frame_id = audio_input_frame_id_;
      RCLCPP_ERROR(this->get_logger(), "%s", response->transcription.c_str());
      return;
    }

    response->header.stamp = this->now();
    response->header.frame_id = audio_input_frame_id_;
    const auto transcription_request_data = make_transcription_request(*request, response->header);

    transcription_result transcription_result_data;
    {
      std::lock_guard<std::mutex> lock(transcription_driver_mutex_);
      try
      {
        transcription_result_data = transcription_pipeline_->run(transcription_request_data);
      }
      catch (const std::exception& e)
      {
        response->success = false;
        const std::string error_message = e.what();
        if (error_message.find("public audio buffer") != std::string::npos ||
            error_message.find("audio buffer") != std::string::npos ||
            error_message.find("Timeout waiting for audio data") != std::string::npos)
        {
          response->transcription = std::string("Device audio not available: ") + error_message;
        }
        else
        {
          response->transcription = std::string("Transcription service error: ") + error_message;
        }
        RCLCPP_ERROR(this->get_logger(), "%s", response->transcription.c_str());
        return;
      }
    }

    RCLCPP_INFO(this->get_logger(), "Sending transcription response: success=%s transcription=\"%s\"",
                transcription_result_data.success ? "true" : "false", transcription_result_data.text.c_str());

    write_transcription_response(transcription_result_data, *response);
  }

  /**
   * @brief Service callback for the speech service
   *
   * This function is called when a request is made to the speech service.
   *
   * @param request The request message containing the audio data and options.
   * @param response The response message to be filled with the transcription result.
   */
  void speech_callback(const std::shared_ptr<Speech::Request> request, std::shared_ptr<Speech::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Received speech request with text: %s", request->input.text.c_str());

    const auto& text_data = fp_perception::msg_to_text_data(request->input);
    const auto result = speech_pipeline_->run(text_data, request->use_device_audio);

    if (result.success)
    {
      if (result.used_device_audio)
      {
        RCLCPP_INFO(this->get_logger(), "Using device audio for speech output.");
        response->success = true;
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Using external audio for speech output.");
        response->audio = fp_perception::audio_data_to_msg(result.audio);
        response->success = true;
      }
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "No speech synthesis result received.");
      response->success = false;
    }
  }

  /**
   * @brief Callback function for the sentiment analysis service
   *
   * This function is called when a request is received by the sentiment analysis service.
   * It processes the request and sends back a response with the sentiment analysis results.
   *
   * @param request The request received from the client
   * @param response The response to be sent back to the client
   */
  void sentiment_callback(const std::shared_ptr<Sentiment::Request> request,
                          std::shared_ptr<Sentiment::Response> response)
  {
    if (!sentiment_driver_)
    {
      response->label = "Error: sentiment driver not loaded";
      response->score = 0.0;
      RCLCPP_ERROR(this->get_logger(), "%s", response->label.c_str());
      return;
    }

    const auto sentiment_request_data = make_sentiment_request(*request);

    sentiment_result sentiment_result_data;

    try
    {
      std::scoped_lock lock(transcription_driver_mutex_, sentiment_driver_mutex_);
      sentiment_result_data = sentiment_pipeline_->run(sentiment_request_data);
    }
    catch (const std::exception& e)
    {
      response->label = std::string("Error: ") + e.what();
      response->score = 0.0;
      RCLCPP_ERROR(this->get_logger(), "%s", response->label.c_str());
      return;
    }

    write_sentiment_response(sentiment_result_data, *response);

    RCLCPP_INFO(this->get_logger(), "Sending sentiment response: label='%s' score=%.3f analyzed_text='%s'",
                response->label.c_str(), response->score, response->analyzed_text.c_str());
  }

  fp_perception::audio_data read_latest_public_audio(int duration_seconds)
  {
    duration_seconds = std::max(1, duration_seconds);

    fp_perception::audio_data out = public_audio_buffer_.readLatest(duration_seconds);

    const int sample_rate = std::max(1, out.sample_rate);
    const int channels = std::max(1, out.channels);
    const size_t requested_samples =
        static_cast<size_t>(sample_rate) * static_cast<size_t>(channels) * static_cast<size_t>(duration_seconds);
    const size_t samples_to_copy = out.samples.size();
    const double returned_seconds =
        static_cast<double>(samples_to_copy) / static_cast<double>(std::max(1, sample_rate * channels));
    const auto buffer_end_time = public_audio_buffer_.endTime();
    const auto returned_duration = rclcpp::Duration::from_seconds(returned_seconds);
    const auto window_start_time = buffer_end_time - returned_duration;

    RCLCPP_INFO(this->get_logger(),
                "Using latest device audio window: requested=%d seconds start=%.9f end=%.9f returned=%.3f seconds "
                "sample_rate=%d channels=%d frames=%d",
                duration_seconds, window_start_time.seconds(), buffer_end_time.seconds(), returned_seconds, sample_rate,
                channels, out.chunk_size);

    if (samples_to_copy < requested_samples)
    {
      RCLCPP_WARN(this->get_logger(),
                  "Requested %d seconds of device audio, but only %.2f seconds are buffered. Transcribing latest "
                  "available audio.",
                  duration_seconds, returned_seconds);
    }

    return out;
  }

  fp_perception::audio_data read_public_audio(const audio_buffer_request& request)
  {
    const int duration_seconds = std::max(1, request.duration_seconds);

    if (request.use_time_window)
    {
      const auto requested_end_time = request.start_time + rclcpp::Duration::from_seconds(duration_seconds);

      RCLCPP_INFO(this->get_logger(),
                  "Waiting for timestamped device audio window: requested=[%.9f, %.9f] timeout=%d seconds",
                  request.start_time.seconds(), requested_end_time.seconds(), std::max(5, duration_seconds + 5));

      public_audio_buffer_.waitForWindow(request.start_time, duration_seconds,
                                         std::chrono::seconds(std::max(5, duration_seconds + 5)));

      auto out = public_audio_buffer_.readWindow(request.start_time, duration_seconds);
      const auto buffered_start_time = public_audio_buffer_.startTime();
      const auto buffered_end_time = public_audio_buffer_.endTime();
      const double returned_seconds =
          static_cast<double>(out.chunk_size) / static_cast<double>(std::max(1, out.sample_rate));

      if (out.samples.empty() || out.chunk_size <= 0)
      {
        RCLCPP_WARN(this->get_logger(),
                    "Requested timestamped device audio window has no overlap with the buffered range. Falling back to "
                    "latest %d seconds of buffered audio. requested=[%.9f, %.9f] buffered=[%.9f, %.9f]",
                    duration_seconds, request.start_time.seconds(), requested_end_time.seconds(),
                    buffered_start_time.seconds(), buffered_end_time.seconds());
        return read_latest_public_audio(duration_seconds);
      }

      if (returned_seconds + 1e-6 < static_cast<double>(duration_seconds))
      {
        RCLCPP_WARN(this->get_logger(),
                    "Requested timestamped device audio window is only partially available. Transcribing %.3f seconds "
                    "of overlapping audio. requested=[%.9f, %.9f] buffered=[%.9f, %.9f]",
                    returned_seconds, request.start_time.seconds(), requested_end_time.seconds(),
                    buffered_start_time.seconds(), buffered_end_time.seconds());
      }

      RCLCPP_INFO(this->get_logger(),
                  "Timestamped device audio window ready: requested=[%.9f, %.9f] buffered=[%.9f, %.9f] returned=%.3f "
                  "seconds frames=%d",
                  request.start_time.seconds(), requested_end_time.seconds(), buffered_start_time.seconds(),
                  buffered_end_time.seconds(), returned_seconds, out.chunk_size);

      return out;
    }

    return read_latest_public_audio(duration_seconds);
  }

  void image_analysis_callback(const std::shared_ptr<ImageAnalysis::Request> request,
                               std::shared_ptr<ImageAnalysis::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Received image analysis request.");

    if (!image_analysis_driver_)
    {
      response->response = "Image analysis driver is not loaded.";
      RCLCPP_ERROR(this->get_logger(), "%s", response->response.c_str());
      return;
    }

    image_analysis_request image_request;
    try
    {
      image_request = make_image_analysis_request(*request);
    }
    catch (const std::exception& e)
    {
      response->response = std::string("Failed to acquire image: ") + e.what();
      RCLCPP_ERROR(this->get_logger(), "%s", response->response.c_str());
      return;
    }

    try
    {
      const auto analysis = image_analysis_pipeline_->run(image_request);
      write_image_analysis_response(analysis, *response);
    }
    catch (const std::exception& e)
    {
      response->response = std::string("Image analysis driver error: ") + e.what();
      RCLCPP_ERROR(this->get_logger(), "%s", response->response.c_str());
      return;
    }
  }

  /** Run Tests */
  bool run_tests_;

  bool use_ros_vision_driver_;
  bool use_non_ros_vision_driver_;
  bool use_microphone_driver_;
  bool use_speaker_driver_;
  bool use_transcription_driver_;
  bool use_sentiment_driver_;
  bool use_speech_driver_;
  bool use_image_analysis_driver_;

  AudioBuffer public_audio_buffer_;

  std::mutex transcription_driver_mutex_;
  std::mutex sentiment_driver_mutex_;

  // Audio inputtopic parameters
  bool audio_input_publish_;
  std::string audio_input_topic_;
  std::string audio_input_frame_id_;
  int audio_input_frequency_;
  int audio_input_retention_window_;
  int default_audio_request_window_;

  // Audio output topic parameters
  std::string audio_output_topic_;
  bool audio_output_subscribe_;

  // Vision input topic parameters
  bool vision_input_publish_;
  std::string vision_input_topic_;
  std::string vision_input_frame_id_;
  int vision_input_frequency_;

  // Transcription service parameters
  bool transcription_enabled_;
  std::string transcription_service_;

  // Speech synthesis service parameters
  bool speech_enabled_;
  std::string speech_service_name_;

  // Sentiment analysis service parameters
  bool sentiment_enabled_;
  std::string sentiment_service_name_;

  bool image_analysis_enabled_;
  std::string image_analysis_service_name_;

  /** plugin loader for drivers */
  pluginlib::ClassLoader<fp_perception::DriverBase> driver_loader_;

  /** shared pointer for vision driver */
  std::shared_ptr<fp_perception::VisionSourceDriver> ros_vision_driver_;

  /** shared pointer for vision driver */
  std::shared_ptr<fp_perception::VisionSourceDriver> non_ros_vision_driver_;

  /** shared pointer for audio listener driver */
  std::shared_ptr<fp_perception::AudioSourceDriver> microphone_driver_;

  /** shared pointer for audio speaker driver */
  std::shared_ptr<fp_perception::AudioSinkDriver> speaker_driver_;

  /** shared pointer for transcription driver */
  std::shared_ptr<fp_perception::TranscriptionDriver> transcription_driver_;

  /** shared pointer for sentiment driver */
  std::shared_ptr<fp_perception::SentimentAnalysisDriver> sentiment_driver_;

  /** shared pointer for speech synthesis driver */
  std::shared_ptr<fp_perception::SpeechSynthesisDriver> speech_driver_;

  /** shared pointer for image analysis driver */
  std::shared_ptr<fp_perception::ImageAnalysisDriver> image_analysis_driver_;

  std::unique_ptr<TranscriptionPipeline> transcription_pipeline_;
  std::unique_ptr<SpeechPipeline> speech_pipeline_;
  std::unique_ptr<SentimentPipeline> sentiment_pipeline_;
  std::unique_ptr<ImageAnalysisPipeline> image_analysis_pipeline_;

  // Publisher for audio data
  rclcpp::Publisher<Audio>::SharedPtr audio_publisher_;

  // Subscriber for audio data
  rclcpp::Subscription<Audio>::SharedPtr audio_subscriber_;

  // Service for transcription
  rclcpp::Service<Transcribe>::SharedPtr transcription_;

  // Service for image analysis
  rclcpp::Service<ImageAnalysis>::SharedPtr image_analysis_service_;

  // Service for speech synthesis
  rclcpp::Service<Speech>::SharedPtr speech_service_;

  // Service for sentiment analysis
  rclcpp::Service<Sentiment>::SharedPtr sentiment_service_;

  // Publisher for vision data
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;

  rclcpp::TimerBase::SharedPtr deferred_initialize_timer_;
  std::mutex initialization_mutex_;
  bool initialized_ = false;
  std::thread audio_publish_thread_;
};

}  // namespace fp_perception