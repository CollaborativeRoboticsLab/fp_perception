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
#include <cv_bridge/cv_bridge.h>

#include <perception_base/driver_base.hpp>
#include <perception_base/audio/structs.hpp>

#include <perception_msgs/msg/perception_audio.hpp>
#include <perception_msgs/msg/perception_text.hpp>
#include <perception_msgs/srv/perception_speech.hpp>
#include <perception_msgs/srv/perception_transcribe.hpp>
#include <perception_msgs/srv/perception_sentiment.hpp>
#include <perception_msgs/srv/perception_image_analysis.hpp>

namespace perception
{

class PerceptionServer : public rclcpp::Node
{
public:
  using Audio = perception_msgs::msg::PerceptionAudio;
  using Speech = perception_msgs::srv::PerceptionSpeech;
  using Transcribe = perception_msgs::srv::PerceptionTranscribe;
  using Sentiment = perception_msgs::srv::PerceptionSentiment;
  using ImageAnalysis = perception_msgs::srv::PerceptionImageAnalysis;

  /**
   * @brief Construct a new Perception Server object
   *
   * @param options Node options for the server
   */
  PerceptionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("perception_server", options), driver_loader_("perception", "perception::DriverBase")
  {
    try
    {
      if (shared_from_this())
      {
        initialize();
      }
    }
    catch (const std::bad_weak_ptr&)
    {
      // Not yet safe — probably standalone without make_shared
    }
  }

  ~PerceptionServer()
  {
  }

  void initialize()
  {
    RCLCPP_INFO(this->get_logger(), "Initializing PerceptionServer...");

    /*************************************************************************
     * Identify what plugins to load
     ************************************************************************/
    // drivers
    this->declare_parameter("use_ros_vision_driver", false);
    this->declare_parameter("use_non_ros_vision_driver", false);
    this->declare_parameter("use_microphone_driver", false);
    this->declare_parameter("use_speaker_driver", false);
    this->declare_parameter("use_transcription_driver", false);
    this->declare_parameter("use_sentiment_driver", false);
    this->declare_parameter("use_speech_driver", false);
    this->declare_parameter("use_image_analysis_driver", false);

    // mischellaneous
    this->declare_parameter("run_tests", false);

    // drivers
    use_ros_vision_driver_ = this->get_parameter("use_ros_vision_driver").as_bool();
    use_non_ros_vision_driver_ = this->get_parameter("use_non_ros_vision_driver").as_bool();
    use_microphone_driver_ = this->get_parameter("use_microphone_driver").as_bool();
    use_speaker_driver_ = this->get_parameter("use_speaker_driver").as_bool();
    use_transcription_driver_ = this->get_parameter("use_transcription_driver").as_bool();
    use_sentiment_driver_ = this->get_parameter("use_sentiment_driver").as_bool();
    use_speech_driver_ = this->get_parameter("use_speech_driver").as_bool();
    use_image_analysis_driver_ = this->get_parameter("use_image_analysis_driver").as_bool();

    // ROS Interface
    this->declare_parameter("interface.audio_input.topic", "perception/microphone");
    this->declare_parameter("interface.audio_input.frame_id", "microphone_frame");
    this->declare_parameter("interface.audio_input.publish", false);
    this->declare_parameter("interface.audio_input.frequency", 10);
    this->declare_parameter("interface.audio_input.buffer_duration", 10);

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
    audio_input_buffer_duration_ = this->get_parameter("interface.audio_input.buffer_duration").as_int();

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

    RCLCPP_INFO(this->get_logger(), "Audio output subscribe: %s", audio_output_subscribe_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Audio output topic: %s", audio_output_topic_.c_str());

    RCLCPP_INFO(this->get_logger(), "Vision input publish: %s", vision_input_publish_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Vision input topic: %s", vision_input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Vision input frame_id: %s", vision_input_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "Vision input frequency: %d", vision_input_frequency_);

    RCLCPP_INFO(this->get_logger(), "Transcription enabled: %s", transcription_enabled_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Transcription service: %s", transcription_service_.c_str());
    RCLCPP_INFO(this->get_logger(), "Transcription buffer duration: %d seconds", audio_input_buffer_duration_);

    RCLCPP_INFO(this->get_logger(), "Speech synthesis enabled: %s", speech_enabled_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Speech synthesis service name: %s", speech_service_name_.c_str());

    RCLCPP_INFO(this->get_logger(), "Sentiment analysis enabled: %s", sentiment_enabled_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Sentiment analysis service name: %s", sentiment_service_name_.c_str());

    RCLCPP_INFO(this->get_logger(), "Image analysis enabled: %s", image_analysis_enabled_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Image analysis service name: %s", image_analysis_service_name_.c_str());

    // run tests
    run_tests_ = this->get_parameter("run_tests").as_bool();

    /*************************************************************************
     * vision driver plugin class loader and driver pointer
     ************************************************************************/

    if (use_ros_vision_driver_)
    {
      this->declare_parameter("ros_vision_driver", "perception::DefaultDriver");
      std::string ros_vision_driver_name = this->get_parameter("ros_vision_driver").as_string();

      RCLCPP_INFO(this->get_logger(), "Loading vision driver plugin: %s", ros_vision_driver_name.c_str());

      ros_vision_driver_ = driver_loader_.createSharedInstance(ros_vision_driver_name);
      ros_vision_driver_->initialize(shared_from_this());

      RCLCPP_INFO(this->get_logger(), "Started vision driver plugin: %s", ros_vision_driver_name.c_str());
    }

    if (use_non_ros_vision_driver_)
    {
      this->declare_parameter("non_ros_vision_driver", "perception::OpenCVDriver");
      std::string non_ros_vision_driver_name = this->get_parameter("non_ros_vision_driver").as_string();

      RCLCPP_INFO(this->get_logger(), "Loading vision driver plugin: %s", non_ros_vision_driver_name.c_str());

      non_ros_vision_driver_ = driver_loader_.createSharedInstance(non_ros_vision_driver_name);
      non_ros_vision_driver_->initialize(shared_from_this());

      RCLCPP_INFO(this->get_logger(), "Started vision driver plugin: %s", non_ros_vision_driver_name.c_str());
    }

    /*************************************************************************
     * microphone driver plugin class loader and driver pointer
     ************************************************************************/

    if (use_microphone_driver_)
    {
      this->declare_parameter("microphone_driver", "perception::MicrophoneAudioDriver");
      std::string mic_driver_name = this->get_parameter("microphone_driver").as_string();

      RCLCPP_INFO(this->get_logger(), "Loading microphone driver plugin: %s", mic_driver_name.c_str());

      microphone_driver_ = driver_loader_.createSharedInstance(mic_driver_name);
      microphone_driver_->initialize(shared_from_this());

      RCLCPP_INFO(this->get_logger(), "Started microphone driver plugin: %s", mic_driver_name.c_str());
    }

    /*************************************************************************
     * speaker driver plugin class loader and driver pointer
     ************************************************************************/

    if (use_speaker_driver_)
    {
      this->declare_parameter("speaker_driver", "perception::SpeakerAudioDriver");
      std::string speaker_driver_name = this->get_parameter("speaker_driver").as_string();

      RCLCPP_INFO(this->get_logger(), "Loading speaker driver plugin: %s", speaker_driver_name.c_str());

      speaker_driver_ = driver_loader_.createSharedInstance(speaker_driver_name);
      speaker_driver_->initialize(shared_from_this());

      RCLCPP_INFO(this->get_logger(), "Started speaker driver plugin: %s", speaker_driver_name.c_str());
    }

    /*************************************************************************
     * transcription driver plugin class loader and driver pointer
     ************************************************************************/
    if (use_transcription_driver_)
    {
      this->declare_parameter("transcription_driver", "perception::OpenAIDriver");
      std::string transcription_driver_name = this->get_parameter("transcription_driver").as_string();

      RCLCPP_INFO(this->get_logger(), "Loading transcription driver plugin: %s", transcription_driver_name.c_str());

      transcription_driver_ = driver_loader_.createSharedInstance(transcription_driver_name);
      transcription_driver_->initialize(shared_from_this());

      RCLCPP_INFO(this->get_logger(), "Started transcription driver plugin: %s", transcription_driver_name.c_str());
    }

    /*************************************************************************
     * speech synthesis driver plugin class loader and driver pointer
     ************************************************************************/
    if (use_speech_driver_)
    {
      this->declare_parameter("speech_synthesis_driver", "perception::OpenAISpeechDriver");
      std::string speech_driver_name = this->get_parameter("speech_synthesis_driver").as_string();

      RCLCPP_INFO(this->get_logger(), "Loading speech synthesis driver plugin: %s", speech_driver_name.c_str());

      speech_driver_ = driver_loader_.createSharedInstance(speech_driver_name);
      speech_driver_->initialize(shared_from_this());

      RCLCPP_INFO(this->get_logger(), "Started speech synthesis driver plugin: %s", speech_driver_name.c_str());
    }

    /*************************************************************************
     * sentiment driver plugin class loader and driver pointer
     ************************************************************************/
    if (use_sentiment_driver_)
    {
      this->declare_parameter("sentiment_driver", "perception::SentimentDriver");
      std::string sentiment_driver_name = this->get_parameter("sentiment_driver").as_string();

      RCLCPP_INFO(this->get_logger(), "Loading sentiment driver plugin: %s", sentiment_driver_name.c_str());

      sentiment_driver_ = driver_loader_.createSharedInstance(sentiment_driver_name);
      sentiment_driver_->initialize(shared_from_this());

      RCLCPP_INFO(this->get_logger(), "Started sentiment driver plugin: %s", sentiment_driver_name.c_str());
    }

    /*************************************************************************
     * image analysis driver plugin class loader and driver pointer
     ************************************************************************/

    if (use_image_analysis_driver_)
    {
      this->declare_parameter("image_analysis_driver", "perception::OpenAIImageAnalysisDriver");
      std::string image_analysis_driver_name = this->get_parameter("image_analysis_driver").as_string();

      RCLCPP_INFO(this->get_logger(), "Loading image analysis driver plugin: %s", image_analysis_driver_name.c_str());

      image_analysis_driver_ = driver_loader_.createSharedInstance(image_analysis_driver_name);
      image_analysis_driver_->initialize(shared_from_this());

      RCLCPP_INFO(this->get_logger(), "Started image analysis driver plugin: %s", image_analysis_driver_name.c_str());
    }

    /*************************************************************************
     * Run tests if requested
     ************************************************************************/
    if (run_tests_)
    {
      RCLCPP_INFO(this->get_logger(), "Running tests...");

      run_tests();

      RCLCPP_INFO(this->get_logger(), "Tests completed.");
    }

    // Start audio producer thread whenever the microphone is enabled.
    // The publishAudio() loop also feeds the public ring buffer used by services.
    if (use_microphone_driver_)
      publish_audio_ = std::thread(&PerceptionServer::publishAudio, this);

    if (vision_input_publish_)
      publish_vision_ = std::thread(&PerceptionServer::publishVideo, this);

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
  }

protected:
  void run_tests()
  {
    RCLCPP_INFO(this->get_logger(), "Running tests for loaded plugins...");

    if (use_ros_vision_driver_)
    {
      RCLCPP_INFO(this->get_logger(), "Testing ros vision driver...");
      ros_vision_driver_->test();
    }

    if (use_non_ros_vision_driver_)
    {
      RCLCPP_INFO(this->get_logger(), "Testing non-ros vision driver...");
      non_ros_vision_driver_->test();
    }

    if (use_microphone_driver_)
    {
      RCLCPP_INFO(this->get_logger(), "Testing microphone driver...");
      microphone_driver_->test();
    }

    if (use_speaker_driver_)
    {
      RCLCPP_INFO(this->get_logger(), "Testing speaker driver...");
      speaker_driver_->test();
    }

    if (use_transcription_driver_)
    {
      RCLCPP_INFO(this->get_logger(), "Testing transcription driver...");
      transcription_driver_->test();
    }

    if (use_speech_driver_)
    {
      RCLCPP_INFO(this->get_logger(), "Testing speech synthesis driver...");
      speech_driver_->test();
    }

    if (use_sentiment_driver_)
    {
      RCLCPP_INFO(this->get_logger(), "Testing sentiment driver...");
      sentiment_driver_->test();
    }

    if (use_image_analysis_driver_)
    {
      RCLCPP_INFO(this->get_logger(), "Testing image analysis driver...");
      image_analysis_driver_->test();
    }
  }

  /**
   * @brief Driver thread function. need to be overridden in derived classesif used
   */
  virtual void publishAudio()
  {
    while (rclcpp::ok())
    {
      audio_data data;

      if (use_microphone_driver_ and microphone_driver_)
        data = std::any_cast<audio_data>(microphone_driver_->getDataStream());

      // Producer: append to public buffer (protected by mutex/cv)
      if (use_microphone_driver_ && microphone_driver_ && !data.samples.empty())
      {
        std::unique_lock<std::mutex> lock(public_buffer_mutex_);

        // If format changes, reset buffer and counters.
        if (public_buffer_.sample_rate != data.sample_rate || public_buffer_.channels != data.channels)
        {
          public_buffer_.samples.clear();
          public_buffer_total_samples_ = 0;
        }

        public_buffer_.sample_rate = data.sample_rate;
        public_buffer_.channels = data.channels;
        public_buffer_.chunk_size = data.chunk_size;
        public_buffer_.chunk_count = data.chunk_count;

        public_buffer_.samples.insert(public_buffer_.samples.end(), data.samples.begin(), data.samples.end());
        public_buffer_total_samples_ += static_cast<uint64_t>(data.samples.size());

        const size_t max_buffer_size = static_cast<size_t>(std::max(1, data.sample_rate)) *
                                       static_cast<size_t>(std::max(1, data.channels)) *
                                       static_cast<size_t>(std::max(1, audio_input_buffer_duration_));

        if (public_buffer_.samples.size() > max_buffer_size)
        {
          const auto excess = public_buffer_.samples.size() - max_buffer_size;
          public_buffer_.samples.erase(public_buffer_.samples.begin(), public_buffer_.samples.begin() + excess);
        }

        // Update chunk count based on current buffer size
        const size_t denom =
            static_cast<size_t>(std::max(1, data.chunk_size)) * static_cast<size_t>(std::max(1, data.channels));
        public_buffer_.chunk_count = denom ? (public_buffer_.samples.size() / denom) : 0;

        lock.unlock();
        public_buffer_cv_.notify_all();
      }

      if (audio_input_publish_ && audio_publisher_)
      {
        // If publishing is enabled, publish the audio data
        Audio msg;
        msg.header.stamp = rclcpp::Clock().now();
        msg.header.frame_id = audio_input_frame_id_;
        msg.sample_rate = data.sample_rate;
        msg.channels = data.channels;
        msg.chunk_size = data.chunk_size;
        msg.chunk_count = data.chunk_count;
        msg.samples = data.samples;

        audio_publisher_->publish(msg);
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(int(1000 / audio_input_frequency_)));
    }
  }

  void publishVideo()
  {
    while (rclcpp::ok())
    {
      if (use_ros_vision_driver_)
      {
        try
        {
          // Convert sensor_msgs::Image to OpenCV Mat
          auto image = std::any_cast<sensor_msgs::msg::Image::ConstSharedPtr>(ros_vision_driver_->getData());
          auto cv_ptr = cv_bridge::toCvCopy(image, image->encoding);
          cv::Mat frame = cv_ptr->image.clone();

          auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
          msg->header = image->header;
        }
        catch (const cv_bridge::Exception& e)
        {
          throw perception_exception("cv_bridge conversion failed: " + std::string(e.what()));
        }
      }

      if (use_non_ros_vision_driver_)
      {
        // Capture frame from the camera
        cv::Mat frame = std::any_cast<cv::Mat>(non_ros_vision_driver_->getData());

        // If publishing is enabled, publish the image
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = this->now();
        msg->header.frame_id = vision_input_frame_id_;
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
   * @throws perception_exception if the stream is not active
   */
  void audioCallback(const Audio& msg)
  {
    auto data = perception::msg_to_audio_data(msg);
    speaker_driver_->setDataStream(data);
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

    perception::audio_data audio_in;

    if (request->use_device_audio)
    {
      const int duration = std::max(1, request->device_buffer_time);

      try
      {
        audio_in = wait_for_public_audio(duration);
      }
      catch (const std::exception& e)
      {
        response->success = false;
        response->transcription = std::string("Device audio not available: ") + e.what();
        RCLCPP_ERROR(this->get_logger(), "%s", response->transcription.c_str());
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Transcription with device audio: %zu samples", audio_in.samples.size());
    }
    else
    {
      audio_in = perception::msg_to_audio_data(request->audio);
      response->header = request->audio.header;
      RCLCPP_INFO(this->get_logger(), "Transcription with external audio: %zu samples", audio_in.samples.size());
    }

    {
      std::lock_guard<std::mutex> lock(transcription_driver_mutex_);
      transcription_driver_->setDataStream(audio_in);
    }

    std::any result;
    {
      std::lock_guard<std::mutex> lock(transcription_driver_mutex_);
      result = transcription_driver_->getData();
    }

    if (result.has_value())
    {
      response->transcription = std::any_cast<std::string>(result);
      response->success = true;
      RCLCPP_INFO(this->get_logger(), "Transcription service processed request successfully.");
    }
    else
    {
      response->success = false;
      response->transcription = "No transcription result received.";
      RCLCPP_ERROR(this->get_logger(), "Transcription service failed to process request.");
    }
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

    const auto& text_data = perception::msg_to_text_data(request->input);

    speech_driver_->setDataStream(text_data);
    auto result = speech_driver_->getData();

    if (result.has_value())
    {
      if (request->use_device_audio)
      {
        RCLCPP_INFO(this->get_logger(), "Using device audio for speech output.");
        speaker_driver_->setDataStream(result);
        response->success = true;
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Using external audio for speech output.");
        response->audio = perception::audio_data_to_msg(std::any_cast<audio_data>(result));
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

    std::string text;

    if (request->use_device_audio)
    {
      RCLCPP_INFO(this->get_logger(), "Using device audio for sentiment analysis.");

      if (!transcription_driver_)
      {
        RCLCPP_ERROR(this->get_logger(), "Device audio sentiment requested but transcription driver is not loaded.");
        response->label = "Error: transcription driver not loaded";
        response->score = 0.0;
        return;
      }

      const int duration = std::max(1, request->device_buffer_time);
      perception::audio_data audio_in;
      try
      {
        audio_in = wait_for_public_audio(duration);
      }
      catch (const std::exception& e)
      {
        response->label = std::string("Error: device audio not available: ") + e.what();
        response->score = 0.0;
        RCLCPP_ERROR(this->get_logger(), "%s", response->label.c_str());
        return;
      }

      std::any transcription_result;
      {
        std::lock_guard<std::mutex> lock(transcription_driver_mutex_);
        transcription_driver_->setDataStream(audio_in);
        transcription_result = transcription_driver_->getData();
      }

      if (transcription_result.has_value())
      {
        try
        {
          text = std::any_cast<std::string>(transcription_result);
        }
        catch (const std::bad_any_cast&)
        {
          RCLCPP_ERROR(this->get_logger(), "Unexpected transcription result type for sentiment analysis.");
          response->label = "Error: unexpected transcription result type";
          response->score = 0.0;
          return;
        }

        RCLCPP_INFO(this->get_logger(), "Transcription result for sentiment analysis: %s", text.c_str());
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "No transcription result available for sentiment analysis.");
        response->label = "Error: No transcription result available for sentiment analysis.";
        response->score = 0.0;
        return;
      }
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Received sentiment analysis request with text: %s", request->text.c_str());
      text = request->text;
    }

    RCLCPP_INFO(this->get_logger(), "Using text for sentiment analysis.");
    {
      std::lock_guard<std::mutex> lock(sentiment_driver_mutex_);
      sentiment_driver_->setDataStream(text);
    }

    std::any result;
    {
      std::lock_guard<std::mutex> lock(sentiment_driver_mutex_);
      result = sentiment_driver_->getData();
    }

    if (result.has_value())
    {
      auto sentiment_result = std::any_cast<std::pair<std::string, double>>(result);

      // Set the response data
      response->label = sentiment_result.first;   // Example response
      response->score = sentiment_result.second;  // Example confidence score
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "No sentiment analysis result available");
      response->label = "Error: No sentiment analysis result available";
      response->score = 0.0;
    }
  }

  perception::audio_data wait_for_public_audio(int duration_seconds)
  {
    duration_seconds = std::max(1, duration_seconds);

    std::unique_lock<std::mutex> lock(public_buffer_mutex_);

    // Wait until we have at least one chunk and can determine sample format.
    if (public_buffer_.sample_rate <= 0 || public_buffer_.channels <= 0)
    {
      public_buffer_cv_.wait_for(lock, std::chrono::seconds(5), [this] {
        return public_buffer_.sample_rate > 0 && public_buffer_.channels > 0 && !public_buffer_.samples.empty();
      });
    }

    if (public_buffer_.sample_rate <= 0 || public_buffer_.channels <= 0)
      throw perception_exception("public audio buffer not initialized");

    const size_t max_samples = static_cast<size_t>(std::max(1, public_buffer_.sample_rate)) *
                               static_cast<size_t>(std::max(1, public_buffer_.channels)) *
                               static_cast<size_t>(std::max(1, audio_input_buffer_duration_));

    const size_t needed_samples = static_cast<size_t>(public_buffer_.sample_rate) *
                                  static_cast<size_t>(public_buffer_.channels) * static_cast<size_t>(duration_seconds);

    if (needed_samples > max_samples)
      throw perception_exception("requested device_buffer_time exceeds configured "
                                 "interface.audio_input.buffer_duration");

    const uint64_t start = public_buffer_total_samples_;
    const uint64_t needed_total = start + static_cast<uint64_t>(needed_samples);

    const auto timeout = std::chrono::seconds(duration_seconds + 10);
    const bool ok = public_buffer_cv_.wait_for(
        lock, timeout, [this, needed_total] { return public_buffer_total_samples_ >= needed_total; });

    if (!ok)
      throw perception_exception("timeout waiting for device audio buffer to fill");

    if (public_buffer_.samples.size() < needed_samples)
      throw perception_exception("buffer underflow for requested duration");

    perception::audio_data out = public_buffer_;
    out.samples.assign(public_buffer_.samples.end() - static_cast<std::ptrdiff_t>(needed_samples),
                       public_buffer_.samples.end());
    out.chunk_count = 1;
    out.chunk_size = static_cast<int>(needed_samples / static_cast<size_t>(std::max(1, out.channels)));
    return out;
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

    const std::string prompt = request->prompt.empty() ? std::string("What's in this image?") : request->prompt;

    cv::Mat frame;
    try
    {
      if (request->use_device_vision)
      {
        if (!use_ros_vision_driver_ && !use_non_ros_vision_driver_)
          throw perception_exception("use_device_vision requested but no vision driver is loaded");

        if (use_non_ros_vision_driver_)
        {
          frame = std::any_cast<cv::Mat>(non_ros_vision_driver_->getData());
        }
        
        if (use_ros_vision_driver_)
        {
          auto image = std::any_cast<sensor_msgs::msg::Image::ConstSharedPtr>(ros_vision_driver_->getData());
          auto cv_ptr = cv_bridge::toCvCopy(image, image->encoding);
          frame = cv_ptr->image;
        }
      }
      else
      {
        const auto& image_msg = request->image;
        auto cv_ptr = cv_bridge::toCvCopy(image_msg, image_msg.encoding);
        frame = cv_ptr->image;
      }
    }
    catch (const std::exception& e)
    {
      response->response = std::string("Failed to acquire image: ") + e.what();
      RCLCPP_ERROR(this->get_logger(), "%s", response->response.c_str());
      return;
    }

    try
    {
      image_analysis_driver_->setDataStream(std::make_pair(frame, prompt));
      auto result = image_analysis_driver_->getData();

      if (result.has_value())
      {
        response->response = std::any_cast<std::string>(result);
        RCLCPP_INFO(this->get_logger(), "Image analysis service processed request successfully.");
      }
      else
      {
        response->response = "No image analysis result received.";
        RCLCPP_ERROR(this->get_logger(), "Image analysis service failed to process request.");
      }
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

  // Buffer to store audio data for other drivers when using device audio
  audio_data public_buffer_;
  std::mutex public_buffer_mutex_;
  std::condition_variable public_buffer_cv_;
  uint64_t public_buffer_total_samples_{ 0 };

  std::mutex transcription_driver_mutex_;
  std::mutex sentiment_driver_mutex_;

  // Audio inputtopic parameters
  bool audio_input_publish_;
  std::string audio_input_topic_;
  std::string audio_input_frame_id_;
  int audio_input_frequency_;
  int audio_input_buffer_duration_;

  // Audio output topic parameters
  std::string audio_output_topic_;
  bool audio_output_subscribe_;

  // Vision input topic parameters
  bool vision_input_publish_;
  std::string vision_input_topic_;
  std::string vision_input_frame_id_;
  int vision_input_frequency_;
  bool vision_input_non_ros_;

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
  pluginlib::ClassLoader<perception::DriverBase> driver_loader_;

  /** shared pointer for vision driver */
  std::shared_ptr<perception::DriverBase> ros_vision_driver_;

  /** shared pointer for vision driver */
  std::shared_ptr<perception::DriverBase> non_ros_vision_driver_;

  /** shared pointer for audio listener driver */
  std::shared_ptr<perception::DriverBase> microphone_driver_;

  /** shared pointer for audio speaker driver */
  std::shared_ptr<perception::DriverBase> speaker_driver_;

  /** shared pointer for transcription driver */
  std::shared_ptr<perception::DriverBase> transcription_driver_;

  /** shared pointer for sentiment driver */
  std::shared_ptr<perception::DriverBase> sentiment_driver_;

  /** shared pointer for speech synthesis driver */
  std::shared_ptr<perception::DriverBase> speech_driver_;

  /** shared pointer for image analysis driver */
  std::shared_ptr<perception::DriverBase> image_analysis_driver_;

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

  // Thread for publishing gathered data from the device
  std::thread publish_audio_;

  // Thread for publishing gathered data from the device
  std::thread publish_vision_;
};

}  // namespace perception