#pragma once

#include <thread>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>
#include <perception_base/driver_base.hpp>
#include <perception_base/algorithm_base.hpp>

namespace perception
{

class PerceptionServer : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Perception Server object
   *
   * @param options Node options for the server
   */
  PerceptionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("perception_server", options)
    , vision_driver_loader_("perception", "perception::DriverBase")
    , listener_driver_loader_("perception", "perception::DriverBase")
    , speaker_driver_loader_("perception", "perception::DriverBase")
    , transcription_driver_loader_("perception", "perception::DriverBase")
    , speech_driver_loader_("perception", "perception::DriverBase")
    , sentiment_driver_loader_("perception", "perception::DriverBase")
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
    this->declare_parameter("use_vision_driver", false);
    this->declare_parameter("use_microphone_driver", false);
    this->declare_parameter("use_speaker_driver", false);
    this->declare_parameter("use_transcription_driver", false);
    this->declare_parameter("use_sentiment_driver", false);
    this->declare_parameter("use_speech_driver", false);

    // mischellaneous
    this->declare_parameter("run_tests", false);

    // drivers
    use_vision_driver_ = this->get_parameter("use_vision_driver").as_bool();
    use_microphone_driver_ = this->get_parameter("use_microphone_driver").as_bool();
    use_speaker_driver_ = this->get_parameter("use_speaker_driver").as_bool();
    use_transcription_driver_ = this->get_parameter("use_transcription_driver").as_bool();
    use_sentiment_driver_ = this->get_parameter("use_sentiment_driver").as_bool();
    use_speech_driver_ = this->get_parameter("use_speech_driver").as_bool();

    // run tests
    run_tests_ = this->get_parameter("run_tests").as_bool();

    // Print usage information
    print_usage();

    /*************************************************************************
     * vision driver plugin class loader and driver pointer
     ************************************************************************/

    if (use_vision_driver_)
    {
      this->declare_parameter("vision_driver", "perception::DefaultDriver");
      std::string vision_driver_name = this->get_parameter("vision_driver").as_string();

      RCLCPP_INFO(this->get_logger(), "Loading vision driver plugin: %s", vision_driver_name.c_str());

      vision_driver_ = vision_driver_loader_.createSharedInstance(vision_driver_name);
      vision_driver_->initialize(shared_from_this());
      vision_driver_->start();

      RCLCPP_INFO(this->get_logger(), "Started vision driver plugin: %s", vision_driver_name.c_str());
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Vision driver plugin not loaded.");
    }

    /*************************************************************************
     * microphone driver plugin class loader and driver pointer
     ************************************************************************/

    if (use_microphone_driver_)
    {
      this->declare_parameter("microphone_driver", "perception::MicrophoneAudioDriver");
      std::string mic_driver_name = this->get_parameter("microphone_driver").as_string();

      RCLCPP_INFO(this->get_logger(), "Loading microphone driver plugin: %s", mic_driver_name.c_str());

      microphone_driver_ = listener_driver_loader_.createSharedInstance(mic_driver_name);
      microphone_driver_->initialize(shared_from_this());
      microphone_driver_->start();

      RCLCPP_INFO(this->get_logger(), "Started microphone driver plugin: %s", mic_driver_name.c_str());
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Microphone driver plugin not loaded.");
    }

    /*************************************************************************
     * speaker driver plugin class loader and driver pointer
     ************************************************************************/

    if (use_speaker_driver_)
    {
      this->declare_parameter("speaker_driver", "perception::SpeakerAudioDriver");
      std::string speaker_driver_name = this->get_parameter("speaker_driver").as_string();

      RCLCPP_INFO(this->get_logger(), "Loading speaker driver plugin: %s", speaker_driver_name.c_str());

      speaker_driver_ = speaker_driver_loader_.createSharedInstance(speaker_driver_name);
      speaker_driver_->initialize(shared_from_this());
      speaker_driver_->start();

      RCLCPP_INFO(this->get_logger(), "Started speaker driver plugin: %s", speaker_driver_name.c_str());
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Speaker driver plugin not loaded.");
    }

    /*************************************************************************
     * transcription driver plugin class loader and driver pointer
     ************************************************************************/
    if (use_transcription_driver_)
    {
      this->declare_parameter("transcription_driver", "perception::OpenAIDriver");
      std::string transcription_driver_name = this->get_parameter("transcription_driver").as_string();

      RCLCPP_INFO(this->get_logger(), "Loading transcription driver plugin: %s", transcription_driver_name.c_str());

      transcription_driver_ = transcription_driver_loader_.createSharedInstance(transcription_driver_name);
      transcription_driver_->initialize(shared_from_this());
      transcription_driver_->start();

      RCLCPP_INFO(this->get_logger(), "Started transcription driver plugin: %s", transcription_driver_name.c_str());
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Transcription driver plugin not loaded.");
    }

    /*************************************************************************
     * speech synthesis driver plugin class loader and driver pointer
     ************************************************************************/
    if (use_speech_driver_)
    {
      this->declare_parameter("speech_synthesis_driver", "perception::OpenAISpeechDriver");
      std::string speech_driver_name = this->get_parameter("speech_synthesis_driver").as_string();

      RCLCPP_INFO(this->get_logger(), "Loading speech synthesis driver plugin: %s", speech_driver_name.c_str());

      speech_driver_ = speech_driver_loader_.createSharedInstance(speech_driver_name);
      speech_driver_->initialize(shared_from_this());
      speech_driver_->start();

      RCLCPP_INFO(this->get_logger(), "Started speech synthesis driver plugin: %s", speech_driver_name.c_str());
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Speech synthesis driver plugin not loaded.");
    }

    /*************************************************************************
     * sentiment driver plugin class loader and driver pointer
     ************************************************************************/
    if (use_sentiment_driver_)
    {
      this->declare_parameter("sentiment_driver", "perception::SentimentDriver");
      std::string sentiment_driver_name = this->get_parameter("sentiment_driver").as_string();

      RCLCPP_INFO(this->get_logger(), "Loading sentiment driver plugin: %s", sentiment_driver_name.c_str());

      sentiment_driver_ = sentiment_driver_loader_.createSharedInstance(sentiment_driver_name);
      sentiment_driver_->initialize(shared_from_this());
      sentiment_driver_->start();

      RCLCPP_INFO(this->get_logger(), "Started sentiment driver plugin: %s", sentiment_driver_name.c_str());
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Sentiment driver plugin not loaded.");
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
  }

protected:
  void run_tests()
  {
    RCLCPP_INFO(this->get_logger(), "Running tests for loaded plugins...");

    if (use_vision_driver_)
    {
      RCLCPP_INFO(this->get_logger(), "Testing vision driver...");
      vision_driver_->test();
    }
    else
      RCLCPP_INFO(this->get_logger(), "Vision driver not loaded, skipping test.");

    if (use_microphone_driver_)
    {
      RCLCPP_INFO(this->get_logger(), "Testing microphone driver...");
      microphone_driver_->test();
    }
    else
      RCLCPP_INFO(this->get_logger(), "Microphone driver not loaded, skipping test.");

    if (use_speaker_driver_)
    {
      RCLCPP_INFO(this->get_logger(), "Testing speaker driver...");
      speaker_driver_->test();
    }
    else
      RCLCPP_INFO(this->get_logger(), "Speaker driver not loaded, skipping test.");

    if (use_transcription_driver_)
    {
      RCLCPP_INFO(this->get_logger(), "Testing transcription driver...");
      transcription_driver_->test();
    }
    else
      RCLCPP_INFO(this->get_logger(), "Transcription driver not loaded, skipping test.");

    if (use_speech_driver_)
    {
      RCLCPP_INFO(this->get_logger(), "Testing speech synthesis driver...");
      speech_driver_->test();
    }
    else
      RCLCPP_INFO(this->get_logger(), "Speech synthesis driver not loaded, skipping test.");

    if (use_sentiment_driver_)
    {
      RCLCPP_INFO(this->get_logger(), "Testing sentiment driver...");
      sentiment_driver_->test();
    }
    else
      RCLCPP_INFO(this->get_logger(), "Sentiment driver not loaded, skipping test.");
  }

  void print_usage()
  {  // drivers
    if (use_vision_driver_)
      RCLCPP_INFO(this->get_logger(), "Using vision driver.");
    else
      RCLCPP_INFO(this->get_logger(), "Will not use vision driver.");

    if (use_microphone_driver_)
      RCLCPP_INFO(this->get_logger(), "Using microphone driver.");
    else
      RCLCPP_INFO(this->get_logger(), "Will not use microphone driver.");

    if (use_speaker_driver_)
      RCLCPP_INFO(this->get_logger(), "Using speaker driver.");
    else
      RCLCPP_INFO(this->get_logger(), "Will not use speaker driver.");

    if (use_transcription_driver_)
      RCLCPP_INFO(this->get_logger(), "Using transcription driver.");
    else
      RCLCPP_INFO(this->get_logger(), "Will not use transcription driver.");

    if (use_speech_driver_)
      RCLCPP_INFO(this->get_logger(), "Using speech synthesis driver.");
    else
      RCLCPP_INFO(this->get_logger(), "Will not use speech synthesis driver.");

    if (use_sentiment_driver_)
      RCLCPP_INFO(this->get_logger(), "Using sentiment driver.");
    else
      RCLCPP_INFO(this->get_logger(), "Will not use sentiment driver.");
  }

  /** Run Tests */
  bool run_tests_;

  bool use_vision_driver_;
  bool use_microphone_driver_;
  bool use_speaker_driver_;
  bool use_transcription_driver_;
  bool use_sentiment_driver_;
  bool use_speech_driver_;

  /** plugin loader for vision driver */
  pluginlib::ClassLoader<perception::DriverBase> vision_driver_loader_;

  /** shared pointer for vision driver */
  std::shared_ptr<perception::DriverBase> vision_driver_;

  /** plugin loader for audio listener driver */
  pluginlib::ClassLoader<perception::DriverBase> listener_driver_loader_;

  /** shared pointer for audio listener driver */
  std::shared_ptr<perception::DriverBase> microphone_driver_;

  /** plugin loader for audio speaker driver */
  pluginlib::ClassLoader<perception::DriverBase> speaker_driver_loader_;

  /** shared pointer for audio speaker driver */
  std::shared_ptr<perception::DriverBase> speaker_driver_;

  /** plugin loader for transcription driver */
  pluginlib::ClassLoader<perception::DriverBase> transcription_driver_loader_;

  /** shared pointer for transcription driver */
  std::shared_ptr<perception::DriverBase> transcription_driver_;

  /** plugin loader for sentiment driver */
  pluginlib::ClassLoader<perception::DriverBase> sentiment_driver_loader_;

  /** shared pointer for sentiment driver */
  std::shared_ptr<perception::DriverBase> sentiment_driver_;

  /** plugin loader for speech synthesis driver */
  pluginlib::ClassLoader<perception::DriverBase> speech_driver_loader_;

  /** shared pointer for speech synthesis driver */
  std::shared_ptr<perception::DriverBase> speech_driver_;
};

}  // namespace perception