#pragma once

#include <thread>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>
#include <perception_base/driver_base.hpp>
#include <perception_base/algorithm_base.hpp>
#include <perception_events/event_client.hpp>

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
    , sentiment_driver_loader_("perception", "perception::DriverBase")
    , eye_gaze_algorithm_loader_("perception", "perception::AlgorithmBase")
    , context_algorithm_loader_("perception", "perception::AlgorithmBase")
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
    event_ = std::make_shared<EventClient>(shared_from_this(), "perception_server", "/events");

    event_->info("PerceptionServer initializing...");

    /*************************************************************************
     * Identify what plugins to load
     ************************************************************************/
    // drivers
    this->declare_parameter("use_vision_driver", false);
    this->declare_parameter("use_microphone_driver", false);
    this->declare_parameter("use_speaker_driver", false);
    this->declare_parameter("use_transcription_driver", false);
    this->declare_parameter("use_sentiment_driver", false);

    // algorithms
    this->declare_parameter("use_eye_gaze_algorithm", false);
    this->declare_parameter("use_context_identification_algorithm", false);

    // mischellaneous
    this->declare_parameter("run_tests", false);

    // drivers
    bool use_vision_driver_ = this->get_parameter("use_vision_driver").as_bool();
    bool use_microphone_driver_ = this->get_parameter("use_microphone_driver").as_bool();
    bool use_speaker_driver_ = this->get_parameter("use_speaker_driver").as_bool();
    bool use_transcription_driver_ = this->get_parameter("use_transcription_driver").as_bool();
    bool use_sentiment_driver_ = this->get_parameter("use_sentiment_driver").as_bool();

    // algorithms
    bool use_eye_gaze_algorithm_ = this->get_parameter("use_eye_gaze_algorithm").as_bool();
    bool use_context_identification_algorithm_ = this->get_parameter("use_context_identification_algorithm").as_bool();

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

      event_->info("Loading vision driver plugin: " + vision_driver_name);

      vision_driver_ = vision_driver_loader_.createSharedInstance(vision_driver_name);
      vision_driver_->initialize(shared_from_this());
      vision_driver_->start();

      event_->info("Started vision driver plugin: " + vision_driver_name);
    }
    else
    {
      event_->info("Vision driver plugin not loaded.");
    }

    /*************************************************************************
     * microphone driver plugin class loader and driver pointer
     ************************************************************************/

    if (use_microphone_driver_)
    {
      this->declare_parameter("microphone_driver", "perception::MicrophoneAudioDriver");
      std::string mic_driver_name = this->get_parameter("microphone_driver").as_string();

      event_->info("Loading microphone driver plugin: " + mic_driver_name);

      microphone_driver_ = listener_driver_loader_.createSharedInstance(mic_driver_name);
      microphone_driver_->initialize(shared_from_this());
      microphone_driver_->start();

      event_->info("Started microphone driver plugin: " + mic_driver_name);
    }
    else
    {
      event_->info("Microphone driver plugin not loaded.");
    }

    /*************************************************************************
     * speaker driver plugin class loader and driver pointer
     ************************************************************************/

    if (use_speaker_driver_)
    {
      this->declare_parameter("speaker_driver", "perception::SpeakerAudioDriver");
      std::string speaker_driver_name = this->get_parameter("speaker_driver").as_string();

      event_->info("Loading speaker driver plugin: " + speaker_driver_name);

      speaker_driver_ = speaker_driver_loader_.createSharedInstance(speaker_driver_name);
      speaker_driver_->initialize(shared_from_this());
      speaker_driver_->start();

      event_->info("Started speaker driver plugin: " + speaker_driver_name);
    }
    else
    {
      event_->info("Speaker driver plugin not loaded.");
    }

    /*************************************************************************
     * transcription driver plugin class loader and driver pointer
     ************************************************************************/
    if (use_transcription_driver_)
    {
      this->declare_parameter("transcription_driver", "perception::PromptToolsTranscribeDriver");
      std::string transcription_driver_name = this->get_parameter("transcription_driver").as_string();

      event_->info("Loading transcription driver plugin: " + transcription_driver_name);

      transcription_driver_ = transcription_driver_loader_.createSharedInstance(transcription_driver_name);
      transcription_driver_->initialize(shared_from_this());
      transcription_driver_->start();

      event_->info("Started transcription driver plugin: " + transcription_driver_name);
    }
    else
    {
      event_->info("Transcription driver plugin not loaded.");
    }

    /*************************************************************************
     * sentiment driver plugin class loader and driver pointer
     ************************************************************************/
    if (use_sentiment_driver_)
    {
      this->declare_parameter("sentiment_driver", "perception::PromptToolsSentimentDriver");
      std::string sentiment_driver_name = this->get_parameter("sentiment_driver").as_string();

      event_->info("Loading sentiment driver plugin: " + sentiment_driver_name);

      sentiment_driver_ = sentiment_driver_loader_.createSharedInstance(sentiment_driver_name);
      sentiment_driver_->initialize(shared_from_this());
      sentiment_driver_->start();

      event_->info("Started sentiment driver plugin: " + sentiment_driver_name);
    }
    else
    {
      event_->info("Sentiment driver plugin not loaded.");
    }

    /*************************************************************************
     * eye gaze algorithm plugin class loader and driver pointer
     ************************************************************************/

    if (use_eye_gaze_algorithm_)
    {
      this->declare_parameter("eye_gaze_algorithm", "perception::GazeAlgorithm");
      std::string eye_gaze_algorithm_name = this->get_parameter("eye_gaze_algorithm").as_string();

      event_->info("Loading eye gaze algorithm plugin: " + eye_gaze_algorithm_name);

      eye_gaze_algorithm_ = eye_gaze_algorithm_loader_.createSharedInstance(eye_gaze_algorithm_name);
      eye_gaze_algorithm_->initialize(shared_from_this());
      eye_gaze_algorithm_->set_vision_driver(vision_driver_);
      eye_gaze_algorithm_->start();

      event_->info("Started eye gaze algorithm plugin: " + eye_gaze_algorithm_name);
    }
    else
    {
      event_->info("Eye gaze algorithm plugin not loaded.");
    }

    /*************************************************************************
     * context identification algorithm plugin class loader and driver pointer
     ************************************************************************/
    if (use_context_identification_algorithm_)
    {
      this->declare_parameter("context_identification_algorithm", "perception::ContextAlgorithm");
      std::string context_algorithm_name = this->get_parameter("context_identification_algorithm").as_string();

      event_->info("Loading context identification algorithm plugin: " + context_algorithm_name);

      context_identification_algorithm_ = context_algorithm_loader_.createSharedInstance(context_algorithm_name);
      context_identification_algorithm_->initialize(shared_from_this());
      context_identification_algorithm_->set_audio_input_driver(microphone_driver_);
      context_identification_algorithm_->set_transcription_driver(transcription_driver_);
      context_identification_algorithm_->set_sentiment_driver(sentiment_driver_);
      context_identification_algorithm_->start();

      event_->info("Started context identification algorithm plugin: " + context_algorithm_name);
    }
    else
    {
      event_->info("Context identification algorithm plugin not loaded.");
    }

    /*************************************************************************
     * Run tests if requested
     ************************************************************************/
    if (run_tests_)
    {
      event_->info("Running tests...");
      run_tests();
      event_->info("Tests completed.");
    }
  }

protected:
  void run_tests()
  {
    event_->info("Running tests for loaded plugins...");

    if (vision_driver_)
    {
      event_->info("Testing vision driver...");
      vision_driver_->test();
    }
    else
      event_->info("Vision driver not loaded, skipping test.");

    if (microphone_driver_)
    {
      event_->info("Testing microphone driver...");
      microphone_driver_->test();
    }
    else
      event_->info("Microphone driver not loaded, skipping test.");

    if (speaker_driver_)
    {
      event_->info("Testing speaker driver...");
      speaker_driver_->test();
    }
    else
      event_->info("Speaker driver not loaded, skipping test.");

    if (transcription_driver_)
    {
      event_->info("Testing transcription driver...");
      transcription_driver_->test();
    }
    else
      event_->info("Transcription driver not loaded, skipping test.");

    if (sentiment_driver_)
    {
      event_->info("Testing sentiment driver...");
      sentiment_driver_->test();
    }
    else
      event_->info("Sentiment driver not loaded, skipping test.");
  }

  void print_usage()
  {  // drivers
    if (use_vision_driver_)
      event_->info("Will use vision driver.");
    else
      event_->info("Will not use vision driver.");

    if (use_microphone_driver_)
      event_->info("Will use microphone driver.");
    else
      event_->info("Will not use microphone driver.");

    if (use_speaker_driver_)
      event_->info("Will use speaker driver.");
    else
      event_->info("Will not use speaker driver.");

    if (use_transcription_driver_)
      event_->info("Will use transcription driver.");
    else
      event_->info("Will not use transcription driver.");

    if (use_sentiment_driver_)
      event_->info("Will use sentiment driver.");
    else
      event_->info("Will not use sentiment driver.");

    // algorithms
    if (use_eye_gaze_algorithm_)
      event_->info("Will use eye gaze algorithm.");
    else
      event_->info("Will not use eye gaze algorithm.");

    if (use_context_identification_algorithm_)
      event_->info("Will use context identification algorithm.");
    else
      event_->info("Will not use context identification algorithm.");
  }

  /** Run Tests */
  bool run_tests_;

  bool use_vision_driver_;
  bool use_microphone_driver_;
  bool use_speaker_driver_;
  bool use_transcription_driver_;
  bool use_sentiment_driver_;
  bool use_eye_gaze_algorithm_;
  bool use_context_identification_algorithm_;

  /** client for publishing events */
  std::shared_ptr<EventClient> event_;

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

  /** plugin loader for eye gaze algorithm */
  pluginlib::ClassLoader<perception::AlgorithmBase> eye_gaze_algorithm_loader_;

  /** shared pointer for eye gaze algorithm */
  std::shared_ptr<perception::AlgorithmBase> eye_gaze_algorithm_;

  /** plugin loader for context identification algorithm */
  pluginlib::ClassLoader<perception::AlgorithmBase> context_algorithm_loader_;

  /** shared pointer for context identification algorithm */
  std::shared_ptr<perception::AlgorithmBase> context_identification_algorithm_;
};

}  // namespace perception