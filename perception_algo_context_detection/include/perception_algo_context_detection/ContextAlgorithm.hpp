#pragma once

#include <perception_base/algorithm_base.hpp>
#include <perception_algo_context_detection/MFCCExtractor.hpp>
#include <perception_algo_context_detection/AmbientDetector.hpp>
#include <perception_algo_context_detection/WhisperAPI.hpp>
#include <perception_algo_context_detection/SentimentAnalyzer.hpp>
#include <perception_algo_context_detection/NaiveBayesClassifier.hpp>
#include <rclcpp/rclcpp.hpp>
#include <any>
#include <vector>

namespace perception
{

/**
 * @brief ContextAlgorithm class
 *
 * This class implements a context detection algorithm that processes audio data,
 * extracts features, performs classification, and analyzes sentiment.
 */
class ContextAlgorithm : public AlgorithmBase
{
public:
  /**
   * @brief Constructor for ContextAlgorithm
   */
  ContextAlgorithm()
  {
  }

  /**
   * @brief Destructor for ContextAlgorithm
   *
   * Stops the timer if it is running.
   */
  ~ContextAlgorithm()
  {
    stop();

    if (timer_)
      timer_->cancel();
  }

  /**
   * @brief Initialize the ContextAlgorithm
   *
   * This function initializes the algorithm with the given ROS node.
   *
   * @param node Sared pointer to the ROS node
   */
  void initialize(const rclcpp::Node::SharedPtr& node) override
  {
    // Declare parameters for the algorithm
    node->declare_parameter("algorithm.ContextAlgorithm.name", "ContextDetection");
    node->declare_parameter("algorithm.ContextAlgorithm.MFCCExtractor.sample_rate", 44100);
    node->declare_parameter("algorithm.ContextAlgorithm.MFCCExtractor.n_mfcc", 40);
    node->declare_parameter("algorithm.ContextAlgorithm.MFCCExtractor.fft_size", 1024);
    node->declare_parameter("algorithm.ContextAlgorithm.MFCCExtractor.hop_length", 512);
    node->declare_parameter("algorithm.ContextAlgorithm.MFCCExtractor.fixed_length", 200);
    node->declare_parameter("algorithm.ContextAlgorithm.AmbientDetector.model_path", "ambient_model.pt");
    node->declare_parameter("algorithm.ContextAlgorithm.SentimentAnalyzer.model_name", "distilbert-base-uncased-"
                                                                                       "finetuned-sst-2-"
                                                                                       "english");

    // Get parameters from the node
    config_.name = node->get_parameter("algorithm.ContextAlgorithm.name").as_string();
    sample_rate_ = node->get_parameter("algorithm.ContextAlgorithm.MFCCExtractor.sample_rate").as_int();
    n_mfcc_ = node->get_parameter("algorithm.ContextAlgorithm.MFCCExtractor.n_mfcc").as_int();
    fft_size_ = node->get_parameter("algorithm.ContextAlgorithm.MFCCExtractor.fft_size").as_int();
    hop_length_ = node->get_parameter("algorithm.ContextAlgorithm.MFCCExtractor.hop_length").as_int();
    fixed_length_ = node->get_parameter("algorithm.ContextAlgorithm.MFCCExtractor.fixed_length").as_int();
    ambient_model_path = node->get_parameter("algorithm.ContextAlgorithm.AmbientDetector.model_path").as_string();
    sentiment_model_name_ = node->get_parameter("algorithm.ContextAlgorithm.SentimentAnalyzer.model_name").as_string();

    // Publish about the assigned parameters
    event_->info("Assigned algorithm name: " + config_.name);
    event_->info("Assigned MFCC sample rate: " + std::to_string(sample_rate_));
    event_->info("Assigned MFCC n_mfcc: " + std::to_string(n_mfcc_));
    event_->info("Assigned MFCC fft_size: " + std::to_string(fft_size_));
    event_->info("Assigned MFCC hop_length: " + std::to_string(hop_length_));
    event_->info("Assigned MFCC fixed_length: " + std::to_string(fixed_length_));
    event_->info("Assigned ambient detection model path: " + ambient_model_path);
    event_->info("Assigned sentiment analysis model name: " + sentiment_model_name_);

    initialize_base(node);

    // Initialize the MFCC extractor
    mfcc_ = std::make_shared<MFCCExtractor>(sample_rate_, n_mfcc_, fft_size_, hop_length_, fixed_length_);

    // Initialize the ambient detector
    ambient_model_ = std::make_shared<AmbientDetector>(ambient_model_path);

    // Initialize the Whisper API client
    whisper_ = std::make_shared<WhisperAPI>(std::getenv("OPENAI_API_KEY"));

    // Initialize the sentiment analyzer
    sentiment_ = std::make_shared<SentimentAnalyzer>(std::getenv("HUGGINGFACE_API_KEY"), sentiment_model_name_);

    // Initialize the Naive Bayes classifier
    nbclassifier_ = std::make_shared<NaiveBayesClassifier>(
        std::vector<float>{ 0.33, 0.33, 0.34 },
        std::vector<std::vector<float>>{ { 1.0, 1.0, 1.0 }, { 1.0, 1.0, 1.0 }, { 1.0, 1.0, 1.0 } },
        std::vector<std::string>{ "Alarmed", "Social", "Disengaged" });
  }

  void start() override
  {
    event_->info("ContextAlgorithm started.");
    timer_ = node_->create_wall_timer(std::chrono::seconds(3), std::bind(&ContextAlgorithm::process_loop, this));
  }

  void stop() override
  {
    timer_.reset();
    event_->info("ContextAlgorithm stopped.");
  }

private:
  /** Sample rate for audio processing */
  int sample_rate_;

  /** Number of MFCC coefficients to extract */
  int n_mfcc_;

  /** FFT size for audio processing */
  int fft_size_;

  /** Hop length for audio processing */
  int hop_length_;

  /** Fixed length of the output MFCC features */
  int fixed_length_;

  /** Model path to ambient detection model */
  std::string ambient_model_path;

  /** Name of the sentiment analysis model */
  std::string sentiment_model_name_;

  /**  pointer to the MFCC extractor */
  std::shared_ptr<MFCCExtractor> mfcc_;

  /**  pointer to the ambient detector */
  std::shared_ptr<AmbientDetector> ambient_model_;

  /**  pointer to the Whisper API client */
  std::shared_ptr<WhisperAPI> whisper_;

  /**  pointer to the sentiment analyzer */
  std::shared_ptr<SentimentAnalyzer> sentiment_;

  /**  pointer to the Naive Bayes classifier */
  std::shared_ptr<NaiveBayesClassifier> nbclassifier_;

  rclcpp::TimerBase::SharedPtr timer_;

  /**
   * @brief Process loop for context detection
   *
   * This function runs in a loop to process audio data, extract features, perform classification,
   * and analyze sentiment. It handles exceptions and logs events.
   */
  void process_loop()
  {
    try
    {
      // 1. Get audio data
      std::vector<int16_t> audio_chunk = std::any_cast<std::vector<int16_t>>(audio_input_driver_->getDataStream());

      // 2. Extract MFCC features
      auto mfcc_tensor = mfcc_->extract(audio_chunk);

      // 3. CNN prediction
      float ambient_conf;
      int ambient_class;
      ambient_model_->predict(mfcc_tensor, ambient_class, ambient_conf);

      std::string label = (ambient_class == 0 ? "Alarmed" : ambient_class == 1 ? "Social" : "Disengaged");

      // 4. Save audio to file
      std::string filepath = "mic.wav";
      std::ofstream out(filepath, std::ios::binary);
      for (auto s : audio_chunk)
        out.write(reinterpret_cast<const char*>(&s), sizeof(int16_t));
      out.close();

      // 5. Transcribe and analyze
      std::string text = whisper_->transcribe(filepath);
      auto [sentiment_label, sentiment_score] = sentiment_->analyze(text);

      // 6. Determine keyword confidence
      float keyword_conf =
          (text.find("help") != std::string::npos || text.find("fire") != std::string::npos) ? 1.0f : 0.2f;

      float sentiment_conf = (sentiment_label == "NEGATIVE") ? sentiment_score : (1.0f - sentiment_score);

      // 7. Context classification
      auto [final_label, probs] = nbclassifier_->predict({ ambient_conf, keyword_conf, sentiment_conf });

      event_->info("Final Context: " + final_label + " (Audio=" + label + ", Sentiment=" + sentiment_label + ")");
    }
    catch (const std::exception& e)
    {
      event_->error(std::string("Exception in context loop: ") + e.what());
    }
  }
};

}  // namespace perception