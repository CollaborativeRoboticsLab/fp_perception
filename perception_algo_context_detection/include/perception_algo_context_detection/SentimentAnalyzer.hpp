#pragma once

#include <string>
#include <future>
#include <curl/curl.h>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <sstream>
#include <iostream>

/**
 * @brief This class provides an interface to the Hugging Face Sentiment Analysis API.
 *
 * It uses libcurl for HTTP requests and nlohmann::json for JSON parsing.
 *
 */
class SentimentAnalyzer
{
public:
  /**
   * @brief Constructor for SentimentAnalyzer
   *
   * @param api_key The API key for Hugging Face API
   * @param model The model to use for sentiment analysis (default: "distilbert-base-uncased-finetuned-sst-2-english")
   */
  SentimentAnalyzer(const std::string& api_key, const std::string& model = "distilbert-base-uncased-finetuned-sst-2-"
                                                                           "english")
    : api_key_(api_key), model_(model)
  {
  }

  /**
   * @brief Analyze sentiment of the given text
   *
   * @param text The input text to analyze
   * @return A pair containing the sentiment label and its score
   * @throws std::runtime_error if the API request fails
   */
  std::pair<std::string, float> analyze(const std::string& text) const
  {
    CURL* curl;
    CURLcode res;
    std::string read_buffer;

    curl_global_init(CURL_GLOBAL_ALL);
    curl = curl_easy_init();

    if (!curl)
      throw std::runtime_error("Failed to initialize CURL");

    // Create request JSON
    nlohmann::json body_json;
    body_json["inputs"] = text;

    std::string json_data = body_json.dump();

    // Set headers
    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    headers = curl_slist_append(headers, ("Authorization: Bearer " + api_key_).c_str());

    // Set cURL options
    std::string url = "https://api-inference.huggingface.co/models/" + model_;
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_data.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &read_buffer);

    // Perform request
    res = curl_easy_perform(curl);

    curl_easy_cleanup(curl);
    curl_slist_free_all(headers);

    if (res != CURLE_OK)
    {
      throw std::runtime_error("Sentiment API call failed: " + std::string(curl_easy_strerror(res)));
    }

    auto json = nlohmann::json::parse(read_buffer);
    std::string label = json[0]["label"].get<std::string>();
    float score = json[0]["score"].get<float>();

    return std::make_pair(label, score);
  }

private:
  /**
   * @brief API key for Hugging Face API
   */
  std::string api_key_;

  /**
   * @brief Model to use for sentiment analysis
   */
  std::string model_;

  /**
   * @brief Callback function for writing response data
   *
   * @param contents Pointer to the data received
   * @param size Size of each element
   * @param nmemb Number of elements
   * @param s String to append the data to
   * @return Total size of the data written
   */
  static size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* s)
  {
    size_t total_size = size * nmemb;
    s->append(static_cast<char*>(contents), total_size);
    return total_size;
  }
};
