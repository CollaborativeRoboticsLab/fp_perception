#pragma once

#include <string>
#include <fstream>
#include <sstream>
#include <curl/curl.h>
#include <nlohmann/json.hpp>

/**
 * @brief WhisperAPI class
 *
 * This class provides an interface to the OpenAI Whisper API for audio transcription.
 * It uses libcurl for HTTP requests and nlohmann::json for JSON parsing.
 *
 */
class WhisperAPI
{
public:
  /**
   * @brief Constructor for WhisperAPI
   *
   * @param api_key The API key for OpenAI Whisper API
   */
  explicit WhisperAPI(const std::string& api_key) : api_key_(api_key)
  {
  }

  /**
   * @brief Transcribe audio file using Whisper API
   *
   * @param audio_path Path to the audio file to be transcribed
   * @return std::string The transcribed text
   * @throws std::runtime_error if the API request fails
   */
  std::string transcribe(const std::string& audio_path) const
  {
    CURL* curl;
    CURLcode res;
    curl_mime* mime;
    curl_mimepart* part;
    std::string read_buffer;

    curl_global_init(CURL_GLOBAL_ALL);
    curl = curl_easy_init();

    if (curl)
    {
      mime = curl_mime_init(curl);

      part = curl_mime_addpart(mime);
      curl_mime_name(part, "file");
      curl_mime_filedata(part, audio_path.c_str());

      part = curl_mime_addpart(mime);
      curl_mime_name(part, "model");
      curl_mime_data(part, "whisper-1", CURL_ZERO_TERMINATED);

      struct curl_slist* headers = nullptr;
      headers = curl_slist_append(headers, ("Authorization: Bearer " + api_key_).c_str());

      curl_easy_setopt(curl, CURLOPT_URL, "https://api.openai.com/v1/audio/transcriptions");
      curl_easy_setopt(curl, CURLOPT_MIMEPOST, mime);
      curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
      curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
      curl_easy_setopt(curl, CURLOPT_WRITEDATA, &read_buffer);

      res = curl_easy_perform(curl);

      curl_slist_free_all(headers);
      curl_mime_free(mime);
      curl_easy_cleanup(curl);

      if (res != CURLE_OK)
      {
        throw std::runtime_error("Whisper API request failed: " + std::string(curl_easy_strerror(res)));
      }

      auto json_response = nlohmann::json::parse(read_buffer);
      return json_response["text"].get<std::string>();
    }

    throw std::runtime_error("Failed to initialize CURL");
  }

private:
  std::string api_key_;

  /**
   * @brief Callback function for writing data received from the API
   *
   * @param contents Pointer to the data received
   * @param size Size of each element in the data
   * @param nmemb Number of elements in the data
   * @param s String to append the data to
   * @return size_t The number of bytes written
   */
  static size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* s)
  {
    size_t new_length = size * nmemb;
    s->append((char*)contents, new_length);
    return new_length;
  }
};