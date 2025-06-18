#pragma once

#include <Poco/Net/PartSource.h>

#include <sstream>
#include <string>
#include <vector>

/**
 * @brief FilePartSource
 *
 * A memory-based PartSource implementation for streaming binary content
 * such as audio files (e.g. WAV). Supports both raw byte input and int16_t PCM samples.
 */
class FilePartSource : public Poco::Net::PartSource
{
public:
  /**
   * @brief Constructor for binary (char) input
   * @param mediaType MIME type of the file (e.g. "audio/wav")
   * @param data Raw binary buffer (e.g. WAV file)
   */
  FilePartSource(const std::string& mediaType, const std::vector<char>& data)
    : Poco::Net::PartSource(mediaType), stream_(&buffer_)
  {
    buffer_.sputn(data.data(), data.size());
    buffer_.pubseekpos(0);
    filename_ = "audio.wav";  // Default filename
  }

  ~FilePartSource() override = default;

  std::istream& stream() override
  {
    return stream_;
  }

  const std::string& filename() const override
  {
    return filename_;
  }

private:
  std::stringbuf buffer_;
  std::istream stream_;
  std::string filename_;
};
