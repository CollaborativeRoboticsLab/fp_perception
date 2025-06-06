#pragma once

#include <vector>
#include <torch/torch.h>
#include <cmath>
#include <iostream>
#include <perception_algo_context_detection/utils/librosa.h>

/**
 * @brief Extracts MFCC features from audio data using librosa-style logic in C++.
 *
 * This class provides methods to extract MFCC features from audio samples,
 * applying pre-emphasis and returning the features as a Torch tensor.
 */
class MFCCExtractor
{
public:
  /**
   * @brief Constructor for MFCCExtractor
   *
   * @param sample_rate Sample rate of the audio (default: 44100)
   * @param n_mfcc Number of MFCC coefficients to extract (default: 40)
   * @param fft_size Size of the FFT window (default: 2048)
   * @param hop_length Hop length for the STFT (default: 512)
   * @param fixed_length Fixed length of the output MFCC features (default: 200)
   */
  MFCCExtractor(int sample_rate = 44100, int n_mfcc = 40, int fft_size = 2048, int hop_length = 512,
                int fixed_length = 200)
    : sample_rate_(sample_rate)
    , n_mfcc_(n_mfcc)
    , fft_size_(fft_size)
    , hop_length_(hop_length)
    , fixed_length_(fixed_length)
  {
  }

  /**
   * @brief Extract MFCC features and return as a Torch tensor
   *
   * @param audio Audio samples (int16_t)
   * @return torch::Tensor [1, n_mfcc_, frames]
   */
  torch::Tensor extract(const std::vector<int16_t>& audio)
  {
    std::vector<float> emphasized = pre_emphasis(audio);

    // values in input are emphasized audio, sample_rate, fft_size, hop_length, window type, center, padding mode,
    // power, n_mels, fmin, fmax, n_mfcc, norm, dct_type
    auto mfcc_data = librosa::Feature::mfcc(emphasized, sample_rate_, fft_size_, hop_length_, "hann", true, "reflect",
                                            2.0f, 128, 0, sample_rate_ / 2, n_mfcc_, true, 2);

    // mfcc_data length
    int frames = static_cast<int>(mfcc_data.size());

    // number of MFCC dimensions
    int dims = frames > 0 ? static_cast<int>(mfcc_data[0].size()) : 0;

    // Pad or truncate MFCC to fixed_length
    if (frames < fixed_length_)
    {
      // Padding with zeros
      // for (auto& frame : mfcc_data)
      // {
      //   frame.resize(fixed_length, 0.0f);  // Pad each MFCC vector to fixed_length columns
      // }

      // If fewer frames, add extra all-zero frames
      while (static_cast<int>(mfcc_data.size()) < fixed_length_)
      {
        mfcc_data.push_back(std::vector<float>(n_mfcc_, 0.0f));
      }

      frames = fixed_length_;  // Update frame count
    }
    else if (frames > fixed_length_)
    {
      // Truncate to fixed length
      mfcc_data.resize(fixed_length_);
      frames = fixed_length_;
    }

    torch::Tensor mfcc_tensor = torch::empty({ 1, frames, dims }, torch::kFloat32);

    for (int i = 0; i < frames; ++i)
    {
      for (int j = 0; j < dims; ++j)
      {
        mfcc_tensor[0][i][j] = mfcc_data[i][j];
      }
    }

    return mfcc_tensor.permute({ 0, 2, 1 });  // Reshape to [1, n_mfcc, frames] to match CNN input
  }

private:
  /**
   * @brief Sample rate of the audio
   */
  int sample_rate_;

  /**
   * @brief Number of MFCC coefficients to extract
   */
  int n_mfcc_;

  /**
   * @brief Size of the FFT window
   */
  int fft_size_;

  /**
   * @brief Hop length for the STFT
   */
  int hop_length_;

  /**
   * @brief Fixed length of the output MFCC features
   */
  int fixed_length_;

  /**
   * @brief Apply pre-emphasis to the audio signal
   *
   * @param audio Audio samples (int16_t)
   * @return std::vector<float> Emphasized audio samples
   */
  std::vector<float> pre_emphasis(const std::vector<int16_t>& audio)
  {
    std::vector<float> emphasized(audio.size());
    emphasized[0] = static_cast<float>(audio[0]);

    for (size_t i = 1; i < audio.size(); ++i)
    {
      emphasized[i] = static_cast<float>(audio[i]) - 0.97f * static_cast<float>(audio[i - 1]);
    }

    return emphasized;
  }
};
