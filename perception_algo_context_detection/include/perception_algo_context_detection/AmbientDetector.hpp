#pragma once

#include <torch/script.h>
#include <iostream>
#include <string>

/**
 * @brief This class is responsible for detecting ambient context using a pre-trained PyTorch model.
 *
 * It loads the model from a specified path and provides a method to predict the ambient context
 * based on input features. The prediction includes the class index and confidence score.
 */
class AmbientDetector
{
public:
  explicit AmbientDetector(const std::string& model_path)
  {
    try
    {
      module_ = torch::jit::load(model_path);
    }
    catch (const c10::Error& e)
    {
      std::cerr << "Error loading the model: " << e.what() << std::endl;
      throw;
    }
  }

  /**
   * @brief Predict the ambient context from input features.
   *
   * @param input The input tensor containing features for prediction.
   * @param class_index Output parameter to store the predicted class index.
   * @param confidence Output parameter to store the confidence score of the prediction.
   */
  void predict(const torch::Tensor& input, int& class_index, float& confidence)
  {
    // Evaluate the model with the input tensor
    torch::Tensor output = module_.forward({ input }).toTensor();

    // Run softmax on the output tensor to get probabilities
    torch::Tensor softmaxed = torch::softmax(output, 1);

    // Get the maximum probability and its corresponding index
    auto max_result = softmaxed.max(1, true);

    confidence = max_result.values.item<float>();
    class_index = max_result.indices.item<int>();
  }

private:
  torch::jit::script::Module module_;
};