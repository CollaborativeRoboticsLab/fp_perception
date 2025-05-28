#pragma once

#include <vector>
#include <string>
#include <algorithm>

/**
 * @brief Naive Bayes Classifier implementation
 *
 * This class implements a simple Naive Bayes classifier for multi-class classification.
 * It uses prior probabilities and likelihoods to predict the class of a given feature vector.
 */

class NaiveBayesClassifier
{
public:
  /**
   * @brief Constructor for NaiveBayesClassifier
   *
   * @param priors Prior probabilities for each class
   * @param likelihoods Likelihoods of features given each class
   * @param labels Class labels corresponding to the indices of priors and likelihoods
   */
  NaiveBayesClassifier(const std::vector<double>& priors, const std::vector<std::vector<double>>& likelihoods,
                       const std::vector<std::string>& labels)
    : priors_(priors), likelihoods_(likelihoods), labels_(labels)
  {
  }

  /**
   * @brief Predict the class for a given feature vector
   *
   * @param features Feature vector for which to predict the class
   * @return A pair containing the predicted class label and posterior probabilities for each class
   */
  std::pair<std::string, std::vector<double>> predict(const std::vector<double>& features) const
  {
    std::vector<double> posteriors = priors_;

    for (size_t c = 0; c < posteriors.size(); ++c)
    {
      for (size_t i = 0; i < features.size(); ++i)
      {
        posteriors[c] *= likelihoods_[c][i] * features[i];
      }
    }

    auto max_it = std::max_element(posteriors.begin(), posteriors.end());
    size_t best_class = std::distance(posteriors.begin(), max_it);
    return { labels_[best_class], posteriors };
  }

private:
  /**
   * @brief Prior probabilities for each class
   */
  std::vector<double> priors_;

  /**
   * @brief Likelihoods of features given each class
   *
   * Each row corresponds to a class, and each column corresponds to a feature.
   */
  std::vector<std::vector<double>> likelihoods_;

  /**
   * @brief Class labels corresponding to the indices of priors and likelihoods
   */
  std::vector<std::string> labels_;
};