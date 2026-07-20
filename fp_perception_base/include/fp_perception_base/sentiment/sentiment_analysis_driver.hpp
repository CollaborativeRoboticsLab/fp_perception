#pragma once

#include <fp_perception_base/driver_base.hpp>
#include <fp_perception_base/sentiment/structs.hpp>

namespace fp_perception
{

class SentimentAnalysisDriver : public virtual DriverBase
{
public:
  virtual sentiment_result analyze(const sentiment_request& request) = 0;
};

}  // namespace fp_perception