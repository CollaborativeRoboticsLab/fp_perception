#pragma once

#include <perception_base/driver_base.hpp>
#include <perception_base/sentiment/structs.hpp>

namespace perception
{

class SentimentAnalysisDriver : public virtual DriverBase
{
public:
  virtual sentiment_result analyze(const sentiment_request& request) = 0;
};

}  // namespace perception