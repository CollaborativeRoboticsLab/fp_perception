#pragma once

#include <memory>

#include <fp_perception_base/exceptions.hpp>
#include <fp_perception_base/image_analysis/image_analysis_driver.hpp>
#include <fp_perception_base/image_analysis/structs.hpp>
#include <fp_perception_base/vision/vision_source_driver.hpp>

namespace fp_perception
{

class ImageAnalysisPipeline
{
public:
  ImageAnalysisPipeline(std::shared_ptr<ImageAnalysisDriver> image_analysis_driver,
                        std::shared_ptr<VisionSourceDriver> ros_vision_driver = nullptr,
                        std::shared_ptr<VisionSourceDriver> non_ros_vision_driver = nullptr)
    : image_analysis_driver_(std::move(image_analysis_driver))
    , ros_vision_driver_(std::move(ros_vision_driver))
    , non_ros_vision_driver_(std::move(non_ros_vision_driver))
  {
  }

  image_analysis_result run(image_analysis_request request)
  {
    if (!image_analysis_driver_)
      throw fp_perception_exception("Image analysis driver is not loaded.");

    if (request.use_device_vision)
    {
      if (non_ros_vision_driver_)
      {
        request.frame = non_ros_vision_driver_->captureFrame();
      }
      else if (ros_vision_driver_)
      {
        request.frame = ros_vision_driver_->captureFrame();
      }
      else
      {
        throw fp_perception_exception("use_device_vision requested but no vision driver is loaded");
      }
    }

    return image_analysis_driver_->analyze(request);
  }

private:
  std::shared_ptr<ImageAnalysisDriver> image_analysis_driver_;
  std::shared_ptr<VisionSourceDriver> ros_vision_driver_;
  std::shared_ptr<VisionSourceDriver> non_ros_vision_driver_;
};

}  // namespace fp_perception