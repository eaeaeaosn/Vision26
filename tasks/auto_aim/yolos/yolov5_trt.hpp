#ifndef AUTO_AIM__YOLOV5_TRT_HPP
#define AUTO_AIM__YOLOV5_TRT_HPP

#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include <list>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/yolo.hpp"

namespace auto_aim
{
class TRTLogger : public nvinfer1::ILogger
{
  void log(Severity severity, const char * msg) noexcept override
  {
    if (severity <= Severity::kWARNING) std::cerr << "[TRT] " << msg << "\n";
  }
};

class YOLOV5_TRT : public YOLOBase
{
public:
  YOLOV5_TRT(const std::string & config_path, bool debug);
  ~YOLOV5_TRT();

  std::list<Armor> detect(const cv::Mat & bgr_img, int frame_count) override;
  std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) override;

private:
  bool debug_, use_roi_, use_traditional_;
  std::string save_path_;
  double min_confidence_, binary_threshold_;

  const float nms_threshold_ = 0.3;
  const float score_threshold_ = 0.7;
  const int input_h_ = 640, input_w_ = 640;
  // output: [1, 25200, 22] — stays contiguous in output_host_
  const int num_anchors_ = 25200, num_cols_ = 22;

  cv::Rect roi_;
  cv::Point2f offset_;
  cv::Mat tmp_img_;

  Detector detector_;

  TRTLogger trt_logger_;
  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> context_;

  cudaStream_t stream_;
  void * gpu_input_{nullptr};
  void * gpu_output_{nullptr};
  std::vector<float> input_host_;
  std::vector<float> output_host_;

  void preprocess(const cv::Mat & bgr_img, float * dst);

  std::list<Armor> parse(double scale, const cv::Mat & bgr_img, int frame_count);

  bool check_name(const Armor & armor) const;
  bool check_type(const Armor & armor) const;
  cv::Point2f get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const;
  void draw_detections(
    const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const;
  double sigmoid(double x);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__YOLOV5_TRT_HPP
