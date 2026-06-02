#include "yolov5_trt.hpp"

#include <fmt/chrono.h>
#include <yaml-cpp/yaml.h>

#include <fstream>

#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

namespace auto_aim
{

YOLOV5_TRT::YOLOV5_TRT(const std::string & config_path, bool debug)
: debug_(debug), detector_(config_path, false)
{
  auto yaml = YAML::LoadFile(config_path);

  std::string engine_path = yaml["yolov5_trt_engine_path"].as<std::string>();
  binary_threshold_ = yaml["threshold"].as<double>();
  min_confidence_ = yaml["min_confidence"].as<double>();
  use_roi_ = yaml["use_roi"].as<bool>();
  use_traditional_ = yaml["use_traditional"].as<bool>();
  int x = yaml["roi"]["x"].as<int>();
  int y = yaml["roi"]["y"].as<int>();
  int width = yaml["roi"]["width"].as<int>();
  int height = yaml["roi"]["height"].as<int>();
  roi_ = cv::Rect(x, y, width, height);
  offset_ = cv::Point2f(x, y);
  save_path_ = "imgs";

  // Load engine
  std::ifstream file(engine_path, std::ios::binary);
  if (!file) throw std::runtime_error("Cannot open TRT engine: " + engine_path);
  file.seekg(0, std::ios::end);
  size_t size = file.tellg();
  file.seekg(0, std::ios::beg);
  std::vector<char> data(size);
  file.read(data.data(), size);

  runtime_.reset(nvinfer1::createInferRuntime(trt_logger_));
  engine_.reset(runtime_->deserializeCudaEngine(data.data(), size));
  context_.reset(engine_->createExecutionContext());

  if (!context_) throw std::runtime_error("Failed to create TRT execution context");

  // Allocate GPU buffers
  size_t input_size = 1 * 3 * input_h_ * input_w_ * sizeof(float);
  size_t output_size = 1 * num_anchors_ * num_cols_ * sizeof(float);
  cudaMalloc(&gpu_input_, input_size);
  cudaMalloc(&gpu_output_, output_size);
  cudaStreamCreate(&stream_);

  input_host_.resize(1 * 3 * input_h_ * input_w_);
  output_host_.resize(1 * num_anchors_ * num_cols_);

  context_->setTensorAddress("images", gpu_input_);
  context_->setTensorAddress("output", gpu_output_);

  tools::logger()->info("YOLOV5_TRT loaded engine: {}", engine_path);
}

YOLOV5_TRT::~YOLOV5_TRT()
{
  cudaFree(gpu_input_);
  cudaFree(gpu_output_);
  cudaStreamDestroy(stream_);
}

void YOLOV5_TRT::preprocess(const cv::Mat & bgr_img, float * dst)
{
  // Letterbox resize to 640x640 with zero padding
  auto x_scale = static_cast<double>(input_w_) / bgr_img.cols;
  auto y_scale = static_cast<double>(input_h_) / bgr_img.rows;
  auto scale = std::min(x_scale, y_scale);
  int w = static_cast<int>(bgr_img.cols * scale);
  int h = static_cast<int>(bgr_img.rows * scale);

  cv::Mat padded(input_h_, input_w_, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::resize(bgr_img, padded(cv::Rect(0, 0, w, h)), {w, h});

  // BGR u8 → RGB float32 /255, then HWC → CHW
  cv::Mat rgb;
  cv::cvtColor(padded, rgb, cv::COLOR_BGR2RGB);
  rgb.convertTo(rgb, CV_32F, 1.0 / 255.0);

  std::vector<cv::Mat> channels(3);
  cv::split(rgb, channels);
  size_t plane = input_h_ * input_w_;
  for (int c = 0; c < 3; c++)
    memcpy(dst + c * plane, channels[c].data, plane * sizeof(float));
}

std::list<Armor> YOLOV5_TRT::detect(const cv::Mat & raw_img, int frame_count)
{
  if (raw_img.empty()) {
    tools::logger()->warn("Empty img!, camera drop!");
    return {};
  }

  cv::Mat bgr_img = raw_img;
  if (use_roi_) {
    if (roi_.width == -1) roi_.width = raw_img.cols;
    if (roi_.height == -1) roi_.height = raw_img.rows;
    bgr_img = raw_img(roi_);
  }

  auto x_scale = static_cast<double>(input_w_) / bgr_img.cols;
  auto y_scale = static_cast<double>(input_h_) / bgr_img.rows;
  auto scale = std::min(x_scale, y_scale);

  preprocess(bgr_img, input_host_.data());

  cudaMemcpyAsync(
    gpu_input_, input_host_.data(), input_host_.size() * sizeof(float),
    cudaMemcpyHostToDevice, stream_);

  context_->enqueueV3(stream_);

  cudaMemcpyAsync(
    output_host_.data(), gpu_output_, output_host_.size() * sizeof(float),
    cudaMemcpyDeviceToHost, stream_);

  cudaStreamSynchronize(stream_);

  // Wrap output as [25200, 22] cv::Mat (no copy)
  cv::Mat output(num_anchors_, num_cols_, CV_32F, output_host_.data());

  return parse(scale, raw_img, frame_count);
}

std::list<Armor> YOLOV5_TRT::parse(
  double scale, const cv::Mat & bgr_img, int frame_count)
{
  cv::Mat output(num_anchors_, num_cols_, CV_32F, output_host_.data());

  std::vector<int> color_ids, num_ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  std::vector<std::vector<cv::Point2f>> armors_key_points;

  for (int r = 0; r < output.rows; r++) {
    double score = sigmoid(output.at<float>(r, 8));
    if (score < score_threshold_) continue;

    cv::Mat color_scores = output.row(r).colRange(9, 13);
    cv::Mat classes_scores = output.row(r).colRange(13, 22);
    cv::Point class_id, color_id;
    double score_num, score_color;
    cv::minMaxLoc(classes_scores, nullptr, &score_num, nullptr, &class_id);
    cv::minMaxLoc(color_scores, nullptr, &score_color, nullptr, &color_id);

    std::vector<cv::Point2f> kpts;
    kpts.push_back({output.at<float>(r, 0) / (float)scale, output.at<float>(r, 1) / (float)scale});
    kpts.push_back({output.at<float>(r, 6) / (float)scale, output.at<float>(r, 7) / (float)scale});
    kpts.push_back({output.at<float>(r, 4) / (float)scale, output.at<float>(r, 5) / (float)scale});
    kpts.push_back({output.at<float>(r, 2) / (float)scale, output.at<float>(r, 3) / (float)scale});

    float min_x = kpts[0].x, max_x = kpts[0].x;
    float min_y = kpts[0].y, max_y = kpts[0].y;
    for (int i = 1; i < 4; i++) {
      min_x = std::min(min_x, kpts[i].x);
      max_x = std::max(max_x, kpts[i].x);
      min_y = std::min(min_y, kpts[i].y);
      max_y = std::max(max_y, kpts[i].y);
    }

    color_ids.push_back(color_id.x);
    num_ids.push_back(class_id.x);
    confidences.push_back((float)score);
    boxes.emplace_back((int)min_x, (int)min_y, (int)(max_x - min_x), (int)(max_y - min_y));
    armors_key_points.push_back(kpts);
  }

  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_, indices);

  std::list<Armor> armors;
  for (const auto & i : indices) {
    if (use_roi_)
      armors.emplace_back(color_ids[i], num_ids[i], confidences[i], boxes[i], armors_key_points[i], offset_);
    else
      armors.emplace_back(color_ids[i], num_ids[i], confidences[i], boxes[i], armors_key_points[i]);
  }

  tmp_img_ = bgr_img;
  for (auto it = armors.begin(); it != armors.end();) {
    if (!check_name(*it)) { it = armors.erase(it); continue; }
    if (!check_type(*it)) { it = armors.erase(it); continue; }
    if (use_traditional_) detector_.detect(*it, bgr_img);
    it->center_norm = get_center_norm(bgr_img, it->center);
    ++it;
  }

  if (debug_) draw_detections(bgr_img, armors, frame_count);
  return armors;
}

std::list<Armor> YOLOV5_TRT::postprocess(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  return parse(scale, bgr_img, frame_count);
}

bool YOLOV5_TRT::check_name(const Armor & armor) const
{
  return armor.name != ArmorName::not_armor && armor.confidence > min_confidence_;
}

bool YOLOV5_TRT::check_type(const Armor & armor) const
{
  return (armor.type == ArmorType::small)
           ? (armor.name != ArmorName::one && armor.name != ArmorName::base)
           : (armor.name != ArmorName::two && armor.name != ArmorName::sentry &&
              armor.name != ArmorName::outpost);
}

cv::Point2f YOLOV5_TRT::get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const
{
  return {center.x / bgr_img.cols, center.y / bgr_img.rows};
}

void YOLOV5_TRT::draw_detections(
  const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const
{
  auto detection = img.clone();
  tools::draw_text(detection, fmt::format("[{}]", frame_count), {10, 30}, {255, 255, 255});
  for (const auto & armor : armors) {
    auto info = fmt::format(
      "{:.2f} {} {} {}", armor.confidence, COLORS[armor.color], ARMOR_NAMES[armor.name],
      ARMOR_TYPES[armor.type]);
    tools::draw_points(detection, armor.points, {0, 255, 0});
    tools::draw_text(detection, info, armor.center, {0, 255, 0});
  }
  cv::resize(detection, detection, {}, 0.5, 0.5);
  cv::imshow("detection", detection);
}

double YOLOV5_TRT::sigmoid(double x)
{
  return x > 0 ? 1.0 / (1.0 + exp(-x)) : exp(x) / (1.0 + exp(x));
}

}  // namespace auto_aim
