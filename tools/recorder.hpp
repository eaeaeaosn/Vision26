#ifndef TOOLS__RECORDER_HPP
#define TOOLS__RECORDER_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

#include "tools/thread_safe_queue.hpp"
namespace tools
{
// 录包器: 录制"视频(.avi) + 姿态四元数(.txt)"用于赛后离线复现.
// - 分段录制: 每segment_seconds秒自动finalize当前段并开新段, 断电时只丢失当前未完成段, 已完成段完整可用.
// - yaml开关: 由上层从配置读取enabled, 关闭时record()为空操作, 不建线程/不写文件.
class Recorder
{
public:
  // fps: 录制帧率(按时间戳节流); segment_seconds: 每段时长(秒); enabled: 是否录制
  Recorder(double fps = 30, double segment_seconds = 60, bool enabled = true);
  ~Recorder();
  void record(
    const cv::Mat & img, const Eigen::Quaterniond & q,
    const std::chrono::steady_clock::time_point & timestamp);

private:
  struct FrameData
  {
    cv::Mat img;
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
  };
  bool enabled_;
  bool thread_started_;
  std::atomic<bool> stop_thread_;
  double fps_;
  double segment_seconds_;
  std::string folder_path_;
  std::ofstream text_writer_;
  cv::VideoWriter video_writer_;
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::steady_clock::time_point last_time_;
  tools::ThreadSafeQueue<FrameData> queue_;
  std::thread saving_thread_;  // 负责保存帧数据的线程
  void open_segment(const cv::Mat & img);
  void close_segment();
  void save_to_file();
};

}  // namespace tools

#endif  // TOOLS__RECORDER_HPP
