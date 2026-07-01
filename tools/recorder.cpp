#include "recorder.hpp"

#include <fmt/chrono.h>

#include <filesystem>
#include <string>

#include "math_tools.hpp"
#include "tools/logger.hpp"

namespace tools
{
Recorder::Recorder(double fps, double segment_seconds, bool enabled)
: enabled_(enabled),
  thread_started_(false),
  stop_thread_(false),
  fps_(fps),
  segment_seconds_(segment_seconds),
  folder_path_("records"),
  queue_(1)
{
  if (!enabled_) {
    tools::logger()->info("[Recorder] disabled by config");
    return;
  }
  start_time_ = std::chrono::steady_clock::now();
  last_time_ = start_time_;
  std::filesystem::create_directory(folder_path_);
  tools::logger()->info("[Recorder] enabled: fps={}, segment={}s", fps_, segment_seconds_);
}

Recorder::~Recorder()
{
  if (!thread_started_) return;  // 从未录制过, 无线程/文件需要收尾

  stop_thread_ = true;
  // 退出时给队列中额外推入一个空帧，避免pop一直等待
  queue_.push({cv::Mat::zeros(0, 0, 0), {0, 0, 0, 0}, std::chrono::steady_clock::now()});
  if (saving_thread_.joinable()) saving_thread_.join();  // 等待视频保存线程结束
  // 当前段已在save_to_file退出时finalize
}

void Recorder::open_segment(const cv::Mat & img)
{
  // 每段用当前时间命名, 保证段间文件名唯一
  auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
  auto text_path = fmt::format("{}/{}.txt", folder_path_, file_name);
  auto video_path = fmt::format("{}/{}.avi", folder_path_, file_name);

  text_writer_.open(text_path);
  auto fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
  video_writer_ = cv::VideoWriter(video_path, fourcc, fps_, img.size());
  tools::logger()->info("[Recorder] new segment: {}", video_path);
}

void Recorder::close_segment()
{
  // release()会finalize AVI索引, 此后该段可被完整播放/离线读取
  if (text_writer_.is_open()) text_writer_.close();
  if (video_writer_.isOpened()) video_writer_.release();
}

void Recorder::save_to_file()
{
  bool seg_open = false;
  std::chrono::steady_clock::time_point seg_start;

  while (!stop_thread_) {
    FrameData frame;
    queue_.pop(frame);  // 从队列中取出帧数据
    if (frame.img.empty()) {
      tools::logger()->debug("Recorder received empty img. Skip this frame.");
      continue;
    }

    // 分段: 首帧开段; 当前段超过segment_seconds_则finalize并开新段(断电只丢当前段)
    if (!seg_open) {
      open_segment(frame.img);
      seg_start = frame.timestamp;
      seg_open = true;
    } else if (tools::delta_time(frame.timestamp, seg_start) >= segment_seconds_) {
      close_segment();
      open_segment(frame.img);
      seg_start = frame.timestamp;
    }

    // 写入视频文件
    video_writer_.write(frame.img);

    // 写入文本文件（输出顺序为wxyz），时间戳相对全局start_time_, 段间连续
    Eigen::Vector4d xyzw = frame.q.coeffs();
    auto since_begin = tools::delta_time(frame.timestamp, start_time_);
    text_writer_ << fmt::format(
      "{} {} {} {} {}\n", since_begin, xyzw[3], xyzw[0], xyzw[1], xyzw[2]);
    text_writer_.flush();  // 断电保命: 每帧刷新, 保证已落盘视频帧都有对应姿态行
  }

  if (seg_open) close_segment();  // 退出时finalize当前段
}

void Recorder::record(
  const cv::Mat & img, const Eigen::Quaterniond & q,
  const std::chrono::steady_clock::time_point & timestamp)
{
  if (!enabled_ || img.empty()) return;

  // 首次录制时启动保存线程(懒启动, 关闭录制时不建线程)
  if (!thread_started_) {
    saving_thread_ = std::thread(&Recorder::save_to_file, this);
    thread_started_ = true;
  }

  auto since_last = tools::delta_time(timestamp, last_time_);
  if (since_last < 1.0 / fps_) return;

  last_time_ = timestamp;
  queue_.push({img, q, timestamp});
}

}  // namespace tools
