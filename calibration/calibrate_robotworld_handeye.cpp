#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>  // 必须在opencv2/core/eigen.hpp上面
#include <fstream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include "tools/img_tools.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ? |                          | 输出命令行参数说明}"
  "{config-path c  | configs/calibration.yaml | yaml配置文件路径 }"
  "{fast f         |                          | 自动翻页，不等待按键 }"
  "{@input-folder  | assets/img_with_q        | 输入文件夹路径   }";

std::vector<cv::Point3f> centers_3d(const cv::Size & pattern_size, const float center_distance)
{
  std::vector<cv::Point3f> centers_3d;

  for (int i = 0; i < pattern_size.height; i++) {
    for (int j = 0; j < pattern_size.width; j++) {
      float x = 0;
      float y = (-j + 0.5 * pattern_size.width) * center_distance;
      float z = (-i + 0.5 * pattern_size.height) * center_distance;
      centers_3d.push_back({x, y, z});
    }
  }

  return centers_3d;
}

Eigen::Quaterniond read_q(const std::string & q_path)
{
  std::ifstream q_file(q_path);
  double w, x, y, z;
  q_file >> w >> x >> y >> z;
  return {w, x, y, z};
}

void load(
  const std::string & input_folder, const std::string & config_path, bool fast,
  std::vector<double> & R_gimbal2imubody_data, std::vector<cv::Mat> & R_world2gimbal_list,
  std::vector<cv::Mat> & t_world2gimbal_list, std::vector<cv::Mat> & rvecs,
  std::vector<cv::Mat> & tvecs)
{
  // 读取yaml参数
  auto yaml = YAML::LoadFile(config_path);
  auto pattern_cols = yaml["pattern_cols"].as<int>();
  auto pattern_rows = yaml["pattern_rows"].as<int>();
  auto center_distance_mm = yaml["center_distance_mm"].as<double>();
  R_gimbal2imubody_data = yaml["R_gimbal2imubody"].as<std::vector<double>>();
  auto camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();
  auto distort_coeffs_data = yaml["distort_coeffs"].as<std::vector<double>>();

  cv::Size pattern_size(pattern_cols, pattern_rows);
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R_gimbal2imubody(R_gimbal2imubody_data.data());
  cv::Matx33d camera_matrix(camera_matrix_data.data());
  cv::Mat distort_coeffs(distort_coeffs_data);

  Eigen::Vector3d ypr_min{1e9, 1e9, 1e9}, ypr_max{-1e9, -1e9, -1e9};

  for (int i = 1; true; i++) {
    // 读取图片和对应四元数
    auto img_path = fmt::format("{}/{}.jpg", input_folder, i);
    auto q_path = fmt::format("{}/{}.txt", input_folder, i);
    auto img = cv::imread(img_path);
    Eigen::Quaterniond q = read_q(q_path);
    if (img.empty()) break;

    // 计算云台的欧拉角
    Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();
    Eigen::Matrix3d R_gimbal2world =
      R_gimbal2imubody.transpose() * R_imubody2imuabs * R_gimbal2imubody;
    Eigen::Vector3d ypr = tools::eulers(R_gimbal2world, 2, 1, 0) * 57.3;  // degree

    // 在图片上显示云台的欧拉角，用来检验R_gimbal2imubody是否正确
    auto drawing = img.clone();
    tools::draw_text(drawing, fmt::format("yaw   {:.2f}", ypr[0]), {40, 40}, {0, 0, 255});
    tools::draw_text(drawing, fmt::format("pitch {:.2f}", ypr[1]), {40, 80}, {0, 0, 255});
    tools::draw_text(drawing, fmt::format("roll  {:.2f}", ypr[2]), {40, 120}, {0, 0, 255});

    // 识别标定板（棋盘格内角点）
    std::vector<cv::Point2f> centers_2d;
    auto success = cv::findChessboardCorners(img, pattern_size, centers_2d);

    // 亚像素级角点精修
    if (success) {
      cv::Mat gray;
      cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
      cv::cornerSubPix(
        gray, centers_2d, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));
    }

    // 显示识别结果
    cv::drawChessboardCorners(drawing, pattern_size, centers_2d, success);
    cv::resize(drawing, drawing, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("Press any to continue", drawing);
    cv::waitKey(fast ? 1 : 0);

    // 输出识别结果
    fmt::print("[{}] {}\n", success ? "success" : "failure", img_path);
    if (!success) continue;

    // 计算所需的数据
    Eigen::Matrix3d R_world2gimbal = R_gimbal2world.transpose();
    cv::Mat t_world2gimbal = (cv::Mat_<double>(3, 1) << 0, 0, 0);
    cv::Mat R_world2gimbal_cv;
    cv::eigen2cv(R_world2gimbal, R_world2gimbal_cv);
    cv::Mat rvec, tvec;
    auto centers_3d_ = centers_3d(pattern_size, center_distance_mm);
    cv::solvePnP(
      centers_3d_, centers_2d, camera_matrix, distort_coeffs, rvec, tvec, false,
      cv::SOLVEPNP_ITERATIVE);

    // 计算重投影误差，用来检查solvePnP是否得到了翻转/错误的位姿
    std::vector<cv::Point2f> reprojected;
    cv::projectPoints(centers_3d_, rvec, tvec, camera_matrix, distort_coeffs, reprojected);
    double reproj_err = cv::norm(centers_2d, reprojected, cv::NORM_L2) / std::sqrt(reprojected.size());
    fmt::print("[{}] reproj_err {:.3f}px, tvec [{:.1f}, {:.1f}, {:.1f}]mm\n", img_path,
      reproj_err, tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));

    // 剔除重投影误差过大的帧（多为运动模糊/位姿翻转，会污染手眼标定）
    constexpr double max_reproj_err = 1.5;  // px
    if (reproj_err > max_reproj_err) {
      fmt::print("  -> skipped (reproj_err > {:.1f}px)\n", max_reproj_err);
      continue;
    }

    // 记录所需的数据
    R_world2gimbal_list.emplace_back(R_world2gimbal_cv);
    t_world2gimbal_list.emplace_back(t_world2gimbal);
    rvecs.emplace_back(rvec);
    tvecs.emplace_back(tvec);

    // 统计被采纳帧的云台角度范围，用来判断旋转激励是否足够
    ypr_min = ypr_min.cwiseMin(ypr);
    ypr_max = ypr_max.cwiseMax(ypr);
  }

  fmt::print(
    "\n[diag] accepted {} frames, gimbal angle span (deg): "
    "yaw [{:.1f}, {:.1f}] = {:.1f}, pitch [{:.1f}, {:.1f}] = {:.1f}, roll [{:.1f}, {:.1f}] = {:.1f}\n",
    rvecs.size(), ypr_min[0], ypr_max[0], ypr_max[0] - ypr_min[0], ypr_min[1], ypr_max[1],
    ypr_max[1] - ypr_min[1], ypr_min[2], ypr_max[2], ypr_max[2] - ypr_min[2]);
}

void print_yaml(
  const std::vector<double> & R_gimbal2imubody_data, const cv::Mat & R_camera2gimbal,
  const cv::Mat & t_camera2gimbal, const Eigen::Vector3d & camera_ypr, double distance,
  const Eigen::Vector3d & board_ypr)
{
  YAML::Emitter result;
  std::vector<double> R_camera2gimbal_data(
    R_camera2gimbal.begin<double>(), R_camera2gimbal.end<double>());
  std::vector<double> t_camera2gimbal_data(
    t_camera2gimbal.begin<double>(), t_camera2gimbal.end<double>());

  result << YAML::BeginMap;
  result << YAML::Key << "R_gimbal2imubody";
  result << YAML::Value << YAML::Flow << R_gimbal2imubody_data;
  result << YAML::Newline;
  result << YAML::Newline;
  result << YAML::Comment(fmt::format(
    "相机同理想情况的偏角: yaw{:.2f} pitch{:.2f} roll{:.2f} degree", camera_ypr[0], camera_ypr[1],
    camera_ypr[2]));
  result << YAML::Newline;
  result << YAML::Comment(fmt::format("标定板到世界坐标系原点的水平距离: {:.2f} m", distance));
  result << YAML::Newline;
  result << YAML::Comment(fmt::format(
    "标定板同竖直摆放时的偏角: yaw{:.2f} pitch{:.2f} roll{:.2f} degree", board_ypr[0], board_ypr[1],
    board_ypr[2]));
  result << YAML::Key << "R_camera2gimbal";
  result << YAML::Value << YAML::Flow << R_camera2gimbal_data;
  result << YAML::Key << "t_camera2gimbal";
  result << YAML::Value << YAML::Flow << t_camera2gimbal_data;
  result << YAML::Newline;
  result << YAML::EndMap;

  fmt::print("\n{}\n", result.c_str());
}

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto input_folder = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");
  auto fast = cli.has("fast");

  // 从输入文件夹中加载标定所需的数据
  std::vector<double> R_gimbal2imubody_data;
  std::vector<cv::Mat> R_world2gimbal_list, t_world2gimbal_list;
  std::vector<cv::Mat> rvecs, tvecs;
  load(
    input_folder, config_path, fast, R_gimbal2imubody_data, R_world2gimbal_list,
    t_world2gimbal_list, rvecs, tvecs);

  // 诊断：对比 SHAH 与 LI 两种闭式解，打印相机相对理想朝向的 yaw
  // （健康结果 yaw 应接近 0；接近 ±180 表示解到了"相机朝后"的错误分支）
  Eigen::Matrix3d R_gimbal2ideal_dbg{{0, -1, 0}, {0, 0, -1}, {1, 0, 0}};
  for (auto [name, method] :
       {std::pair{"SHAH", cv::CALIB_ROBOT_WORLD_HAND_EYE_SHAH},
        std::pair{"LI", cv::CALIB_ROBOT_WORLD_HAND_EYE_LI}}) {
    try {
      cv::Mat R_g2c, t_g2c, R_w2b, t_w2b;
      cv::calibrateRobotWorldHandEye(
        rvecs, tvecs, R_world2gimbal_list, t_world2gimbal_list, R_w2b, t_w2b, R_g2c, t_g2c, method);
      cv::Mat R_c2g;
      cv::transpose(R_g2c, R_c2g);
      Eigen::Matrix3d R_c2g_eigen;
      cv::cv2eigen(R_c2g, R_c2g_eigen);
      Eigen::Vector3d ypr = tools::eulers(R_gimbal2ideal_dbg * R_c2g_eigen, 1, 0, 2) * 57.3;
      fmt::print(
        "[diag] method={} camera-vs-ideal yaw {:.2f} pitch {:.2f} roll {:.2f}\n", name, ypr[0],
        ypr[1], ypr[2]);
    } catch (const cv::Exception & e) {
      fmt::print("[diag] method={} FAILED: {}\n", name, e.what());
    }
  }

  // 手眼标定
  cv::Mat R_gimbal2camera, t_gimbal2camera;
  cv::Mat R_world2board, t_world2board;
  cv::calibrateRobotWorldHandEye(
    rvecs, tvecs, R_world2gimbal_list, t_world2gimbal_list, R_world2board, t_world2board,
    R_gimbal2camera, t_gimbal2camera);
  t_gimbal2camera /= 1e3;  // mm to m
  t_world2board /= 1e3;    // mm to m

  // 计算所需的数据
  cv::Mat R_camera2gimbal, t_camera2gimbal;
  cv::Mat R_board2world, t_board2world;
  cv::transpose(R_gimbal2camera, R_camera2gimbal);
  cv::transpose(R_world2board, R_board2world);
  t_camera2gimbal = -R_camera2gimbal * t_gimbal2camera;
  t_board2world = -R_board2world * t_world2board;

  // 计算相机同理想情况的偏角
  Eigen::Matrix3d R_camera2gimbal_eigen;
  cv::cv2eigen(R_camera2gimbal, R_camera2gimbal_eigen);
  Eigen::Matrix3d R_gimbal2ideal{{0, -1, 0}, {0, 0, -1}, {1, 0, 0}};
  Eigen::Matrix3d R_camera2ideal = R_gimbal2ideal * R_camera2gimbal_eigen;
  Eigen::Vector3d camera_ypr = tools::eulers(R_camera2ideal, 1, 0, 2) * 57.3;  // degree

  // 计算标定板到世界坐标系原点的水平距离
  auto x = t_board2world.at<double>(0);
  auto y = t_board2world.at<double>(1);
  auto distance = std::sqrt(x * x + y * y);

  // 计算标定板同竖直摆放时的偏角
  Eigen::Matrix3d R_board2world_eigen;
  cv::cv2eigen(R_board2world, R_board2world_eigen);
  Eigen::Vector3d board_ypr = tools::eulers(R_board2world_eigen, 2, 1, 0) * 57.3;  // degree

  // 输出yaml
  print_yaml(
    R_gimbal2imubody_data, R_camera2gimbal, t_camera2gimbal, camera_ypr, distance, board_ypr);
}
