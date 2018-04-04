#ifndef CAPSTONE_VISUALIZER_H_
#define CAPSTONE_VISUALIZER_H_

#include <functional>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace visualizer {

// Point cloud visualizer. Uses the http://pointclouds.org/documentation/tutorials/pcl_visualizer.php#pcl-visualizer
// library.
class Visualizer {
 public:
  Visualizer();

  // Initializes the visualizer with the point cloud contained in the file.
  void AddPointsFromFile(const std::string& file_path);

  void Display();

  // Sets the background color to the given RGB values. These values should be in the range of 0 - 255.
  // The default background color is black.
  void set_background_color(int r, int g, int b);

  void set_on_update_callback(std::function<void()> callback);

  void parse_command_line_args(int argc, char** argv);

  // Whether or not the visualizer should display the X, Y and Z axis's on screen.
  void show_axis(bool show_axis);
 private:
  static const std::string kPointCloudName;
  static constexpr int kUpdateFrequency = 100;
  float z_scale_factor_;
  bool use_color_; // Whether or not this point cloud should display the points in color.
  bool interactive_;
  bool show_axis_;
  bool play_;
  int text_id_;
  int up_x_;
  pcl::visualization::Camera camera_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
  pcl::RGB background_color_;
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color_handler_;
  std::function<void()> on_update_callback_;

  static void KeyboardEventOccured(const pcl::visualization::KeyboardEvent& event, void* viewer_void);
  static void mouseEventOccurred(const pcl::visualization::MouseEvent& event, void* viewer_void);
  void translateViewer(int dx, int dy, int dz);
  void play_or_pause();
  static void log(const std::string& msg);

};

}  // namespace visualizer

#endif  // CAPSTONE_VISUALIZER_H_